"""
fr5_node.py (WBS 3.4)

Fr5Task 액션 서버의 ROS2 래퍼. 실제 로직은 fr5_controller.py
(Fr5TaskExecutor, ROS 의존성 없음)에 전부 있고, 이 파일은:
  - config/fr5.yaml을 읽어서 Fr5Hardware(실제 Fairino SDK 래퍼)를 만들고
  - Fr5Task 액션 요청을 Fr5TaskExecutor.run() 호출로 연결하고
  - 단계(PICK/PLACE/HOME) 진입마다 feedback을 publish하고
  - execute_cb 맨 앞에서 매번(멱등) 에러 리셋을 호출한다
    (orchestrator_node.py 설계 의도: 오케스트레이터는 재시도인지 여부를
    몰라도 되고, 로봇 쪽에서 항상 안전하게 리셋부터 하고 시작한다).

MoveJ(blendT=-1 기본값)는 블로킹 호출이라, execute_cb 자체가 오래 걸린다.
따라서 다른 액션 요청(동시에 들어올 일은 설계상 거의 없지만)이나 노드의
다른 콜백이 막히지 않도록 MultiThreadedExecutor + ReentrantCallbackGroup을
쓴다 (go_to_station_server.py와 동일한 패턴).

⚠️ 알려진 제한: 진행 중인 MoveJ를 cancel_goal_async로 중간에 멈추는 기능은
아직 없다 (Fairino SDK의 정지 명령을 아직 안 붙임). 지금은 한 번 시작된
PICK/PLACE/HOME 동작은 끝까지 블로킹으로 실행된다 — 오케스트레이터의
타임아웃은 걸리지만, 로봇이 실제로 그 자리에서 멈추진 않는다는 뜻이다.
"""

import yaml
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from factory_interfaces.action import Fr5Task
from factory_fr5.fairino_client import Fr5Hardware
from factory_fr5.fr5_controller import Fr5TaskExecutor


class Fr5Node(Node):
    def __init__(self):
        super().__init__("fr5_node")

        self.declare_parameter("robot_ip", "192.168.58.2")
        self.declare_parameter("config_path", "config/fr5.yaml")

        robot_ip = self.get_parameter("robot_ip").value
        config_path = self.get_parameter("config_path").value
        with open(config_path, encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

        self.get_logger().info(f"FR5 연결 시도 중... ({robot_ip})")
        self.hw = Fr5Hardware(robot_ip, self.config)
        self.get_logger().info("FR5 연결 및 그리퍼 설정 완료")

        self.controller = Fr5TaskExecutor(self.hw, self.config)

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            Fr5Task,
            "/fr5/task",
            self._execute_cb,
            callback_group=self._cb_group,
        )

    def _execute_cb(self, goal_handle):
        # 멱등 에러 리셋: 재시도로 들어왔는지 처음 호출인지 이 노드는
        # 신경 쓰지 않는다 — 매번 안전하게 리셋부터 하고 시작한다.
        self.hw.reset_errors()

        feedback = Fr5Task.Feedback()

        def on_phase(phase: str) -> None:
            feedback.phase = phase
            goal_handle.publish_feedback(feedback)

        request = goal_handle.request
        success, failed_phase, message = self.controller.run(
            request.task_type, request.level, on_phase=on_phase,
        )

        result = Fr5Task.Result()
        result.success = success
        result.failed_phase = failed_phase
        result.message = message

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def destroy_node(self):
        self.hw.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Fr5Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
