"""
mock_fr5.py (WBS 1.7)

실제 FR5 로봇 없이 /fr5/task 액션 서버를 흉내 내는 목업 노드.

목적: orchestrator_node.py(1.8)의 전체 흐름과 job_runner.py(1.4)의
"PICK 실패는 재시도, PLACE 실패는 즉시 ERROR" 로직을, 실제 로봇/그리퍼
하드웨어 없이 ROS2 액션 레벨에서 끝까지(1.9 mock.launch.py) 검증하기 위함이다.

실패 주입 파라미터:
  fail_phase  (string, 기본 ""): 실패시킬 phase ("PICK" 또는 "PLACE"). 빈 문자열이면
              절대 실패하지 않는다.
  fail_count  (int, 기본 0): 몇 번 연속으로 실패시킬지. 0이면 fail_phase를 줘도 무시된다.
              예: fail_phase:=PICK fail_count:=2 -> 처음 2번 호출은 PICK에서 실패하고,
              3번째 호출부터는 성공한다. job_builder.py가 FR5_TASK Step에
              max_retries=2를 주고 있으므로, 이 조합으로 "재시도 2번 만에 성공"
              시나리오를 그대로 재현할 수 있다. fail_count를 3 이상으로 주면
              max_retries를 넘겨서 Job이 ERROR로 멈추는 경로를 재현한다.

실행 예:
  ros2 run factory_mock mock_fr5
  ros2 run factory_mock mock_fr5 --ros-args -p fail_phase:=PICK -p fail_count:=2
  ros2 run factory_mock mock_fr5 --ros-args -p fail_phase:=PLACE -p fail_count:=1

참고: 이 환경에는 rclpy/factory_interfaces 빌드 결과물이 없어서 실제로 실행해
검증하지는 못했다. ActionServer 콜백 시그니처와 goal_handle API는 ROS2(rclpy)
표준 패턴을 그대로 따랐지만, 실제 ROS2 환경에서 `colcon build` 후 한 번
돌려보고 (특히 goal_handle.publish_feedback / succeed / canceled 호출부) 문제가
있으면 그때 고치는 걸 권장한다.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from factory_interfaces.action import Fr5Task

PHASES = [
    Fr5Task.Result.PHASE_PICK,
    Fr5Task.Result.PHASE_PLACE,
    Fr5Task.Result.PHASE_HOME,
]
PHASE_DURATIONS_S = {
    Fr5Task.Result.PHASE_PICK: 1.0,
    Fr5Task.Result.PHASE_PLACE: 1.0,
    Fr5Task.Result.PHASE_HOME: 0.5,
}


class MockFr5Node(Node):
    def __init__(self):
        super().__init__("mock_fr5")

        self.declare_parameter("fail_phase", "")
        self.declare_parameter("fail_count", 0)

        self._fail_phase = self.get_parameter("fail_phase").value
        self._fail_count = self.get_parameter("fail_count").value
        self._attempts_failed = 0

        self._cb_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            Fr5Task,
            "/fr5/task",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"mock_fr5 시작됨 (fail_phase={self._fail_phase!r}, fail_count={self._fail_count})"
        )

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"FR5 작업 시작: {goal.task_type} level={goal.level}")

        # 이번 실행에서 실패를 주입할지는 "콜백 시작 시점"의 누적 실패 횟수로 미리 정한다.
        # (중간에 바뀌면 안 되므로 한 번만 평가)
        will_fail = (
            self._fail_phase in (Fr5Task.Result.PHASE_PICK, Fr5Task.Result.PHASE_PLACE)
            and self._attempts_failed < self._fail_count
        )

        for phase in PHASES:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(False, "", "CANCELLED")

            feedback = Fr5Task.Feedback()
            feedback.phase = phase
            goal_handle.publish_feedback(feedback)
            time.sleep(PHASE_DURATIONS_S[phase])

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(False, "", "CANCELLED")

            if will_fail and phase == self._fail_phase:
                self._attempts_failed += 1
                self.get_logger().warn(
                    f"[목업 실패 주입] {phase} 단계에서 실패 처리 "
                    f"({self._attempts_failed}/{self._fail_count})"
                )
                goal_handle.succeed()  # 액션 자체는 정상 종료, result.success=False로 실패를 표현
                return self._make_result(False, phase, f"MOCK_FAIL_AT_{phase}")

        goal_handle.succeed()
        return self._make_result(True, "", "")

    @staticmethod
    def _make_result(success: bool, failed_phase: str, message: str) -> Fr5Task.Result:
        result = Fr5Task.Result()
        result.success = success
        result.failed_phase = failed_phase
        result.message = message
        return result


def main():
    rclpy.init()
    node = MockFr5Node()
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
