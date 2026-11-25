"""
mock_limo.py (WBS 1.7)

실제 Limo Pro 없이 /limo/go_to_station 액션 서버를 흉내 내는 목업 노드.

GoToStation Step의 retry_policy는 job_builder.py에서 항상 기본값(NEVER)이므로,
이 목업에서 실패를 주입하면 "재시도"가 아니라 오케스트레이터가 바로 Job을
ERROR로 멈추고 사람이 /factory/retry_or_cancel로 처리해야 하는 경로를 검증하게
된다. 즉 mock_fr5가 "재시도 끝에 성공" 경로를, mock_limo는 "사람 개입 필요" 경로를
검증하는 역할 분담이다.

실패 주입 파라미터:
  fail_once (bool, 기본 false): true면 ArUco를 못 찾은 상황(MARKER_NOT_FOUND)으로
             딱 한 번만 실패하고, 그 다음 호출부터는 정상 동작한다
             (사람이 /factory/retry_or_cancel로 RETRY를 호출하는 시나리오 재현용).
  travel_duration_s (float, 기본 1.5): 평이동(coarse) 구간 시뮬레이션 시간
  dock_duration_s   (float, 기본 1.0): ArUco 도킹 구간 시뮬레이션 시간

실행 예:
  ros2 run factory_mock mock_limo
  ros2 run factory_mock mock_limo --ros-args -p fail_once:=true

참고: 이 환경에는 rclpy/factory_interfaces 빌드 결과물이 없어서 실제로 실행해
검증하지는 못했다 (mock_fr5.py와 동일한 제약). 실제 ROS2 환경에서 한 번
돌려보는 걸 권장한다.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from factory_interfaces.action import GoToStation


class MockLimoNode(Node):
    def __init__(self):
        super().__init__("mock_limo")

        self.declare_parameter("fail_once", False)
        self.declare_parameter("travel_duration_s", 1.5)
        self.declare_parameter("dock_duration_s", 1.0)

        self._fail_once = self.get_parameter("fail_once").value
        self._travel_duration_s = self.get_parameter("travel_duration_s").value
        self._dock_duration_s = self.get_parameter("dock_duration_s").value
        self._already_failed = False

        self._cb_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            GoToStation,
            "/limo/go_to_station",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f"mock_limo 시작됨 (fail_once={self._fail_once})")

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f"Limo 이동 시작: station={goal.station}")

        feedback = GoToStation.Feedback()
        feedback.phase = GoToStation.Feedback.PHASE_COARSE
        feedback.distance_remaining = 1.0
        goal_handle.publish_feedback(feedback)
        time.sleep(self._travel_duration_s)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._make_result(False, "CANCELLED")

        if self._fail_once and not self._already_failed:
            self._already_failed = True
            self.get_logger().warn("[목업 실패 주입] ArUco 마커를 찾지 못함")
            feedback.phase = GoToStation.Feedback.PHASE_NOT_FOUND
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()  # 액션 자체는 정상 종료, result.success=False로 실패를 표현
            return self._make_result(False, "MARKER_NOT_FOUND")

        feedback.phase = GoToStation.Feedback.PHASE_DOCKING
        feedback.distance_remaining = 0.1
        goal_handle.publish_feedback(feedback)
        time.sleep(self._dock_duration_s)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return self._make_result(False, "CANCELLED")

        goal_handle.succeed()
        return self._make_result(True, "", final_error_m=0.01, final_error_yaw=0.02)

    @staticmethod
    def _make_result(
        success: bool, message: str, final_error_m: float = 0.0, final_error_yaw: float = 0.0
    ) -> GoToStation.Result:
        result = GoToStation.Result()
        result.success = success
        result.message = message
        result.final_error_m = final_error_m
        result.final_error_yaw = final_error_yaw
        return result


def main():
    rclpy.init()
    node = MockLimoNode()
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
