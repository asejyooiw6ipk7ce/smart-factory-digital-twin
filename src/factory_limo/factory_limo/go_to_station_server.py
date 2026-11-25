"""
go_to_station_server.py (WBS 2.5)

GoToStation 액션 하나만 외부(오케스트레이터)에 노출하는 단일 진입점 노드.
내부적으로 coarse_mover(2.4, 대략 이동)로 목표 ArUco 마커 근처까지 간 다음,
aruco_docker(2.3, 정밀 P-제어)로 넘겨서 정확히 도킹시킨다.

멘토 피드백 반영: 원래는 "대략 이동 노드"와 "정밀 도킹 노드"를 따로 두고
오케스트레이터가 둘을 순서대로 호출하는 설계였는데, 그러면 오케스트레이터가
Limo의 내부 사정(coarse->fine 전환 시점)까지 알아야 해서 책임이 새어나간다.
그래서 이 노드 하나가 GoToStation 액션 뒤에서 coarse->fine 전환을 전부
숨기고, 오케스트레이터는 "station 이름 하나 던지고 성공/실패만 받는" 것으로
단순화했다 (orchestrator_node.py의 _execute_step이 GO_TO를 다루는 방식 그대로).

실행 모델: execute_callback 안에서 time.sleep으로 제어 루프를 도므로(블로킹),
취소 요청(is_cancel_requested)에 제때 응답하려면 MultiThreadedExecutor +
별도 콜백 그룹이 필요하다 (fr5_node.py와 같은 패턴).

참고: 이 환경에는 rclpy가 없어서 실제로 실행해 검증하지는 못했다. 단,
coarse_mover.py/aruco_docker.py의 핵심 제어 로직 자체는 이 파일과 분리된
순수 로직이라 이미 유닛 테스트(16개)로 검증을 마쳤다 — 여기서 추가로
검증이 필요한 부분은 ROS 토픽 파싱/퍼블리시 배선뿐이다.
"""

import json
import time
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from factory_interfaces.action import GoToStation
from factory_limo.aruco_docker import ArucoDocker, DockConfig, DockResult
from factory_limo.coarse_mover import CoarseMover, CoarseMoveSegment, CoarseResult


class GoToStationServer(Node):
    def __init__(self):
        super().__init__("go_to_station_server")

        self.declare_parameter("config_path", "config/limo.yaml")
        config_path = self.get_parameter("config_path").value
        config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}

        self._marker_ids: dict[str, int] = config.get("marker_ids", {})
        routes = {
            station: [CoarseMoveSegment(**seg) for seg in segs]
            for station, segs in config.get("coarse_routes", {}).items()
        }
        self._coarse = CoarseMover(routes, handoff_distance_m=config.get("handoff_distance_m", 0.6))
        self._docker = ArucoDocker(DockConfig(**config.get("dock", {})))

        self._latest_detections: list[dict] = []
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._docked_pub = self.create_publisher(String, "/limo/docked_at", 10)
        self.create_subscription(String, "/limo/aruco_detections", self._on_detections, 10)

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
        self.get_logger().info("go_to_station_server 시작됨")

    def _on_detections(self, msg: String) -> None:
        try:
            self._latest_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"aruco_detections JSON 파싱 실패: {msg.data!r}")

    def _get_detection_for(self, marker_id: int) -> Optional[dict]:
        for d in self._latest_detections:
            if d.get("id") == marker_id:
                return d
        return None

    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _goal_cb(self, goal_request):
        if goal_request.station not in self._marker_ids:
            self.get_logger().warn(f"알 수 없는 station: {goal_request.station!r}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        station = goal_handle.request.station
        marker_id = self._marker_ids[station]
        self.get_logger().info(f"Limo 이동 시작: station={station} (marker_id={marker_id})")

        coarse_result = self._coarse.run(
            station=station,
            cmd_vel=self._publish_cmd_vel,
            get_latest_detection=lambda: self._get_detection_for(marker_id),
            sleep_fn=time.sleep,
            is_cancelled=lambda: goal_handle.is_cancel_requested,
        )

        if coarse_result == CoarseResult.CANCELLED:
            goal_handle.canceled()
            return self._make_result(False, "CANCELLED")
        if coarse_result == CoarseResult.TIMEOUT:
            goal_handle.succeed()
            return self._make_result(False, "MARKER_NOT_FOUND")

        # 정밀 도킹 루프 (coarse_result == HANDED_OFF)
        self._docker.reset()
        rate_hz = 10.0
        dt = 1.0 / rate_hz
        while True:
            if goal_handle.is_cancel_requested:
                self._publish_cmd_vel(0.0, 0.0)
                goal_handle.canceled()
                return self._make_result(False, "CANCELLED")

            detection = self._get_detection_for(marker_id)
            result, linear_x, angular_z = self._docker.step(detection, dt)

            if result is None:
                self._publish_cmd_vel(linear_x, angular_z)
                feedback = GoToStation.Feedback()
                feedback.phase = GoToStation.Feedback.PHASE_DOCKING
                feedback.distance_remaining = (
                    detection["distance_m"] if detection else -1.0
                )
                goal_handle.publish_feedback(feedback)
                time.sleep(dt)
                continue

            if result == DockResult.MARKER_NOT_FOUND:
                goal_handle.succeed()
                return self._make_result(False, "MARKER_NOT_FOUND")

            # DockResult.SUCCESS
            goal_handle.succeed()
            docked_msg = String()
            docked_msg.data = station
            self._docked_pub.publish(docked_msg)  # world_pose_tracker가 이걸 듣고 드리프트 보정
            last = self._get_detection_for(marker_id) or {}
            return self._make_result(
                True,
                "",
                final_error_m=abs(last.get("distance_m", 0.0) - self._docker.config.target_distance_m),
                final_error_yaw=abs(last.get("yaw_rad", 0.0)),
            )

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
    node = GoToStationServer()
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
