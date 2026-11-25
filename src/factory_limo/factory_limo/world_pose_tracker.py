"""
world_pose_tracker.py (WBS 2.6)

Limo의 "world 프레임 기준 대략적인 위치(x, y, yaw)"를 추정해서 Unity 쪽
LimoTwin(5.4)에 넘겨주기 위한 노드. SLAM 없이 odometry만 적분하면 시간이
지날수록 오차가 누적되는데(odometry drift), 이 프로젝트는 go_to_station_server가
"도킹 성공"을 보고할 때마다 그 station의 알려진 world 좌표로 추정치를 스냅해서
오차를 주기적으로 리셋한다 — 매번 정밀 위치를 재는 SLAM 대신, "도킹할 때마다
한 번씩 보정되는 적당히 정확한 추정"으로 타협한 것 (3일 일정에 맞는 선택).

이 파일은 두 부분으로 나뉜다:
  - WorldPoseEstimator: 순수 계산 로직 (ROS 의존성 없음, 이 환경에서도 테스트 가능)
  - WorldPoseTrackerNode: 위 로직을 odometry 구독 + PoseStamped 발행으로 감싸는
    ROS2 노드 (rclpy가 없으면 import만 건너뛰고, 로직 자체는 그대로 테스트 가능하게
    가드 처리했다)
"""

import math
from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
    from tf_transformations import euler_from_quaternion, quaternion_from_euler

    _ROS_AVAILABLE = True
except ImportError:
    # 이 샌드박스에는 ROS2가 없다. WorldPoseEstimator(순수 로직)는 그래도
    # import/테스트가 가능하도록, ROS 노드 쪽만 조용히 비활성화한다.
    _ROS_AVAILABLE = False
    Node = object  # type: ignore


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float  # radian


def _wrap_angle(angle: float) -> float:
    """각도를 (-pi, pi] 범위로 정규화."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


class WorldPoseEstimator:
    def __init__(self, initial_pose: Pose2D, station_world_poses: dict[str, dict]):
        self.pose = initial_pose
        self.station_world_poses = station_world_poses
        self._last_odom: Optional[tuple[float, float, float]] = None

    def on_odom(self, odom_x: float, odom_y: float, odom_yaw: float) -> Pose2D:
        """odom 좌표계에서의 raw (x, y, yaw)를 받아, 이전 값과의 변화량만큼 world
        좌표에 누적한다 (odom 원점/방향이 world와 다를 수 있으므로 '변화량'만 쓴다)."""
        if self._last_odom is not None:
            dx = odom_x - self._last_odom[0]
            dy = odom_y - self._last_odom[1]
            dyaw = _wrap_angle(odom_yaw - self._last_odom[2])

            cos_y = math.cos(self.pose.yaw)
            sin_y = math.sin(self.pose.yaw)
            self.pose = Pose2D(
                x=self.pose.x + dx * cos_y - dy * sin_y,
                y=self.pose.y + dx * sin_y + dy * cos_y,
                yaw=_wrap_angle(self.pose.yaw + dyaw),
            )

        self._last_odom = (odom_x, odom_y, odom_yaw)
        return self.pose

    def on_docked(self, station: str) -> Optional[Pose2D]:
        """도킹 성공 시, 이 station의 알려진 world 좌표로 추정치를 스냅해서
        누적된 odometry 오차를 제거한다. 모르는 station이면 아무것도 안 한다."""
        wp = self.station_world_poses.get(station)
        if wp is None:
            return None
        self.pose = Pose2D(x=wp["x"], y=wp["y"], yaw=wp["yaw"])
        return self.pose


if _ROS_AVAILABLE:

    class WorldPoseTrackerNode(Node):
        def __init__(self):
            super().__init__("world_pose_tracker")

            self.declare_parameter("config_path", "config/limo.yaml")
            self.declare_parameter("initial_x", 0.0)
            self.declare_parameter("initial_y", 0.0)
            self.declare_parameter("initial_yaw", 0.0)

            import yaml  # 지역 import: ROS 환경에서만 필요

            config_path = self.get_parameter("config_path").value
            with open(config_path, encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
            station_world_poses = config.get("station_world_poses", {})

            initial_pose = Pose2D(
                x=self.get_parameter("initial_x").value,
                y=self.get_parameter("initial_y").value,
                yaw=self.get_parameter("initial_yaw").value,
            )
            self._estimator = WorldPoseEstimator(initial_pose, station_world_poses)

            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self.create_subscription(String, "/limo/docked_at", self._on_docked, 10)
            self._pose_pub = self.create_publisher(PoseStamped, "/limo/world_pose", 10)

            self.get_logger().info("world_pose_tracker 시작됨")

        def _on_odom(self, msg: Odometry) -> None:
            q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            pose = self._estimator.on_odom(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self._publish(pose)

        def _on_docked(self, msg: String) -> None:
            pose = self._estimator.on_docked(msg.data)
            if pose is not None:
                self.get_logger().info(f"world_pose를 station={msg.data}로 스냅: {pose}")
                self._publish(pose)

        def _publish(self, pose: Pose2D) -> None:
            out = PoseStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "world"
            out.pose.position.x = pose.x
            out.pose.position.y = pose.y
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, pose.yaw)
            out.pose.orientation.x = qx
            out.pose.orientation.y = qy
            out.pose.orientation.z = qz
            out.pose.orientation.w = qw
            self._pose_pub.publish(out)

    def main():
        rclpy.init()
        node = WorldPoseTrackerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

else:

    def main():  # pragma: no cover
        raise RuntimeError("rclpy를 찾을 수 없습니다. 실제 ROS2 환경에서 실행하세요.")


if __name__ == "__main__":
    main()
