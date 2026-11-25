"""
test_world_pose_tracker.py (WBS 2.6에 딸린 유닛 테스트)

WorldPoseEstimator(순수 로직)만 검증한다. ROS 노드 부분(WorldPoseTrackerNode)은
이 환경에 rclpy가 없어서 애초에 정의되지 않으므로 테스트 대상이 아니다.

실행:
    cd factory_ws/src/factory_limo
    PYTHONPATH=. python3 -m pytest test/test_world_pose_tracker.py -v
"""

import math

from factory_limo.world_pose_tracker import Pose2D, WorldPoseEstimator


def test_first_odom_sample_does_not_move_pose():
    est = WorldPoseEstimator(Pose2D(0.0, 0.0, 0.0), station_world_poses={})
    pose = est.on_odom(1.0, 2.0, 0.3)
    # 첫 샘플은 "기준점"으로만 기록되고, 변화량이 없으므로 pose는 그대로여야 함
    assert pose.x == 0.0 and pose.y == 0.0 and pose.yaw == 0.0


def test_straight_line_motion_accumulates_in_world_frame():
    est = WorldPoseEstimator(Pose2D(0.0, 0.0, 0.0), station_world_poses={})
    est.on_odom(0.0, 0.0, 0.0)
    pose = est.on_odom(1.0, 0.0, 0.0)  # odom 기준 x방향으로 1m 이동
    assert pose.x == 1.0
    assert abs(pose.y) < 1e-9


def test_rotation_changes_subsequent_translation_direction():
    est = WorldPoseEstimator(Pose2D(0.0, 0.0, 0.0), station_world_poses={})
    est.on_odom(0.0, 0.0, 0.0)
    est.on_odom(0.0, 0.0, math.pi / 2)  # 제자리에서 90도 회전 (world yaw가 90도가 됨)
    pose = est.on_odom(1.0, 0.0, math.pi / 2)  # odom 기준 "앞으로 1m"
    # world yaw가 90도이므로, odom의 +x 전진이 world의 +y 방향으로 나타나야 함
    assert abs(pose.x) < 1e-9
    assert abs(pose.y - 1.0) < 1e-9


def test_on_docked_snaps_to_known_station_pose():
    station_world_poses = {"LEVEL_1": {"x": 5.0, "y": 1.0, "yaw": 1.57}}
    est = WorldPoseEstimator(Pose2D(0.0, 0.0, 0.0), station_world_poses)
    est.on_odom(0.0, 0.0, 0.0)
    est.on_odom(100.0, 100.0, 3.0)  # odometry가 엉망으로 틀어진 상황을 가정

    pose = est.on_docked("LEVEL_1")

    assert pose.x == 5.0 and pose.y == 1.0 and pose.yaw == 1.57
    assert est.pose.x == 5.0  # 내부 상태도 스냅됐는지 확인


def test_on_docked_ignores_unknown_station():
    est = WorldPoseEstimator(Pose2D(1.0, 2.0, 0.5), station_world_poses={})
    result = est.on_docked("NO_SUCH_STATION")
    assert result is None
    assert est.pose.x == 1.0 and est.pose.y == 2.0 and est.pose.yaw == 0.5  # 그대로 유지
