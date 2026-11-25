"""
test_aruco_docker.py (WBS 2.3에 딸린 유닛 테스트)

실행:
    cd factory_ws/src/factory_limo
    PYTHONPATH=. python3 -m pytest test/test_aruco_docker.py -v
"""

from factory_limo.aruco_docker import ArucoDocker, DockConfig, DockResult


def make_docker(**overrides):
    config = DockConfig(
        kp_linear=0.6,
        kp_angular=1.2,
        target_distance_m=0.15,
        distance_tolerance_m=0.02,
        yaw_tolerance_rad=0.03,
        max_linear_mps=0.25,
        max_angular_rps=0.6,
        stable_frames_required=3,
        not_found_timeout_s=1.0,
    )
    for k, v in overrides.items():
        setattr(config, k, v)
    return ArucoDocker(config)


def test_in_progress_returns_none_with_nonzero_velocity_when_far():
    docker = make_docker()
    result, linear_x, angular_z = docker.step(
        {"distance_m": 0.5, "lateral_m": 0.1, "yaw_rad": 0.0}, dt_s=0.1
    )
    assert result is None
    assert linear_x > 0  # 목표보다 멀리 있으니 전진해야 함
    assert angular_z > 0  # lateral_m이 양수니 그 방향으로 보정해야 함


def test_success_after_stable_frames_within_tolerance():
    docker = make_docker()
    # target_distance_m=0.15, tolerance=0.02 -> 0.15가 완벽히 허용오차 안
    detection = {"distance_m": 0.15, "lateral_m": 0.0, "yaw_rad": 0.0}

    r1, _, _ = docker.step(detection, dt_s=0.1)
    r2, _, _ = docker.step(detection, dt_s=0.1)
    assert r1 is None and r2 is None  # 아직 stable_frames_required(3)에 못 미침

    r3, lin, ang = docker.step(detection, dt_s=0.1)
    assert r3 == DockResult.SUCCESS
    assert lin == 0.0 and ang == 0.0


def test_instability_resets_stable_count():
    docker = make_docker()
    good = {"distance_m": 0.15, "lateral_m": 0.0, "yaw_rad": 0.0}
    bad = {"distance_m": 0.30, "lateral_m": 0.0, "yaw_rad": 0.0}  # 허용오차 밖

    docker.step(good, dt_s=0.1)
    docker.step(good, dt_s=0.1)
    result, _, _ = docker.step(bad, dt_s=0.1)  # 안정 카운트가 리셋돼야 함
    assert result is None

    # 다시 good을 3번 연속으로 줘야 성공해야 한다 (이전 카운트가 이어지면 안 됨)
    docker.step(good, dt_s=0.1)
    docker.step(good, dt_s=0.1)
    result, _, _ = docker.step(good, dt_s=0.1)
    assert result == DockResult.SUCCESS


def test_marker_not_found_after_timeout():
    docker = make_docker(not_found_timeout_s=0.25)
    r1, _, _ = docker.step(None, dt_s=0.1)
    r2, _, _ = docker.step(None, dt_s=0.1)
    assert r1 is None and r2 is None  # 아직 0.25초(timeout) 전

    r3, lin, ang = docker.step(None, dt_s=0.1)  # 누적 0.3초 >= 0.25초
    assert r3 == DockResult.MARKER_NOT_FOUND
    assert lin == 0.0 and ang == 0.0


def test_brief_marker_loss_is_tolerated_and_resets_on_reacquire():
    docker = make_docker(not_found_timeout_s=1.0)
    docker.step(None, dt_s=0.1)  # 잠깐 놓침, 아직 타임아웃 아님

    detection = {"distance_m": 0.5, "lateral_m": 0.0, "yaw_rad": 0.0}
    result, linear_x, _ = docker.step(detection, dt_s=0.1)  # 다시 찾음
    assert result is None
    assert linear_x > 0  # 정상적으로 제어가 계속됨


def test_velocity_is_clamped_to_max():
    docker = make_docker(max_linear_mps=0.2, max_angular_rps=0.5)
    # 아주 멀리 있어서 P 제어값이 limit을 넘길 만한 상황
    result, linear_x, angular_z = docker.step(
        {"distance_m": 5.0, "lateral_m": 5.0, "yaw_rad": 5.0}, dt_s=0.1
    )
    assert result is None
    assert linear_x == 0.2
    assert angular_z == 0.5


def test_reset_clears_internal_state():
    docker = make_docker()
    docker.step(None, dt_s=0.5)  # not_found 카운터를 좀 쌓아둠
    docker.reset()
    # reset 후에는 다시 timeout까지 전체 시간이 필요해야 함
    result, _, _ = docker.step(None, dt_s=0.5)
    assert result is None
