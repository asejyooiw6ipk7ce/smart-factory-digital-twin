"""
test_coarse_mover.py (WBS 2.4에 딸린 유닛 테스트)

coarse_mover.py는 ROS 의존성이 없는 순수 로직이라, 실제 로봇/카메라 없이
가짜 cmd_vel/detection/sleep/cancel 함수만으로 검증할 수 있다.

실행:
    cd factory_ws/src/factory_limo
    PYTHONPATH=. python3 -m pytest test/test_coarse_mover.py -v
"""

from factory_limo.coarse_mover import CoarseMover, CoarseMoveSegment, CoarseResult


def make_mover(duration_s=1.0, handoff_distance_m=0.6):
    routes = {"LEVEL_1": [CoarseMoveSegment(linear_x=0.2, angular_z=0.0, duration_s=duration_s)]}
    return CoarseMover(routes, handoff_distance_m=handoff_distance_m)


def test_hands_off_when_marker_seen_within_distance():
    mover = make_mover(duration_s=10.0)  # 충분히 긴 경로를 줘서, 핸드오프가 "조기 종료"임을 확인
    cmd_calls = []
    ticks = {"n": 0}

    def get_latest_detection():
        ticks["n"] += 1
        if ticks["n"] >= 3:
            return {"distance_m": 0.5}  # handoff_distance_m(0.6)보다 가까움
        return None

    result = mover.run(
        station="LEVEL_1",
        cmd_vel=lambda lin, ang: cmd_calls.append((lin, ang)),
        get_latest_detection=get_latest_detection,
        sleep_fn=lambda s: None,
        is_cancelled=lambda: False,
        rate_hz=10.0,
    )

    assert result == CoarseResult.HANDED_OFF
    assert cmd_calls[-1] == (0.0, 0.0)  # 멈추는 명령으로 끝나야 함
    assert ticks["n"] < 100  # 10초 전체를 다 돌지 않고 조기 종료됐어야 함


def test_timeout_when_marker_never_close_enough():
    mover = make_mover(duration_s=0.3)  # 0.3초 / 0.1초(10Hz) = 3틱짜리 짧은 경로

    result = mover.run(
        station="LEVEL_1",
        cmd_vel=lambda lin, ang: None,
        get_latest_detection=lambda: None,  # 끝까지 마커 안 보임
        sleep_fn=lambda s: None,
        is_cancelled=lambda: False,
        rate_hz=10.0,
    )

    assert result == CoarseResult.TIMEOUT


def test_cancel_stops_immediately():
    mover = make_mover(duration_s=10.0)
    cmd_calls = []

    result = mover.run(
        station="LEVEL_1",
        cmd_vel=lambda lin, ang: cmd_calls.append((lin, ang)),
        get_latest_detection=lambda: None,
        sleep_fn=lambda s: None,
        is_cancelled=lambda: True,  # 첫 틱부터 취소 요청
        rate_hz=10.0,
    )

    assert result == CoarseResult.CANCELLED
    assert cmd_calls == [(0.0, 0.0)]


def test_unknown_station_raises():
    mover = make_mover()
    try:
        mover.run(
            station="NO_SUCH_STATION",
            cmd_vel=lambda lin, ang: None,
            get_latest_detection=lambda: None,
            sleep_fn=lambda s: None,
            is_cancelled=lambda: False,
        )
        assert False, "ValueError가 발생했어야 함"
    except ValueError:
        pass
