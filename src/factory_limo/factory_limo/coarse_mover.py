"""
coarse_mover.py (WBS 2.4)

Station마다 미리 정해둔 "대략 이동 경로"(선속도/각속도/지속시간 구간의 나열)를
그대로 재생하면서, 매 tick마다 최신 ArUco 탐지 결과를 확인해 목표 마커가
핸드오프 거리 안에 들어오면 즉시 멈추고 정밀 도킹(aruco_docker.py)으로 넘겨준다.

왜 odometry/시간 기반 "경로 재생"이냐: 이 프로젝트는 SLAM/Nav2 없이
"선반 앞까지 대충 가서, ArUco가 보이면 그때부터 비전 기반으로 정밀 정렬"하는
2단계 전략이다. coarse 구간은 정밀도가 중요하지 않으므로, station별로 한 번
실측해서 기록해 둔 (선속도, 각속도, 지속시간) 시퀀스를 그냥 타이머로 재생하는
게 3일짜리 일정에서 가장 빠르고 단순한 방법이다.

go_to_station_server.py(2.5)가 ROS 토픽/퍼블리셔/시간을 함수로 감싸서 넘겨주면
(ROS 없이도 테스트 가능하도록), 이 클래스는 그 함수들만 호출한다.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional


class CoarseResult(Enum):
    HANDED_OFF = "HANDED_OFF"  # ArUco가 보여서 정밀 도킹으로 넘길 준비가 됨
    TIMEOUT = "TIMEOUT"        # 예상 이동시간을 다 썼는데도 마커를 핸드오프 거리 안에서 못 봄
    CANCELLED = "CANCELLED"


@dataclass
class CoarseMoveSegment:
    linear_x: float    # m/s
    angular_z: float   # rad/s
    duration_s: float


class CoarseMover:
    def __init__(self, routes: dict[str, list[CoarseMoveSegment]], handoff_distance_m: float = 0.6):
        self.routes = routes
        self.handoff_distance_m = handoff_distance_m

    def run(
        self,
        station: str,
        cmd_vel: Callable[[float, float], None],
        get_latest_detection: Callable[[], Optional[dict]],
        sleep_fn: Callable[[float], None],
        is_cancelled: Callable[[], bool],
        rate_hz: float = 10.0,
    ) -> CoarseResult:
        """
        cmd_vel(linear_x, angular_z): 속도 명령을 실제로 publish하는 함수
        get_latest_detection(): 지금 target 마커가 보이면 {"distance_m": ...}
                                 (그 외 필드 포함 가능), 안 보이면 None을 돌려주는 함수
                                 — 어떤 마커 id를 볼지 필터링하는 건 호출자(go_to_station_server)
                                 책임이다. 여기서는 "target 마커를 봤는지 여부"만 다룬다.
        sleep_fn(seconds): time.sleep 대신 주입받는 함수 (테스트에서는 즉시 리턴하는 가짜로 대체)
        is_cancelled(): 액션 취소 요청이 왔는지 확인하는 함수
        """
        segments = self.routes.get(station)
        if not segments:
            raise ValueError(f"station={station!r}에 대한 coarse 경로가 config에 없습니다")

        dt = 1.0 / rate_hz
        for seg in segments:
            elapsed = 0.0
            while elapsed < seg.duration_s:
                if is_cancelled():
                    cmd_vel(0.0, 0.0)
                    return CoarseResult.CANCELLED

                detection = get_latest_detection()
                if detection is not None and detection["distance_m"] <= self.handoff_distance_m:
                    cmd_vel(0.0, 0.0)
                    return CoarseResult.HANDED_OFF

                cmd_vel(seg.linear_x, seg.angular_z)
                sleep_fn(dt)
                elapsed += dt

        cmd_vel(0.0, 0.0)

        # 경로를 다 재생한 직후 마지막으로 한 번 더 확인 (멈추는 순간 바로 보일 수도 있음)
        detection = get_latest_detection()
        if detection is not None and detection["distance_m"] <= self.handoff_distance_m:
            return CoarseResult.HANDED_OFF
        return CoarseResult.TIMEOUT
