"""
aruco_docker.py (WBS 2.3)

coarse_mover가 핸드오프한 지점(목표 마커까지 대략 ≤0.6m)부터, P-제어로
선속도/각속도를 계산해 ArUco 마커 정면에 정확히 정렬시키는 정밀 도킹 로직.

차동구동(differential-drive) Limo의 한계: 근거리에서는 "옆으로 미끄러지듯"
움직일 수 없고 회전+직진의 조합으로만 위치를 고칠 수 있다. 그래서 횡방향
오차(lateral_m)는 각속도로, 전후방 오차(distance_m)는 선속도로 분리해서
제어한다 — 완벽한 정렬은 아니지만 FR5의 티칭 좌표 허용 오차 안에는 들어오도록
설계했다 (실측 검증은 2.8에서).

N프레임 연속 안정성 체크를 쓰는 이유: ArUco 인식은 카메라 노이즈로 프레임마다
오차가 들쭉날쭉하다. 허용오차 안에 "한 번" 들어온 순간 바로 도착 처리하면,
사실은 흔들리다가 우연히 한 프레임만 맞은 걸 도착으로 오판할 수 있다.

go_to_station_server.py(2.5)가 ROS 토픽/퍼블리셔/시간을 함수로 감싸서 넘겨주면
이 클래스는 순수 계산만 한다 (ROS 의존성 없음 -> 이 환경에서도 유닛 테스트 가능).
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class DockResult(Enum):
    SUCCESS = "SUCCESS"
    MARKER_NOT_FOUND = "MARKER_NOT_FOUND"


@dataclass
class DockConfig:
    kp_linear: float = 0.6
    kp_angular: float = 1.2
    target_distance_m: float = 0.15       # 도킹 완료로 볼 최종 거리 (마커로부터)
    distance_tolerance_m: float = 0.02
    yaw_tolerance_rad: float = 0.03
    max_linear_mps: float = 0.25
    max_angular_rps: float = 0.6
    stable_frames_required: int = 5       # 이만큼 연속으로 허용오차 안에 있어야 "도착"으로 인정
    not_found_timeout_s: float = 3.0      # 이 시간 동안 마커를 한 번도 못 보면 실패


class ArucoDocker:
    def __init__(self, config: Optional[DockConfig] = None):
        self.config = config or DockConfig()
        self._stable_count = 0
        self._not_found_elapsed_s = 0.0

    def reset(self) -> None:
        self._stable_count = 0
        self._not_found_elapsed_s = 0.0

    def step(
        self, detection: Optional[dict], dt_s: float
    ) -> tuple[Optional[DockResult], float, float]:
        """
        detection: {"distance_m": float, "lateral_m": float, "yaw_rad": float} 또는
                   마커가 안 보이면 None
        dt_s: 이번 tick이 실제로 걸린 시간(초) — not_found_timeout_s 누적에 사용

        반환: (result, linear_x, angular_z)
          - result가 None이면 "아직 진행 중이니 이 속도로 계속 움직여라"
          - result가 채워지면 "끝났다" (이때 linear_x/angular_z는 항상 0.0)
        """
        cfg = self.config

        if detection is None:
            self._stable_count = 0
            self._not_found_elapsed_s += dt_s
            if self._not_found_elapsed_s >= cfg.not_found_timeout_s:
                return DockResult.MARKER_NOT_FOUND, 0.0, 0.0
            return None, 0.0, 0.0  # 잠깐 안 보이는 건 봐주고 제자리에서 재탐색

        self._not_found_elapsed_s = 0.0

        distance_error = detection["distance_m"] - cfg.target_distance_m
        lateral_error = detection["lateral_m"]
        yaw_error = detection["yaw_rad"]

        within_tolerance = (
            abs(distance_error) <= cfg.distance_tolerance_m
            and abs(lateral_error) <= cfg.distance_tolerance_m
            and abs(yaw_error) <= cfg.yaw_tolerance_rad
        )

        if within_tolerance:
            self._stable_count += 1
            if self._stable_count >= cfg.stable_frames_required:
                return DockResult.SUCCESS, 0.0, 0.0
            return None, 0.0, 0.0  # 멈춘 채로 안정성 프레임을 더 쌓는다
        else:
            self._stable_count = 0

        linear_x = _clamp(cfg.kp_linear * distance_error, cfg.max_linear_mps)
        # 횡방향 오차(lateral_error)와 yaw_error를 합쳐 각속도로 보정
        angular_z = _clamp(cfg.kp_angular * (lateral_error + yaw_error), cfg.max_angular_rps)
        return None, linear_x, angular_z


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
