"""
job.py (WBS 1.3)

Job / Step 데이터 모델.

오케스트레이터가 다루는 "작업 하나(Job)"와 그 안의 "단계 하나(Step)"를
표현하는 순수 파이썬 모듈이다. ROS2에 전혀 의존하지 않기 때문에
rclpy 없이, 로봇 없이도 단위 테스트가 가능하다 (WBS 1.5에서 검증).

실제 동작(로봇을 움직이거나 DB를 쓰는 일)은 여기서 하지 않는다.
이 파일은 "무엇을, 어떤 순서로, 실패하면 어떻게 할지"에 대한 설계도만 들고 있고,
실행은 job_runner.py가 한다.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional


class JobType(Enum):
    """Job이 입고용인지 출고용인지."""

    INBOUND = "INBOUND"
    OUTBOUND = "OUTBOUND"


class JobStatus(Enum):
    """Job 전체의 현재 상태."""

    PENDING = "PENDING"       # 큐에 있지만 아직 시작 안 함
    RUNNING = "RUNNING"       # 실행 중
    DONE = "DONE"             # 모든 Step 성공
    ERROR = "ERROR"           # 사람 개입이 필요한 실패 (PLACE 실패, 타임아웃 등)
    CANCELLED = "CANCELLED"   # 사용자가 취소함


class StepKind(Enum):
    """Step 하나가 실제로 어떤 일을 하는지."""

    GO_TO = "GO_TO"                   # Limo를 특정 station(ArUco 위치)으로 이동
    FR5_TASK = "FR5_TASK"             # FR5가 집고/놓는 작업 1회
    DB_COMMIT_IN = "DB_COMMIT_IN"     # 입고 완료 -> 재고 +1 확정
    DB_COMMIT_OUT = "DB_COMMIT_OUT"   # 출고 완료 -> 재고 -1 확정
    DB_RELEASE = "DB_RELEASE"         # 취소/실패 시 예약(reserved/incoming) 해제
    MARK_SHIPPED = "MARK_SHIPPED"     # 주문 상태를 SHIPPED로 변경


class RetryPolicy(Enum):
    """이 Step이 실패했을 때 자동으로 재시도해도 되는지.

    멘토 피드백 반영: FR5 작업은 "집는 데 실패(PICK)"했을 때만 안전하게
    재시도할 수 있다. "놓는 데 실패(PLACE)"했다면 물건이 이미 로봇 손이나
    허공 어딘가에 있을 수 있어서, 블라인드 재시도는 물리적 상태와 DB를
    둘 다 망가뜨릴 수 있다. 그래서 기본값은 NEVER이고, FR5_TASK Step에만
    명시적으로 IF_PICK_FAILED를 지정한다.
    """

    NEVER = "NEVER"                     # 실패하면 무조건 Job을 ERROR로 (기본값)
    IF_PICK_FAILED = "IF_PICK_FAILED"   # FR5Task 전용: failed_phase == PICK일 때만 재시도
    ALWAYS = "ALWAYS"                   # 무조건 재시도 (현재는 안 쓰지만 확장을 위해 남겨둠)


@dataclass
class Step:
    """Job을 구성하는 작업 단위 하나."""

    kind: StepKind
    params: dict[str, Any] = field(default_factory=dict)
    retry_policy: RetryPolicy = RetryPolicy.NEVER
    max_retries: int = 1
    attempts: int = 0  # JobRunner가 재시도할 때마다 올리는 카운터 (Step 자체는 건드리지 않음)


@dataclass
class Job:
    """오케스트레이터 큐에 들어가는 작업 하나 (입고 1건 또는 출고 1건)."""

    job_type: JobType
    steps: list[Step]
    item_id: int
    order_id: Optional[int] = None   # 출고 Job에서만 사용 (주문 ID)
    job_id: Optional[int] = None     # DB에 저장된 뒤 채워지는 PK
    current: int = 0                 # 지금 실행 중인 steps의 인덱스
    status: JobStatus = JobStatus.PENDING
