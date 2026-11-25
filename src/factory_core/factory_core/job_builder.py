"""
job_builder.py (WBS 1.3)

입고/출고 "레시피"를 실제 Job(Step의 나열)으로 조립하는 조립 공장.

프로젝트 설명서에 적힌 흐름을 그대로 Step 순서로 옮긴 것뿐이다:

  입고: Limo가 선반 위치로 이동
        -> FR5가 Limo 위 물건을 집어 선반에 놓음 (실패 시 PICK이면 재시도)
        -> DB 재고 +1 확정

  출고: Limo가 선반 위치로 이동
        -> FR5가 선반에서 집어 Limo 위에 놓음 (실패 시 PICK이면 재시도)
        -> DB 재고 -1 확정
        -> Limo가 원점(HOME)으로 복귀
        -> 주문 상태를 SHIPPED로 변경

이 "레시피"를 한곳에 모아두는 이유: 나중에 흐름이 바뀌어도(예: 출고 전에
바코드 스캔 단계를 추가하고 싶다) 여기 메서드 하나만 고치면 되고,
orchestrator_node.py나 job_runner.py는 전혀 건드릴 필요가 없다.
"""

from factory_core.job import Job, JobType, RetryPolicy, Step, StepKind


class JobBuilder:
    @staticmethod
    def build_inbound(item_id: int, level: int) -> Job:
        steps = [
            Step(kind=StepKind.GO_TO, params={"station": f"LEVEL_{level}"}),
            Step(
                kind=StepKind.FR5_TASK,
                params={"task_type": "INBOUND", "level": level},
                retry_policy=RetryPolicy.IF_PICK_FAILED,
                max_retries=2,
            ),
            Step(kind=StepKind.DB_COMMIT_IN, params={"item_id": item_id}),
        ]
        return Job(job_type=JobType.INBOUND, steps=steps, item_id=item_id)

    @staticmethod
    def build_outbound(order_id: int, item_id: int, level: int) -> Job:
        steps = [
            Step(kind=StepKind.GO_TO, params={"station": f"LEVEL_{level}"}),
            Step(
                kind=StepKind.FR5_TASK,
                params={"task_type": "OUTBOUND", "level": level},
                retry_policy=RetryPolicy.IF_PICK_FAILED,
                max_retries=2,
            ),
            Step(kind=StepKind.DB_COMMIT_OUT, params={"item_id": item_id}),
            Step(kind=StepKind.GO_TO, params={"station": "HOME"}),
            Step(kind=StepKind.MARK_SHIPPED, params={"order_id": order_id}),
        ]
        return Job(
            job_type=JobType.OUTBOUND,
            steps=steps,
            item_id=item_id,
            order_id=order_id,
        )
