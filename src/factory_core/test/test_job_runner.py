"""
test_job_runner.py (WBS 1.5, 위험도: 핵심)

job_runner.py가 "멘토 피드백에서 지적된 버그를 실제로 막고 있는지"를
실제 로봇/ROS2 없이 검증하는 단위 테스트.

실행 방법:
    cd factory_ws/src/factory_core
    PYTHONPATH=. python3 -m pytest test/test_job_runner.py -v

핵심 아이디어: JobRunner.executor에 "가짜 로봇" 역할을 하는 평범한 파이썬
함수를 넣는다. 이 가짜 executor가 FakeDB와 함께 "PICK에서 실패/PLACE에서
실패/몇 번 실패 후 성공" 같은 상황을 그대로 재현해준다.
"""

from factory_core.job import JobStatus, JobType, RetryPolicy, StepKind
from factory_core.job_builder import JobBuilder
from factory_core.job_runner import JobRunner


class FakeDB:
    """InventoryDB(1.2/1.6) 대신 쓰는 가짜 DB. 호출 기록만 남긴다."""

    def __init__(self):
        self.status_log = []      # (job_id, status_value)
        self.released = []        # item_id (release, 출고)
        self.released_inbound = []  # item_id (release_inbound, 입고)

    def set_job_status(self, job_id, status):
        self.status_log.append((job_id, status))

    def release(self, item_id):
        self.released.append(item_id)

    def release_inbound(self, item_id):
        self.released_inbound.append(item_id)


def make_outbound_job(job_id=1, order_id=100, item_id=7, level=2):
    job = JobBuilder.build_outbound(order_id=order_id, item_id=item_id, level=level)
    job.job_id = job_id
    return job


def make_inbound_job(job_id=1, item_id=7, level=2):
    job = JobBuilder.build_inbound(item_id=item_id, level=level)
    job.job_id = job_id
    return job


def always_succeed_executor(step, result_cb):
    result_cb(True, "", "")


# ----------------------------------------------------------------------
# 1. 모든 Step 성공 -> DONE
# ----------------------------------------------------------------------
def test_all_steps_succeed_job_is_done():
    db = FakeDB()
    runner = JobRunner(executor=always_succeed_executor, db=db, on_change=None)
    job = make_outbound_job()

    runner.start(job)

    assert job.status == JobStatus.DONE
    assert job.current == len(job.steps)
    assert not runner.busy
    assert db.status_log[-1] == (job.job_id, "DONE")


# ----------------------------------------------------------------------
# 2. PICK 실패가 max_retries 이내면 재시도 후 성공
# ----------------------------------------------------------------------
def test_pick_failure_within_max_retries_then_succeeds():
    db = FakeDB()
    fail_count = {"n": 0}

    def executor(step, result_cb):
        if step.kind == StepKind.FR5_TASK and fail_count["n"] < step.max_retries:
            fail_count["n"] += 1
            result_cb(False, "PICK", "gripper empty")
        else:
            result_cb(True, "", "")

    runner = JobRunner(executor=executor, db=db, on_change=None)
    job = make_outbound_job()

    runner.start(job)

    assert job.status == JobStatus.DONE
    fr5_step = job.steps[1]
    assert fr5_step.kind == StepKind.FR5_TASK
    assert fr5_step.attempts == fr5_step.max_retries


# ----------------------------------------------------------------------
# 3. PICK 실패가 max_retries를 넘으면 -> ERROR로 멈춤
# ----------------------------------------------------------------------
def test_pick_failure_beyond_max_retries_goes_to_error():
    db = FakeDB()

    def always_fail_pick(step, result_cb):
        if step.kind == StepKind.FR5_TASK:
            result_cb(False, "PICK", "gripper empty")
        else:
            result_cb(True, "", "")

    runner = JobRunner(executor=always_fail_pick, db=db, on_change=None)
    job = make_outbound_job()

    runner.start(job)

    assert job.status == JobStatus.ERROR
    assert runner.busy  # ERROR여도 busy는 유지되어야 함 (다음 Job 시작 금지)
    fr5_step = job.steps[1]
    assert fr5_step.attempts == fr5_step.max_retries + 1


# ----------------------------------------------------------------------
# 4. PLACE(또는 다른 phase) 실패는 재시도 없이 즉시 ERROR
#    (멘토 피드백이 지적한 바로 그 버그: 블라인드 재시도 금지)
# ----------------------------------------------------------------------
def test_place_failure_never_retries_goes_straight_to_error():
    db = FakeDB()
    attempts = {"n": 0}

    def fail_place_once(step, result_cb):
        if step.kind == StepKind.FR5_TASK:
            attempts["n"] += 1
            result_cb(False, "PLACE", "drop detected")
        else:
            result_cb(True, "", "")

    runner = JobRunner(executor=fail_place_once, db=db, on_change=None)
    job = make_outbound_job()

    runner.start(job)

    assert job.status == JobStatus.ERROR
    assert attempts["n"] == 1  # 단 한 번만 시도되고 바로 ERROR (재시도 없음)
    fr5_step = job.steps[1]
    assert fr5_step.attempts == 1
    assert fr5_step.retry_policy == RetryPolicy.IF_PICK_FAILED  # PLACE는 대상이 아님


# ----------------------------------------------------------------------
# 5. ERROR 상태에서 cancel() -> 출고 Job이면 release(), busy 해제, CANCELLED
# ----------------------------------------------------------------------
def test_cancel_outbound_job_releases_reservation():
    db = FakeDB()

    def fail_place(step, result_cb):
        if step.kind == StepKind.FR5_TASK:
            result_cb(False, "PLACE", "drop detected")
        else:
            result_cb(True, "", "")

    runner = JobRunner(executor=fail_place, db=db, on_change=None)
    job = make_outbound_job(item_id=42)
    runner.start(job)
    assert job.status == JobStatus.ERROR

    ok = runner.cancel()

    assert ok is True
    assert job.status == JobStatus.CANCELLED
    assert not runner.busy
    assert db.released == [42]
    assert db.released_inbound == []


# ----------------------------------------------------------------------
# 5b. 입고 Job을 cancel()하면 release_inbound()가 불려야 함
# ----------------------------------------------------------------------
def test_cancel_inbound_job_releases_inbound_reservation():
    db = FakeDB()

    def fail_place(step, result_cb):
        if step.kind == StepKind.FR5_TASK:
            result_cb(False, "PLACE", "drop detected")
        else:
            result_cb(True, "", "")

    runner = JobRunner(executor=fail_place, db=db, on_change=None)
    job = make_inbound_job(item_id=99)
    runner.start(job)
    assert job.status == JobStatus.ERROR

    ok = runner.cancel()

    assert ok is True
    assert db.released_inbound == [99]
    assert db.released == []


# ----------------------------------------------------------------------
# 6. ERROR 상태에서 retry() -> 실패했던 그 Step부터 재개
# ----------------------------------------------------------------------
def test_retry_resumes_from_failed_step():
    db = FakeDB()
    mode = {"fail": True}

    def executor(step, result_cb):
        if step.kind == StepKind.FR5_TASK and mode["fail"]:
            result_cb(False, "PLACE", "drop detected")
        else:
            result_cb(True, "", "")

    runner = JobRunner(executor=executor, db=db, on_change=None)
    job = make_outbound_job()
    runner.start(job)
    assert job.status == JobStatus.ERROR
    assert job.current == 1  # FR5_TASK Step(인덱스 1)에서 멈춤

    mode["fail"] = False  # 사람이 현장 확인 후 문제 없다고 판단한 상황을 가정
    ok = runner.retry()

    assert ok is True
    assert job.status == JobStatus.DONE
    assert job.current == len(job.steps)


def test_retry_returns_false_when_not_in_error():
    db = FakeDB()
    runner = JobRunner(executor=always_succeed_executor, db=db, on_change=None)
    job = make_outbound_job()
    runner.start(job)  # 바로 DONE으로 끝남

    assert runner.retry() is False


# ----------------------------------------------------------------------
# 7. busy는 RUNNING/ERROR일 때 True, DONE/CANCELLED 이후엔 False
# ----------------------------------------------------------------------
def test_busy_reflects_job_lifecycle():
    db = FakeDB()
    runner = JobRunner(executor=always_succeed_executor, db=db, on_change=None)

    assert not runner.busy
    job = make_outbound_job()
    runner.start(job)
    assert not runner.busy  # 동기 executor라 start() 안에서 바로 DONE까지 끝남

    db2 = FakeDB()

    def fail_place(step, result_cb):
        if step.kind == StepKind.FR5_TASK:
            result_cb(False, "PLACE", "drop detected")
        else:
            result_cb(True, "", "")

    runner2 = JobRunner(executor=fail_place, db=db2, on_change=None)
    job2 = make_outbound_job(job_id=2)
    runner2.start(job2)
    assert runner2.busy  # ERROR 상태에서도 busy

    runner2.cancel()
    assert not runner2.busy


# ----------------------------------------------------------------------
# 8. cancel() 이후에 뒤늦게 도착하는 콜백은 무시된다 (_run_id 세대 체크)
# ----------------------------------------------------------------------
def test_stale_callback_after_cancel_is_ignored():
    db = FakeDB()
    pending_cb = {}

    def capturing_executor(step, result_cb):
        # 콜백을 바로 부르지 않고 보관해둔다 (비동기 액션이 아직 안 끝난 상황 재현)
        pending_cb["cb"] = result_cb

    runner = JobRunner(executor=capturing_executor, db=db, on_change=None)
    job = make_outbound_job()
    runner.start(job)
    assert runner.busy
    assert job.current == 0  # 첫 Step(GO_TO)이 아직 진행 중

    runner.cancel()
    assert job.status == JobStatus.CANCELLED
    assert not runner.busy

    # 취소된 뒤에야 "성공했다"는 결과가 뒤늦게 도착
    pending_cb["cb"](True, "", "")

    # 뒤늦은 콜백이 Job을 다시 건드리면 안 된다
    assert job.status == JobStatus.CANCELLED
    assert job.current == 0
    assert not runner.busy


def test_stale_callback_after_retry_restarted_is_ignored():
    db = FakeDB()
    fr5_calls = []  # FR5_TASK Step에 대한 result_cb만 모은다 (GO_TO는 매번 바로 성공 처리)

    def executor(step, result_cb):
        if step.kind != StepKind.FR5_TASK:
            result_cb(True, "", "")
            return
        fr5_calls.append(result_cb)
        if len(fr5_calls) == 1:
            # FR5_TASK 첫 시도는 PLACE 실패로 바로 응답 -> ERROR
            result_cb(False, "PLACE", "drop detected")
        # 두 번째(retry 이후) 호출은 응답하지 않고 보류해서 "아직 실행 중" 상태로 둔다

    runner = JobRunner(executor=executor, db=db, on_change=None)
    job = make_outbound_job()
    runner.start(job)
    assert job.status == JobStatus.ERROR
    assert job.current == 1  # FR5_TASK(인덱스 1)에서 멈춤

    runner.retry()
    assert job.status == JobStatus.RUNNING
    assert len(fr5_calls) == 2

    # retry로 새로 시작된 실행이 아직 안 끝났는데, 그 이전(첫 시도) 콜백이 뒤늦게
    # 다시 불린다 해도 (이론상 있어서는 안 되지만 방어적으로) 무시되어야 한다
    fr5_calls[0](True, "", "")

    assert job.status == JobStatus.RUNNING  # 여전히 재시도 중이어야 함
    assert job.current == 1
