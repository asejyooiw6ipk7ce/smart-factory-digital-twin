"""
job_runner.py (WBS 1.4)

Job(1.3에서 정의한 설계도)을 실제로 한 Step씩 실행시키는 엔진.

ROS2에 의존하지 않도록 일부러 설계했다: 실제로 로봇을 움직이는 부분은
"executor"라는 콜백 함수로 밖에서 주입받는다. 그래서 오케스트레이터는

    JobRunner(executor=self._execute_step, db=self.db, on_change=self._on_job_change)

처럼 자기 자신의 액션 클라이언트 호출 메서드를 넘겨주기만 하면 되고,
WBS 1.5(test_job_runner.py)에서는 진짜 ROS 액션 대신 "성공/PICK 실패/PLACE 실패"를
흉내 내는 가짜 executor를 넣어서 로직만 따로 검증할 수 있다.

executor의 약속(계약):
    executor(step: Step, result_cb) 형태로 호출된다.
    executor는 당장 끝나지 않아도 된다 (ROS 액션은 비동기이므로).
    끝나면 반드시 한 번, result_cb(success: bool, failed_phase: str, message: str)를 호출해야 한다.

실행 모델: JobRunner는 항상 Job을 한 번에 하나만 돌린다 (공장에 로봇이 Limo 1대,
FR5 1대뿐이라 어차피 동시에 여러 개를 처리할 수 없다). 그래서 JobRunner는
"지금 바쁜가(busy)"만 밖에 알려주면 되고, 오케스트레이터는 매 tick마다
busy가 아닐 때만 큐에서 다음 Job을 꺼내 start()를 호출한다.
"""

from typing import Callable, Optional

from factory_core.job import Job, JobStatus, JobType, RetryPolicy, Step

# result_cb(success, failed_phase, message)
ResultCallback = Callable[[bool, str, str], None]
# executor(step, result_cb)
Executor = Callable[[Step, ResultCallback], None]
# on_change(job)
OnChange = Callable[[Job], None]


class JobRunner:
    def __init__(self, executor: Executor, db, on_change: Optional[OnChange] = None):
        self.executor = executor
        self.db = db
        self._on_change = on_change or (lambda job: None)
        self.current_job: Optional[Job] = None
        # cancel()/retry()로 지금 실행 중이던 액션이 끊겼을 때,
        # 그 액션의 결과가 뒤늦게 도착해도 무시하기 위한 세대(generation) 번호.
        # orchestrator_node.py가 ROS 액션 타임아웃에 쓰는 goal_token과 같은 패턴이다.
        self._run_id = 0

    @property
    def busy(self) -> bool:
        """Job을 하나 처리 중이면 True (ERROR 상태로 멈춰 있는 것도 '바쁜' 것으로 친다:
        사람이 retry/cancel 하기 전까지는 다음 Job을 시작하면 안 되기 때문)."""
        return self.current_job is not None

    # ------------------------------------------------------------------
    # 외부(오케스트레이터)에서 호출하는 API
    # ------------------------------------------------------------------
    def start(self, job: Job) -> None:
        if self.busy:
            raise RuntimeError("JobRunner는 이미 다른 Job을 처리 중입니다")
        job.status = JobStatus.RUNNING
        self.current_job = job
        self._sync_status_to_db(job)
        self._on_change(job)
        self._run_current_step()

    def retry(self) -> bool:
        """ERROR 상태에서 멈춘 Job을, 실패했던 바로 그 Step부터 다시 실행한다.

        사람이 현장을 보고 "FR5가 실제로 물건을 못 집었다 / 떨어뜨리지 않았다"를
        확인한 뒤에 호출한다고 가정한다 (재시작 복구 설계와 동일한 전제).
        """
        job = self.current_job
        if job is None or job.status != JobStatus.ERROR:
            return False
        job.status = JobStatus.RUNNING
        self._sync_status_to_db(job)
        self._on_change(job)
        self._run_current_step()
        return True

    def cancel(self) -> bool:
        """지금 처리 중인(또는 ERROR로 멈춘) Job을 취소하고 예약을 풀어준다."""
        job = self.current_job
        if job is None:
            return False

        self._run_id += 1  # 진행 중이던 액션의 결과가 뒤늦게 와도 무시하게 됨

        if job.job_type == JobType.OUTBOUND:
            self.db.release(job.item_id)
        else:
            self.db.release_inbound(job.item_id)

        job.status = JobStatus.CANCELLED
        self._sync_status_to_db(job)
        self._on_change(job)
        self.current_job = None
        return True

    # ------------------------------------------------------------------
    # 내부 로직
    # ------------------------------------------------------------------
    def _run_current_step(self) -> None:
        job = self.current_job
        assert job is not None
        step = job.steps[job.current]

        self._run_id += 1
        run_id = self._run_id

        def result_cb(success: bool, failed_phase: str = "", message: str = "") -> None:
            self._on_step_result(run_id, success, failed_phase, message)

        self.executor(step, result_cb)

    def _on_step_result(
        self, run_id: int, success: bool, failed_phase: str, message: str
    ) -> None:
        if run_id != self._run_id:
            return  # cancel()/retry()로 이미 무효화된, 뒤늦게 온 결과

        job = self.current_job
        if job is None:
            return  # 이론상 run_id 체크로 걸러지지만 방어적으로 한 번 더 확인

        step = job.steps[job.current]

        if success:
            job.current += 1
            if job.current >= len(job.steps):
                job.status = JobStatus.DONE
                self._sync_status_to_db(job)
                self._on_change(job)
                self.current_job = None
            else:
                self._on_change(job)
                self._run_current_step()
            return

        step.attempts += 1
        can_retry = step.retry_policy == RetryPolicy.ALWAYS or (
            step.retry_policy == RetryPolicy.IF_PICK_FAILED and failed_phase == "PICK"
        )
        if can_retry and step.attempts <= step.max_retries:
            self._on_change(job)
            self._run_current_step()
        else:
            job.status = JobStatus.ERROR
            self._sync_status_to_db(job)
            self._on_change(job)
            # current_job은 그대로 둔다: busy가 True로 유지되어 다음 Job이 끼어들지
            # 못하게 막고, 사람이 /factory/retry_or_cancel로 처리할 때까지 기다린다.

    def _sync_status_to_db(self, job: Job) -> None:
        if job.job_id is not None:
            self.db.set_job_status(job.job_id, job.status.value)
