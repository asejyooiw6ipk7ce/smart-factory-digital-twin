"""
orchestrator_node.py (WBS 1.8)

의존 모듈 (아직 미작성이면 아래 계약대로 먼저/나중에 작성):
  - factory_interfaces (WBS 1.1): action/GoToStation, action/Fr5Task,
    srv/PlaceOrder, srv/GetInventory, srv/RequestInbound, srv/RetryOrCancel,
    msg/FactoryState, msg/SlotStock, msg/Inventory
  - factory_core.job            (WBS 1.3): Job, Step, StepKind, JobType, JobStatus, RetryPolicy
  - factory_core.job_builder    (WBS 1.3): JobBuilder.build_inbound / build_outbound
  - factory_core.job_runner     (WBS 1.4): JobRunner
  - factory_core.inventory_db   (WBS 1.2/1.6): InventoryDB

실행 모델 (멘토 피드백 반영):
  - 싱글스레드 실행기 가정. 콜백 안에서 spin_until_future_complete 사용 안 함.
  - 모든 액션 결과는 add_done_callback 체인으로 받는다.
  - 타임아웃 시 cancel_goal_async 호출 + 토큰(goal token)으로 늦게 오는 결과를 무시한다.
  - FR5 에러 리셋(Fairino ResetAllError 류)은 여기가 아니라 fr5_node.py의
    execute_cb 맨 앞에서 매번(멱등) 호출한다. 오케스트레이터는 재시도인지 여부를
    몰라도 되게 설계했다.
"""

from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from factory_core.inventory_db import InventoryDB
from factory_core.job import Job, JobStatus, JobType, Step, StepKind
from factory_core.job_builder import JobBuilder
from factory_core.job_runner import JobRunner

from factory_interfaces.action import Fr5Task, GoToStation
from factory_interfaces.msg import FactoryState, Inventory, SlotStock
from factory_interfaces.srv import (
    GetInventory,
    PlaceOrder,
    RequestInbound,
    RetryOrCancel,
)


class OrchestratorNode(Node):
    def __init__(self):
        super().__init__("orchestrator_node")

        self.declare_parameter("db_path", "factory.db")
        self.declare_parameter("schema_path", "db/schema.sql")
        self.declare_parameter("config_path", "config/factory.yaml")
        self.declare_parameter("go_to_timeout_s", 60.0)
        self.declare_parameter("fr5_task_timeout_s", 45.0)

        db_path = self.get_parameter("db_path").value
        schema_path = self.get_parameter("schema_path").value
        config_path = self.get_parameter("config_path").value
        self._timeouts = {
            StepKind.GO_TO: self.get_parameter("go_to_timeout_s").value,
            StepKind.FR5_TASK: self.get_parameter("fr5_task_timeout_s").value,
        }

        self.db = InventoryDB(db_path, schema_path, config_path)
        self.queue: deque[Job] = deque()
        self.runner = JobRunner(
            executor=self._execute_step,
            db=self.db,
            on_change=self._on_job_change,
        )
        self._last_message = ""

        # 액션 클라이언트
        self.limo_client = ActionClient(self, GoToStation, "/limo/go_to_station")
        self.fr5_client = ActionClient(self, Fr5Task, "/fr5/task")

        # 진행 중인 액션 호출 추적용 (타임아웃/취소/스테일 결과 무시)
        self._goal_token = 0
        self._active_goal_handle = None
        self._active_timer = None

        # 서비스
        self.create_service(PlaceOrder, "/factory/order", self._on_order)
        self.create_service(GetInventory, "/factory/get_inventory", self._on_get_inventory)
        self.create_service(RequestInbound, "/factory/request_inbound", self._on_request_inbound)
        self.create_service(RetryOrCancel, "/factory/retry_or_cancel", self._on_retry_or_cancel)

        # 발행
        self.state_pub = self.create_publisher(FactoryState, "/factory/state", 10)
        self.inv_pub = self.create_publisher(Inventory, "/factory/inventory", 10)

        # 1~2Hz 주기 상태 발행 (유니티가 늦게 켜져도 현재 상태를 따라잡게)
        self.create_timer(0.5, self._publish_state)
        # 0.2초 틱: 큐에서 다음 Job 꺼내기
        self.create_timer(0.2, self._tick)

        self._recover_unfinished_jobs()
        self._publish_inventory()
        self.get_logger().info("orchestrator_node 시작됨")

    # ------------------------------------------------------------------
    # 재시작 복구
    # ------------------------------------------------------------------
    def _recover_unfinished_jobs(self):
        """재시작 시 미완료 Job을 전부 ERROR로 표시하고 예약을 해제한다.

        로봇이 실제로 어디까지 진행했는지 알 수 없으므로 자동 재개는 하지 않는다
        (멘토 피드백). 예약(reserved/incoming)만 안전하게 풀어서 재고가 영구히
        묶이는 것을 막는다. 실제 물리적으로 작업이 완료됐었는지는 사람이
        현장을 보고 /factory/retry_or_cancel 이전에 DB를 직접 확인해야 한다.
        """
        rows = self.db.unfinished_jobs()
        for row in rows:
            if row["job_type"] == JobType.OUTBOUND.value:
                self.db.release(row["item_id"])
            else:
                self.db.release_inbound(row["item_id"])
            self.db.set_job_status(row["job_id"], JobStatus.ERROR.value)
            self.get_logger().warn(
                f"재시작 복구: job_id={row['job_id']} "
                f"({row['job_type']}, item={row['item_id']})를 ERROR로 표시하고 "
                f"예약을 해제했습니다. 현장을 확인한 뒤 /factory/retry_or_cancel로 "
                f"처리하세요."
            )

    # ------------------------------------------------------------------
    # 서비스 콜백
    # ------------------------------------------------------------------
    def _on_order(self, request, response):
        if not self.db.reserve(request.item_id):
            response.accepted = False
            response.order_id = -1
            response.reason = "OUT_OF_STOCK"
            return response

        order_id = self.db.create_order(request.item_id, request.customer)
        level = self.db.get_level_by_item(request.item_id)
        job = JobBuilder.build_outbound(order_id, request.item_id, level)
        job.job_id = self.db.save_job(job)
        self.queue.append(job)

        response.accepted = True
        response.order_id = order_id
        response.reason = ""
        return response

    def _on_request_inbound(self, request, response):
        if not self.db.reserve_inbound(request.item_id):
            response.accepted = False
            response.reason = "SLOT_FULL"
            return response

        level = self.db.get_level_by_item(request.item_id)
        job = JobBuilder.build_inbound(request.item_id, level)
        job.job_id = self.db.save_job(job)
        self.queue.append(job)

        response.accepted = True
        response.reason = ""
        return response

    def _on_get_inventory(self, request, response):
        response.slots = [self._slot_to_msg(s) for s in self.db.list_slots()]
        return response

    def _on_retry_or_cancel(self, request, response):
        if request.action == "RETRY":
            response.ok = self.runner.retry()
        elif request.action == "CANCEL":
            response.ok = self.runner.cancel()
        else:
            response.ok = False
        self._publish_inventory()
        return response

    # ------------------------------------------------------------------
    # 메인 틱: 큐에서 다음 Job 실행
    # ------------------------------------------------------------------
    def _tick(self):
        if self.runner.busy:
            return
        if not self.queue:
            return
        self.runner.start(self.queue.popleft())

    def _on_job_change(self, job: Job):
        self._last_message = job.steps[job.current].kind.value if job.status == JobStatus.ERROR else ""
        self._publish_state()
        if job.status in (JobStatus.DONE, JobStatus.CANCELLED, JobStatus.ERROR):
            self._publish_inventory()

    # ------------------------------------------------------------------
    # Step 실행 (JobRunner가 호출)
    # ------------------------------------------------------------------
    def _execute_step(self, step: Step, result_cb):
        """result_cb(success: bool, failed_phase: str, message: str)"""
        if step.kind == StepKind.GO_TO:
            goal = GoToStation.Goal()
            goal.station = step.params["station"]
            self._call_action(
                self.limo_client, goal, self._timeouts[StepKind.GO_TO],
                lambda ok, phase, msg: result_cb(ok, "", msg),
            )
        elif step.kind == StepKind.FR5_TASK:
            goal = Fr5Task.Goal()
            goal.task_type = step.params["task_type"]
            goal.level = step.params["level"]
            self._call_action(
                self.fr5_client, goal, self._timeouts[StepKind.FR5_TASK], result_cb,
            )
        elif step.kind == StepKind.DB_COMMIT_IN:
            self.db.add_stock(step.params["item_id"])
            result_cb(True, "", "")
        elif step.kind == StepKind.DB_COMMIT_OUT:
            self.db.commit_out(step.params["item_id"])
            result_cb(True, "", "")
        elif step.kind == StepKind.DB_RELEASE:
            self.db.release(step.params["item_id"])
            result_cb(True, "", "")
        elif step.kind == StepKind.MARK_SHIPPED:
            self.db.set_order_status(step.params["order_id"], "SHIPPED")
            result_cb(True, "", "")
        else:
            result_cb(False, "", f"UNKNOWN_STEP_KIND:{step.kind}")

    # ------------------------------------------------------------------
    # 액션 호출 공통 로직: 타임아웃 → 취소, 토큰으로 stale 결과 무시
    # ------------------------------------------------------------------
    def _call_action(self, client: ActionClient, goal_msg, timeout_s: float, result_cb):
        self._goal_token += 1
        token = self._goal_token
        self._active_goal_handle = None

        if not client.wait_for_server(timeout_sec=2.0):
            result_cb(False, "", "ACTION_SERVER_UNAVAILABLE")
            return

        def _cancel_timer():
            if self._active_timer is not None:
                self._active_timer.cancel()
                self.destroy_timer(self._active_timer)
                self._active_timer = None

        def _on_timeout():
            if token != self._goal_token:
                return
            _cancel_timer()
            self.get_logger().warn(f"액션 타임아웃 (token={token}) → 취소 요청")
            if self._active_goal_handle is not None:
                self._active_goal_handle.cancel_goal_async()
            self._goal_token += 1  # 이후 들어오는 결과는 전부 stale 처리
            result_cb(False, "", "TIMEOUT")

        self._active_timer = self.create_timer(timeout_s, _on_timeout)

        def _on_goal_result(future):
            if token != self._goal_token:
                return  # 타임아웃으로 이미 처리됨
            _cancel_timer()
            try:
                result = future.result().result
            except Exception as exc:  # noqa: BLE001
                result_cb(False, "", f"RESULT_EXCEPTION:{exc}")
                return
            failed_phase = getattr(result, "failed_phase", "")
            result_cb(result.success, failed_phase, result.message)

        def _on_goal_response(future):
            if token != self._goal_token:
                return
            goal_handle = future.result()
            if not goal_handle.accepted:
                _cancel_timer()
                result_cb(False, "", "GOAL_REJECTED")
                return
            self._active_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(_on_goal_result)

        client.send_goal_async(goal_msg).add_done_callback(_on_goal_response)

    # ------------------------------------------------------------------
    # 발행 헬퍼
    # ------------------------------------------------------------------
    def _publish_state(self):
        msg = FactoryState()
        job = self.runner.current_job
        if job is None:
            msg.mode = FactoryState.MODE_RUNNING if self.queue else FactoryState.MODE_IDLE
            msg.job_type = ""
            msg.order_id = -1
            msg.step = ""
        else:
            msg.mode = (
                FactoryState.MODE_ERROR if job.status == JobStatus.ERROR else FactoryState.MODE_RUNNING
            )
            msg.job_type = job.job_type.value
            msg.order_id = job.order_id if job.order_id is not None else -1
            msg.step = job.steps[job.current].kind.value if job.current < len(job.steps) else ""
        msg.message = self._last_message
        msg.queue_length = len(self.queue)
        self.state_pub.publish(msg)

    def _publish_inventory(self):
        msg = Inventory()
        msg.slots = [self._slot_to_msg(s) for s in self.db.list_slots()]
        self.inv_pub.publish(msg)

    @staticmethod
    def _slot_to_msg(row: dict) -> SlotStock:
        msg = SlotStock()
        msg.slot_id = row["slot_id"]
        msg.level = row["level"]
        msg.item_id = row["item_id"]
        msg.item_name = row.get("item_name", "")
        msg.quantity = row["quantity"]
        msg.reserved = row["reserved"]
        return msg


def main():
    rclpy.init()
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
