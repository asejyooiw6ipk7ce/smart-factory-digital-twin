"""
confirm_ui.py (WBS 2.7)

Limo 상판 모니터에 띄우는 "확인" 버튼 UI. 사람이 물건을 Limo 위에 얹고
어떤 물건인지 고른 뒤 확인 버튼을 누르면, /factory/request_inbound 서비스를
호출해서 입고 Job을 시작시킨다 (프로젝트 설명의 "확인 버튼을 누르면 Limo가
출발한다"는 그 버튼).

실행 모델에 대한 설명: Tkinter의 메인 루프(root.mainloop())와 ROS2의
rclpy.spin()은 둘 다 "자기가 이벤트 루프를 독점하고 싶어하는" 블로킹 호출이라
한 스레드에 같이 둘 수 없다. 그래서:
  - rclpy.spin()은 백그라운드 스레드에서 돌리고,
  - Tkinter는 메인 스레드에서 mainloop()을 돈다.
  - ROS 콜백(서비스 응답, FactoryState 구독)에서 Tkinter 위젯을 직접 건드리면
    스레드 충돌이 나므로, ROS 콜백은 공유 변수에 최신 값만 저장해두고,
    Tkinter 쪽에서 root.after()로 주기적으로 그 변수를 읽어 화면을 갱신한다
    (Tkinter는 메인 스레드에서만 위젯을 건드려야 하는 제약이 있기 때문).

참고: 이 환경에는 rclpy/tkinter 실행 환경이 없어서 실제로 띄워 검증하지는
못했다. 로직(서비스 호출, 스레드 간 값 전달)은 표준 패턴을 따랐지만, 실제
모니터 화면 크기/터치 조작감은 현장에서 확인 후 레이아웃을 조정하는 걸
권장한다.
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from factory_interfaces.msg import FactoryState
from factory_interfaces.srv import RequestInbound

# 화면에 보여줄 품목 목록 (item_id -> 표시 이름). config/factory.yaml의 slots와
# 맞춰서 관리한다. 품목이 추가되면 여기도 같이 늘려야 한다.
ITEM_CHOICES = {1: "부품A", 2: "부품B"}


class ConfirmUiNode(Node):
    """ROS 쪽 절반: 서비스 클라이언트 + 상태 구독만 담당하고, 화면은 모른다."""

    def __init__(self):
        super().__init__("confirm_ui")
        self._inbound_client = self.create_client(RequestInbound, "/factory/request_inbound")
        self.create_subscription(FactoryState, "/factory/state", self._on_state, 10)

        # Tkinter 스레드가 읽어가는 공유 상태 (락으로 보호)
        self._lock = threading.Lock()
        self.latest_state_text = "연결 대기 중..."

    def _on_state(self, msg: FactoryState) -> None:
        with self._lock:
            self.latest_state_text = (
                f"모드: {msg.mode} | 대기열: {msg.queue_length}"
                + (f" | {msg.message}" if msg.message else "")
            )

    def get_state_text(self) -> str:
        with self._lock:
            return self.latest_state_text

    def request_inbound(self, item_id: int, on_done) -> None:
        """비동기 호출. on_done(accepted: bool, reason: str)을 ROS 스레드에서 호출한다
        (Tkinter 쪽은 반드시 root.after로 안전하게 받아야 한다)."""
        if not self._inbound_client.service_is_ready():
            on_done(False, "SERVICE_NOT_READY")
            return
        req = RequestInbound.Request()
        req.item_id = item_id
        future = self._inbound_client.call_async(req)

        def _on_response(fut):
            try:
                res = fut.result()
                on_done(res.accepted, res.reason)
            except Exception as exc:  # noqa: BLE001
                on_done(False, f"EXCEPTION:{exc}")

        future.add_done_callback(_on_response)


class ConfirmUiApp:
    """Tkinter 쪽 절반: 메인 스레드에서만 위젯을 건드린다."""

    def __init__(self, node: ConfirmUiNode):
        self._node = node
        self._result_text = tk.StringVar(value="")
        self._busy = False

        self.root = tk.Tk()
        self.root.title("입고 확인")
        self.root.geometry("480x320")

        ttk.Label(self.root, text="입고할 물품을 고르세요", font=("", 16)).pack(pady=12)

        self._item_var = tk.IntVar(value=next(iter(ITEM_CHOICES)))
        for item_id, name in ITEM_CHOICES.items():
            ttk.Radiobutton(
                self.root, text=name, variable=self._item_var, value=item_id
            ).pack(anchor="w", padx=40)

        self._confirm_btn = ttk.Button(
            self.root, text="확인 (입고 시작)", command=self._on_confirm
        )
        self._confirm_btn.pack(pady=16, ipadx=20, ipady=10)

        ttk.Label(self.root, textvariable=self._result_text, foreground="blue").pack()
        self._state_label = ttk.Label(self.root, text="")
        self._state_label.pack(side="bottom", pady=8)

        self._poll_state()

    def _on_confirm(self) -> None:
        if self._busy:
            return
        self._busy = True
        self._confirm_btn.state(["disabled"])
        item_id = self._item_var.get()
        self._result_text.set("처리 중...")

        def on_done(accepted: bool, reason: str) -> None:
            # ROS 콜백 스레드에서 호출되므로, Tkinter 갱신은 after(0, ...)로 메인
            # 스레드에 넘긴다.
            self.root.after(0, lambda: self._finish(accepted, reason))

        self._node.request_inbound(item_id, on_done)

    def _finish(self, accepted: bool, reason: str) -> None:
        if accepted:
            self._result_text.set("입고 요청 접수됨")
        else:
            self._result_text.set(f"거부됨: {reason}")
        self._busy = False
        self._confirm_btn.state(["!disabled"])

    def _poll_state(self) -> None:
        self._state_label.config(text=self._node.get_state_text())
        self.root.after(500, self._poll_state)

    def run(self) -> None:
        self.root.mainloop()


def main():
    rclpy.init()
    node = ConfirmUiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = ConfirmUiApp(node)
    try:
        app.run()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
