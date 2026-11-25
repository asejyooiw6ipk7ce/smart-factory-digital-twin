"""
inventory_db.py (WBS 1.2 / 1.6)

SQLite 기반 재고 DB. orchestrator_node.py와 job_runner.py가 기대하는
메서드 이름/동작을 그대로 구현한다 (1.8, 1.4, 1.5에서 이미 이 인터페이스를
가정하고 코드/테스트를 작성해뒀다).

핵심 설계: "예약(reserve) -> 확정(commit) / 해제(release)" 패턴을
출고와 입고에 각각 대칭으로 적용한다.

  출고: reserve(주문 시점, reserved+1) -> commit_out(FR5가 실제로 꺼낸 시점, quantity-1·reserved-1)
                                       -> 실패/취소 시 release(reserved-1)

  입고: reserve_inbound(요청 시점, incoming+1) -> add_stock(FR5가 실제로 올린 시점, quantity+1·incoming-1)
                                               -> 실패/취소 시 release_inbound(incoming-1)

reserve 단계에서 미리 DB에 UPDATE ... WHERE 조건으로 자리를 확보해두기 때문에,
"자리가 있는지 확인 -> 나중에 자리가 사라짐" 같은 경쟁 상태(race condition)가
생기지 않는다 (SQLite의 단일 트랜잭션이 WHERE 조건과 UPDATE를 원자적으로 처리).
"""

import sqlite3
from pathlib import Path
from typing import Any, Optional

import yaml


class InventoryDB:
    def __init__(self, db_path: str, schema_path: str, config_path: str):
        self.db_path = db_path
        self._conn = sqlite3.connect(db_path)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA foreign_keys = ON")
        self._apply_schema(schema_path)
        self._seed_from_config_if_empty(config_path)

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def _apply_schema(self, schema_path: str) -> None:
        sql = Path(schema_path).read_text(encoding="utf-8")
        self._conn.executescript(sql)
        self._conn.commit()

    def _seed_from_config_if_empty(self, config_path: str) -> None:
        """slots 테이블이 비어있을 때만 factory.yaml로 초기 데이터를 채운다.
        (이미 데이터가 있으면, 즉 2번째 이후 실행이면 아무것도 하지 않는다 —
        재시작할 때마다 재고가 초기화되면 안 되므로.)"""
        row = self._conn.execute("SELECT COUNT(*) AS n FROM slots").fetchone()
        if row["n"] > 0:
            return
        path = Path(config_path)
        if not path.exists():
            return
        config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for slot in config.get("slots", []):
            self._conn.execute(
                "INSERT INTO slots (level, item_id, item_name, quantity, reserved, incoming, capacity) "
                "VALUES (?, ?, ?, ?, 0, 0, ?)",
                (
                    slot["level"],
                    slot["item_id"],
                    slot["item_name"],
                    slot.get("initial_quantity", 0),
                    slot["capacity"],
                ),
            )
        self._conn.commit()

    # ------------------------------------------------------------------
    # 출고: reserve -> commit_out / release
    # ------------------------------------------------------------------
    def reserve(self, item_id: int) -> bool:
        """출고 주문이 들어왔을 때, 실제로 꺼낼 수 있는 재고가 있으면 미리 찜한다.
        반환값이 False면 재고 부족(OUT_OF_STOCK)."""
        cur = self._conn.execute(
            "UPDATE slots SET reserved = reserved + 1 "
            "WHERE item_id = ? AND (quantity - reserved) > 0",
            (item_id,),
        )
        self._conn.commit()
        return cur.rowcount > 0

    def commit_out(self, item_id: int) -> None:
        """FR5가 실제로 선반에서 꺼내 Limo에 실었을 때 확정 반영."""
        self._conn.execute(
            "UPDATE slots SET quantity = quantity - 1, reserved = reserved - 1 WHERE item_id = ?",
            (item_id,),
        )
        self._conn.commit()

    def release(self, item_id: int) -> None:
        """출고가 취소되거나 복구 불가능하게 실패했을 때, 찜해둔 자리를 풀어준다."""
        self._conn.execute(
            "UPDATE slots SET reserved = MAX(reserved - 1, 0) WHERE item_id = ?",
            (item_id,),
        )
        self._conn.commit()

    # ------------------------------------------------------------------
    # 입고: reserve_inbound -> add_stock / release_inbound
    # ------------------------------------------------------------------
    def reserve_inbound(self, item_id: int) -> bool:
        """입고 요청이 들어왔을 때, 선반에 넣을 빈 자리가 있으면 미리 찜한다.
        반환값이 False면 자리 부족(SLOT_FULL)."""
        cur = self._conn.execute(
            "UPDATE slots SET incoming = incoming + 1 "
            "WHERE item_id = ? AND (quantity + incoming) < capacity",
            (item_id,),
        )
        self._conn.commit()
        return cur.rowcount > 0

    def add_stock(self, item_id: int) -> None:
        """FR5가 실제로 선반에 올려놨을 때 확정 반영."""
        self._conn.execute(
            "UPDATE slots SET quantity = quantity + 1, incoming = incoming - 1 WHERE item_id = ?",
            (item_id,),
        )
        self._conn.commit()

    def release_inbound(self, item_id: int) -> None:
        """입고가 취소되거나 복구 불가능하게 실패했을 때, 찜해둔 자리를 풀어준다."""
        self._conn.execute(
            "UPDATE slots SET incoming = MAX(incoming - 1, 0) WHERE item_id = ?",
            (item_id,),
        )
        self._conn.commit()

    # ------------------------------------------------------------------
    # 조회
    # ------------------------------------------------------------------
    def list_slots(self) -> list[dict[str, Any]]:
        cur = self._conn.execute(
            "SELECT slot_id, level, item_id, item_name, quantity, reserved "
            "FROM slots ORDER BY level"
        )
        return [dict(row) for row in cur.fetchall()]

    def get_level_by_item(self, item_id: int) -> int:
        row = self._conn.execute(
            "SELECT level FROM slots WHERE item_id = ?", (item_id,)
        ).fetchone()
        if row is None:
            raise ValueError(f"item_id={item_id}에 해당하는 선반을 찾을 수 없습니다")
        return row["level"]

    # ------------------------------------------------------------------
    # 주문
    # ------------------------------------------------------------------
    def create_order(self, item_id: int, customer: str) -> int:
        cur = self._conn.execute(
            "INSERT INTO orders (item_id, customer, status) VALUES (?, ?, 'PENDING')",
            (item_id, customer),
        )
        self._conn.commit()
        return cur.lastrowid

    def set_order_status(self, order_id: int, status: str) -> None:
        self._conn.execute(
            "UPDATE orders SET status = ? WHERE order_id = ?", (status, order_id)
        )
        self._conn.commit()

    # ------------------------------------------------------------------
    # Job (재시작 복구용 영속 기록)
    # ------------------------------------------------------------------
    def save_job(self, job) -> int:
        cur = self._conn.execute(
            "INSERT INTO jobs (job_type, item_id, order_id, status) VALUES (?, ?, ?, ?)",
            (job.job_type.value, job.item_id, job.order_id, job.status.value),
        )
        self._conn.commit()
        return cur.lastrowid

    def set_job_status(self, job_id: int, status: str) -> None:
        self._conn.execute(
            "UPDATE jobs SET status = ?, updated_at = datetime('now') WHERE job_id = ?",
            (status, job_id),
        )
        self._conn.commit()

    def unfinished_jobs(self) -> list[dict[str, Any]]:
        """재시작 시, 전원이 꺼지기 전에 끝내지 못한(PENDING/RUNNING) Job들을 찾는다."""
        cur = self._conn.execute(
            "SELECT job_id, job_type, item_id, order_id FROM jobs "
            "WHERE status IN ('PENDING', 'RUNNING')"
        )
        return [dict(row) for row in cur.fetchall()]

    # ------------------------------------------------------------------
    def close(self) -> None:
        self._conn.close()
