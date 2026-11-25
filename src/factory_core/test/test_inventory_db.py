"""
test_inventory_db.py (WBS 1.6)

InventoryDB를 메모리상의 임시 SQLite 파일로 열어서 검증하는 단위 테스트.
실제 factory.db를 건드리지 않도록 매 테스트마다 pytest의 tmp_path로 새
DB/임시 설정 파일을 만든다.

실행 방법:
    cd factory_ws/src/factory_core
    PYTHONPATH=. python3 -m pytest test/test_inventory_db.py -v
"""

import sqlite3

import pytest
import yaml

from factory_core.inventory_db import InventoryDB
from factory_core.job import Job, JobStatus, JobType, Step, StepKind

SCHEMA_PATH = "../../db/schema.sql"  # factory_core/test/ 기준 상대경로


@pytest.fixture
def db(tmp_path):
    config = {
        "slots": [
            {"level": 1, "item_id": 1, "item_name": "부품A", "initial_quantity": 3, "capacity": 5},
            {"level": 2, "item_id": 2, "item_name": "부품B", "initial_quantity": 0, "capacity": 2},
        ]
    }
    config_path = tmp_path / "factory.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    db_path = tmp_path / "factory.db"
    instance = InventoryDB(str(db_path), SCHEMA_PATH, str(config_path))
    yield instance
    instance.close()


# ----------------------------------------------------------------------
# 시드 데이터
# ----------------------------------------------------------------------
def test_seed_from_config_populates_slots(db):
    slots = {s["item_id"]: s for s in db.list_slots()}
    assert slots[1]["quantity"] == 3
    assert slots[1]["level"] == 1
    assert slots[1]["item_name"] == "부품A"
    assert slots[2]["quantity"] == 0


def test_seed_is_not_reapplied_on_reopen(tmp_path):
    config = {"slots": [{"level": 1, "item_id": 1, "item_name": "부품A", "initial_quantity": 3, "capacity": 5}]}
    config_path = tmp_path / "factory.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    db_path = tmp_path / "factory.db"

    db1 = InventoryDB(str(db_path), SCHEMA_PATH, str(config_path))
    db1.commit_out_called = db1.reserve(1) and db1.commit_out(1)  # quantity 3 -> 2
    db1.close()

    # 재시작 (같은 DB 파일, 같은 config) -> quantity가 다시 3으로 초기화되면 안 됨
    db2 = InventoryDB(str(db_path), SCHEMA_PATH, str(config_path))
    slot = db2.list_slots()[0]
    assert slot["quantity"] == 2
    db2.close()


# ----------------------------------------------------------------------
# 출고: reserve / commit_out / release
# ----------------------------------------------------------------------
def test_reserve_succeeds_when_stock_available(db):
    assert db.reserve(1) is True
    slot = next(s for s in db.list_slots() if s["item_id"] == 1)
    assert slot["reserved"] == 1
    assert slot["quantity"] == 3  # 확정 전이라 quantity는 그대로


def test_reserve_fails_when_out_of_stock(db):
    # item_id=2는 initial_quantity=0
    assert db.reserve(2) is False


def test_reserve_cannot_exceed_available_quantity(db):
    # item_id=1은 quantity=3. 3번 reserve는 성공, 4번째는 실패해야 함
    assert db.reserve(1) is True
    assert db.reserve(1) is True
    assert db.reserve(1) is True
    assert db.reserve(1) is False


def test_commit_out_decrements_quantity_and_reserved(db):
    db.reserve(1)
    db.commit_out(1)
    slot = next(s for s in db.list_slots() if s["item_id"] == 1)
    assert slot["quantity"] == 2
    assert slot["reserved"] == 0


def test_release_undoes_reservation_without_touching_quantity(db):
    db.reserve(1)
    db.release(1)
    slot = next(s for s in db.list_slots() if s["item_id"] == 1)
    assert slot["reserved"] == 0
    assert slot["quantity"] == 3  # release는 취소일 뿐, 재고 자체는 안 건드림


def test_release_does_not_go_negative(db):
    db.release(1)  # reserve 안 했는데 release 호출 (방어적 케이스)
    slot = next(s for s in db.list_slots() if s["item_id"] == 1)
    assert slot["reserved"] == 0


# ----------------------------------------------------------------------
# 입고: reserve_inbound / add_stock / release_inbound
# ----------------------------------------------------------------------
def test_reserve_inbound_succeeds_when_capacity_available(db):
    # item_id=2: quantity=0, capacity=2 -> 2번까지 reserve_inbound 가능
    assert db.reserve_inbound(2) is True
    assert db.reserve_inbound(2) is True
    assert db.reserve_inbound(2) is False  # 3번째는 capacity 초과


def test_add_stock_increments_quantity_and_decrements_incoming(db):
    # capacity=2인 item_id=2에 reserve_inbound를 2번(한도까지) 걸어두고,
    # 그중 하나를 add_stock으로 확정한다. add_stock은 "약속된 자리(incoming)"를
    # "실제로 채워진 자리(quantity)"로 바꾸는 것뿐이라, quantity+incoming 합계
    # 자체는 줄지 않는다 — 그래서 그 상태에서 새 reserve_inbound는 여전히 실패해야
    # 하고(물리적으로 꽉 차 있으므로), 반대로 release_inbound를 하면 그제서야
    # 자리가 실제로 빈다. list_slots는 incoming을 노출하지 않으므로(오케스트레이터가
    # Unity로 보내는 SlotStock에는 quantity/reserved만 필요) incoming 값 자체는
    # 원시 SQL로 직접 확인한다.
    db.reserve_inbound(2)
    db.reserve_inbound(2)
    assert db.reserve_inbound(2) is False  # capacity 꽉 참 (quantity=0, incoming=2)

    db.add_stock(2)  # quantity 0->1, incoming 2->1 (합계는 그대로 2)

    slot = next(s for s in db.list_slots() if s["item_id"] == 2)
    assert slot["quantity"] == 1

    conn = sqlite3.connect(db.db_path)
    incoming = conn.execute(
        "SELECT incoming FROM slots WHERE item_id = 2"
    ).fetchone()[0]
    conn.close()
    assert incoming == 1

    # 합계가 여전히 capacity(2)이므로 새 reserve_inbound는 계속 실패해야 함
    assert db.reserve_inbound(2) is False

    # release_inbound로 남은 incoming 하나를 풀어주면 비로소 자리가 생긴다
    db.release_inbound(2)
    assert db.reserve_inbound(2) is True


def test_release_inbound_undoes_reservation(db):
    db.reserve_inbound(2)
    db.release_inbound(2)
    # 다시 capacity(=2)만큼 reserve_inbound가 전부 성공해야 함 -> 자리가 풀렸다는 뜻
    assert db.reserve_inbound(2) is True
    assert db.reserve_inbound(2) is True


# ----------------------------------------------------------------------
# CHECK 제약 (capacity, quantity 하한)
# ----------------------------------------------------------------------
def test_capacity_check_constraint_blocks_raw_overflow(db):
    # InventoryDB의 공개 메서드로는 capacity를 못 넘기지만, DB 자체의
    # CHECK 제약도 살아있는지 직접 SQL로 확인한다 (방어의 마지막 층).
    conn = sqlite3.connect(db.db_path)
    with pytest.raises(sqlite3.IntegrityError):
        conn.execute(
            "UPDATE slots SET quantity = 999 WHERE item_id = 1"
        )  # capacity=5를 초과
    conn.close()


# ----------------------------------------------------------------------
# 주문 / Job 저장 및 재시작 복구 조회
# ----------------------------------------------------------------------
def test_create_order_and_set_order_status(db):
    order_id = db.create_order(item_id=1, customer="홍길동")
    assert isinstance(order_id, int)
    db.set_order_status(order_id, "SHIPPED")  # 에러 없이 통과하면 OK


def test_get_level_by_item(db):
    assert db.get_level_by_item(1) == 1
    assert db.get_level_by_item(2) == 2


def test_get_level_by_item_raises_for_unknown_item(db):
    with pytest.raises(ValueError):
        db.get_level_by_item(999)


def test_save_job_and_unfinished_jobs(db):
    job = Job(
        job_type=JobType.OUTBOUND,
        steps=[Step(kind=StepKind.GO_TO, params={"station": "LEVEL_1"})],
        item_id=1,
        order_id=10,
        status=JobStatus.RUNNING,
    )
    job_id = db.save_job(job)

    unfinished = db.unfinished_jobs()
    assert any(j["job_id"] == job_id and j["job_type"] == "OUTBOUND" for j in unfinished)

    db.set_job_status(job_id, JobStatus.DONE.value)
    unfinished_after = db.unfinished_jobs()
    assert all(j["job_id"] != job_id for j in unfinished_after)
