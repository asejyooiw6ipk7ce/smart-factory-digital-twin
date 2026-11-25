-- schema.sql (WBS 1.2 / 1.6)
--
-- 팩토리 프로젝트 재고 DB 스키마.
-- InventoryDB가 처음 열릴 때 이 파일을 통째로 실행해서 테이블을 만든다
-- (executescript, IF NOT EXISTS라서 여러 번 실행해도 안전함).

PRAGMA foreign_keys = ON;

-- 선반의 "층(level)" 하나가 slot 한 줄이다.
-- 이 프로젝트는 1열 선반이라 level == slot이라고 봐도 된다.
CREATE TABLE IF NOT EXISTS slots (
    slot_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    level     INTEGER NOT NULL UNIQUE,
    item_id   INTEGER NOT NULL UNIQUE,
    item_name TEXT    NOT NULL,

    quantity  INTEGER NOT NULL DEFAULT 0,  -- 실제 확정된 재고 수
    reserved  INTEGER NOT NULL DEFAULT 0,  -- 출고 주문이 찜해 둔 수량 (아직 안 나감)
    incoming  INTEGER NOT NULL DEFAULT 0,  -- 입고가 찜해 둔 자리 (아직 안 들어옴)
    capacity  INTEGER NOT NULL,            -- 이 층에 들어갈 수 있는 최대 수량

    CHECK (quantity >= 0),
    CHECK (reserved >= 0 AND reserved <= quantity),
    CHECK (incoming >= 0),
    CHECK (quantity + incoming <= capacity)
);

-- 출고 주문 1건 = 행 1개.
CREATE TABLE IF NOT EXISTS orders (
    order_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    item_id    INTEGER NOT NULL,
    customer   TEXT    NOT NULL,
    status     TEXT    NOT NULL DEFAULT 'PENDING',  -- PENDING / SHIPPED / CANCELLED
    created_at TEXT    NOT NULL DEFAULT (datetime('now'))
);

-- 오케스트레이터가 처리하는 Job(입고 1건 또는 출고 1건)의 영속 기록.
-- 재시작 복구(1.8의 _recover_unfinished_jobs)가 PENDING/RUNNING 상태로 남아있는
-- 행을 찾아서 예약을 풀어주고 ERROR로 표시하는 데 쓴다.
CREATE TABLE IF NOT EXISTS jobs (
    job_id     INTEGER PRIMARY KEY AUTOINCREMENT,
    job_type   TEXT    NOT NULL,                    -- INBOUND / OUTBOUND
    item_id    INTEGER NOT NULL,
    order_id   INTEGER,                             -- 출고일 때만 채워짐
    status     TEXT    NOT NULL DEFAULT 'PENDING',   -- PENDING/RUNNING/DONE/ERROR/CANCELLED
    created_at TEXT    NOT NULL DEFAULT (datetime('now')),
    updated_at TEXT    NOT NULL DEFAULT (datetime('now'))
);

CREATE INDEX IF NOT EXISTS idx_jobs_status ON jobs (status);
