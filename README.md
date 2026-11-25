# 팩토리 프로젝트

FR5(로봇팔) + Limo Pro(모바일 로봇)로 입출고를 자동화하고, SQLite 재고 DB와
Unity 디지털트윈을 연동하는 다중로봇 제어 프로젝트.

- 입고: Limo 위에 물건을 올리고 확인 버튼 → Limo가 ArUco 도킹 지점으로 이동 →
  FR5가 집어서 선반에 올림 → DB 재고 +1
- 출고: 유니티(또는 다른 클라이언트)에서 주문 → Limo가 도킹 지점으로 이동 →
  FR5가 선반에서 집어 Limo 위에 올림 → DB 재고 -1 → Limo 복귀
- 유니티에 로봇 상태·재고 상태를 실시간 미러링

## WBS / 일정
작업 분류와 일정은 Notion 데이터베이스에서 관리합니다:
https://app.notion.com/p/423b51d04f15816dbc9ecd819d488e55

## 폴더 구조
```
factory_ws/src/
├── factory_interfaces/   # action/srv/msg 정의 (WBS 1.1)
├── factory_core/         # 오케스트레이터, Job/Step, InventoryDB (WBS 1.2~1.9)
├── factory_fr5/          # FR5 제어 (WBS 3.x)
├── factory_limo/         # Limo 온보드 (WBS 2.x)
├── factory_mock/         # 1일차용 가짜 로봇 노드
└── factory_bringup/      # launch 파일
```

## 빌드
```bash
cd factory_ws
colcon build
source install/setup.bash
```

## 현재 진행 상태
- [x] 1.1 factory_interfaces 작성
- [x] 1.8 orchestrator_node.py (job.py/job_runner.py/inventory_db.py 작성 전이라 아직 실행은 안 됨)
- [ ] 나머지는 Notion WBS 참고
