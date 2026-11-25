"""test_fr5_controller.py (WBS 3.3) — fr5_controller.py의 순수 로직 단위 테스트.

FakeHardware로 실제 로봇/그리퍼 없이 PICK/PLACE/HOME 단계 분기와
failed_phase 보고가 정확한지 검증한다.
"""

from factory_fr5.fr5_controller import Fr5TaskExecutor

CONFIG = {
    "home_joint": [0, 0, 0, 0, 0, 0],
    "limo_pose": {
        "approach_joint": [1, 0, 0, 0, 0, 0],
        "target_joint": [2, 0, 0, 0, 0, 0],
        "retreat_joint": [3, 0, 0, 0, 0, 0],
    },
    "shelf": {
        1: {
            "approach_joint": [10, 0, 0, 0, 0, 0],
            "target_joint": [20, 0, 0, 0, 0, 0],
            "retreat_joint": [30, 0, 0, 0, 0, 0],
        }
    },
}


class FakeHardware:
    def __init__(self, fail_on=None, grasp_detected=True):
        """fail_on: {"move_joint": [joint_pos, ...], "gripper_close": True, ...}
        처럼 실패시킬 호출을 지정. move_joint는 특정 joint_pos 리스트와
        일치할 때만 실패하게 해서, 어느 단계(approach/target/retreat)에서
        실패했는지까지 테스트로 구분할 수 있게 한다."""
        self.fail_on = fail_on or {}
        self.grasp_detected = grasp_detected
        self.calls = []

    def move_joint(self, joint_pos):
        self.calls.append(("move_joint", joint_pos))
        bad = self.fail_on.get("move_joint_at")
        if bad is not None and joint_pos == bad:
            return False
        return True

    def gripper_close(self):
        self.calls.append(("gripper_close",))
        return not self.fail_on.get("gripper_close", False)

    def gripper_open(self):
        self.calls.append(("gripper_open",))
        return not self.fail_on.get("gripper_open", False)

    def check_grasped(self):
        self.calls.append(("check_grasped",))
        return self.grasp_detected


def test_inbound_success_picks_from_limo_places_on_shelf():
    hw = FakeHardware()
    executor = Fr5TaskExecutor(hw, CONFIG)

    ok, failed_phase, msg = executor.run("INBOUND", 1)

    assert (ok, failed_phase, msg) == (True, "", "")
    # INBOUND는 limo_pose에서 집어서 shelf[1]에 놓아야 한다.
    moved = [c[1] for c in hw.calls if c[0] == "move_joint"]
    assert moved == [
        [1, 0, 0, 0, 0, 0],   # limo approach
        [2, 0, 0, 0, 0, 0],   # limo target
        [3, 0, 0, 0, 0, 0],   # limo retreat
        [10, 0, 0, 0, 0, 0],  # shelf approach
        [20, 0, 0, 0, 0, 0],  # shelf target
        [30, 0, 0, 0, 0, 0],  # shelf retreat
        [0, 0, 0, 0, 0, 0],   # home
    ]


def test_outbound_success_picks_from_shelf_places_on_limo():
    hw = FakeHardware()
    executor = Fr5TaskExecutor(hw, CONFIG)

    ok, failed_phase, msg = executor.run("OUTBOUND", 1)

    assert (ok, failed_phase, msg) == (True, "", "")
    moved = [c[1] for c in hw.calls if c[0] == "move_joint"]
    assert moved[0] == [10, 0, 0, 0, 0, 0]  # shelf approach (pick 쪽이 선반)
    assert moved[3] == [1, 0, 0, 0, 0, 0]   # limo approach (place 쪽이 limo)


def test_grasp_not_detected_reports_pick_failure_and_reopens_gripper():
    hw = FakeHardware(grasp_detected=False)
    executor = Fr5TaskExecutor(hw, CONFIG)

    ok, failed_phase, msg = executor.run("INBOUND", 1)

    assert ok is False
    assert failed_phase == "PICK"
    assert msg == "GRASP_NOT_DETECTED"
    # 안전하게 다시 벌리는 동작까지 호출됐는지 확인
    assert ("gripper_open",) in hw.calls


def test_move_failure_during_place_reports_place_phase():
    hw = FakeHardware(fail_on={"move_joint_at": [20, 0, 0, 0, 0, 0]})
    executor = Fr5TaskExecutor(hw, CONFIG)

    ok, failed_phase, msg = executor.run("INBOUND", 1)

    assert ok is False
    assert failed_phase == "PLACE"
    assert msg == "MOVE_TARGET_FAILED"


def test_move_failure_during_home_reports_home_phase():
    hw = FakeHardware(fail_on={"move_joint_at": [0, 0, 0, 0, 0, 0]})
    executor = Fr5TaskExecutor(hw, CONFIG)

    ok, failed_phase, msg = executor.run("INBOUND", 1)

    assert ok is False
    assert failed_phase == "HOME"
    assert msg == "MOVE_HOME_FAILED"


def test_unknown_shelf_level_reports_pick_failure():
    hw = FakeHardware()
    executor = Fr5TaskExecutor(hw, CONFIG)

    ok, failed_phase, msg = executor.run("INBOUND", 99)

    assert ok is False
    assert failed_phase == "PICK"
    assert "UNKNOWN_SHELF_LEVEL" in msg


def test_on_phase_callback_called_in_order():
    hw = FakeHardware()
    executor = Fr5TaskExecutor(hw, CONFIG)
    seen = []

    executor.run("INBOUND", 1, on_phase=seen.append)

    assert seen == ["PICK", "PLACE", "HOME"]
