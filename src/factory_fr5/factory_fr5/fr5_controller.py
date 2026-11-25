"""
fr5_controller.py (WBS 3.3)

FR5의 PICK/PLACE/HOME 작업 순서를 ROS2와 완전히 분리된 순수 파이썬으로
구현한다 (job_runner.py/coarse_mover.py/aruco_docker.py와 같은 설계 원칙).

Fr5Task.action 계약(factory_interfaces)과 1:1로 맞춘다:
    goal:   task_type (INBOUND/OUTBOUND), level
    result: success, failed_phase (PICK/PLACE/HOME), message

- INBOUND: Limo 위 짐칸에서 집어서(PICK) 선반 level에 올린다(PLACE).
- OUTBOUND: 선반 level에서 집어서(PICK) Limo 위 짐칸에 올린다(PLACE).
- 두 경우 모두 끝나면 HOME으로 복귀한다.

PICK이 끝나면 "진짜로 뭔가를 집었는지"(check_grasped)를 확인한다 —
그리퍼가 close_pos까지 완전히 닫혔다면 아무것도 안 집힌 것이므로 실패로 본다.
이 판정이 job.py의 RetryPolicy.IF_PICK_FAILED(최대 2회 재시도)가 걸리는
지점이다. PLACE/HOME 실패는 재시도하지 않고 바로 사람이 확인해야 하는
ERROR로 올라간다 (orchestrator_node.py의 기존 설계와 동일한 철학).

hw(하드웨어 객체)는 아래 메서드만 있으면 되는 duck-typed 인터페이스다
(실제 구현은 fairino_client.py의 Fr5Hardware, 테스트에서는 FakeHardware):
    hw.move_joint(joint_pos: list[float]) -> bool
    hw.gripper_close() -> bool
    hw.gripper_open() -> bool
    hw.check_grasped() -> bool
"""

from typing import Callable, Optional


class Phase:
    PICK = "PICK"
    PLACE = "PLACE"
    HOME = "HOME"


class Fr5TaskExecutor:
    def __init__(self, hw, config: dict):
        self.hw = hw
        self.config = config

    def run(
        self,
        task_type: str,
        level: int,
        on_phase: Optional[Callable[[str], None]] = None,
    ) -> tuple[bool, str, str]:
        """성공하면 (True, "", ""), 실패하면 (False, failed_phase, message)."""
        if level not in self.config["shelf"]:
            return False, Phase.PICK, f"UNKNOWN_SHELF_LEVEL:{level}"

        shelf_pose = self.config["shelf"][level]
        limo_pose = self.config["limo_pose"]

        if task_type == "INBOUND":
            pick_pose, place_pose = limo_pose, shelf_pose
        elif task_type == "OUTBOUND":
            pick_pose, place_pose = shelf_pose, limo_pose
        else:
            return False, Phase.PICK, f"UNKNOWN_TASK_TYPE:{task_type}"

        if on_phase:
            on_phase(Phase.PICK)
        ok, msg = self._do_pick(pick_pose)
        if not ok:
            return False, Phase.PICK, msg

        if on_phase:
            on_phase(Phase.PLACE)
        ok, msg = self._do_place(place_pose)
        if not ok:
            return False, Phase.PLACE, msg

        if on_phase:
            on_phase(Phase.HOME)
        ok, msg = self._do_home()
        if not ok:
            return False, Phase.HOME, msg

        return True, "", ""

    # ------------------------------------------------------------------
    def _do_pick(self, pose: dict) -> tuple[bool, str]:
        if not self.hw.move_joint(pose["approach_joint"]):
            return False, "MOVE_APPROACH_FAILED"
        if not self.hw.move_joint(pose["target_joint"]):
            return False, "MOVE_TARGET_FAILED"
        if not self.hw.gripper_close():
            return False, "GRIPPER_CLOSE_FAILED"
        if not self.hw.check_grasped():
            # 아무것도 안 집혔으면 안전하게 다시 벌려서(놓치고 있던 걸 더
            # 조이고 있지 않게) 깔끔한 상태로 ERROR를 올린다.
            self.hw.gripper_open()
            return False, "GRASP_NOT_DETECTED"
        if not self.hw.move_joint(pose["retreat_joint"]):
            return False, "MOVE_RETREAT_FAILED"
        return True, ""

    def _do_place(self, pose: dict) -> tuple[bool, str]:
        if not self.hw.move_joint(pose["approach_joint"]):
            return False, "MOVE_APPROACH_FAILED"
        if not self.hw.move_joint(pose["target_joint"]):
            return False, "MOVE_TARGET_FAILED"
        if not self.hw.gripper_open():
            return False, "GRIPPER_OPEN_FAILED"
        if not self.hw.move_joint(pose["retreat_joint"]):
            return False, "MOVE_RETREAT_FAILED"
        return True, ""

    def _do_home(self) -> tuple[bool, str]:
        if not self.hw.move_joint(self.config["home_joint"]):
            return False, "MOVE_HOME_FAILED"
        return True, ""
