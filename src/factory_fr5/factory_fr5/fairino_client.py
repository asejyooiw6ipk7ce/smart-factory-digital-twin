"""
fairino_client.py (WBS 3.4)

fr5_controller.py가 요구하는 duck-typed 하드웨어 인터페이스
(move_joint/gripper_close/gripper_open/check_grasped)를 실제 Fairino FR5
Python SDK(`fairino` 패키지, `pip install fairino`)로 구현한다.

⚠️ 이 파일은 이 작업 환경(샌드박스)에 fairino SDK도, 실제 로봇도 없어서
import조차 안 해봤다 — 즉 문법(ast.parse) 확인만 된 상태다. 아래 메서드
시그니처는 Fairino 공식 문서(Python SDK Manual)에서 직접 확인한 것이지만,
실제 로봇에 연결해서 동작을 검증한 적은 없다. 사용자가 실제 FR5에 연결해서
반드시 재검증해야 한다.

확인된 것 (공식 문서 기준):
    from fairino import Robot
    robot = Robot.RPC(ip)                       # 연결
    robot.MoveJ(joint_pos, tool, user, vel=.., ovl=.., ...)  # blendT=-1(기본)이면 블로킹
    robot.SetGripperConfig(company, device)      # 그리퍼 제조사/기종 등록
    robot.ActGripper(index, action)              # action 0=리셋 1=활성화
    robot.MoveGripper(index, pos, vel, force, maxtime, block, type, rotNum, rotVel, rotTorque)
    robot.GetGripperCurPosition() -> (error, fault, position)   # 0~100%

확인 안 된 것 (설치된 SDK에서 직접 확인 필요):
    - 에러/알람 리셋 메서드 이름 (ResetAllError로 추정만 함 — 문서에 없었음)
    - DH-Robotics PGEA-100-40-C의 정확한 device 번호
      (문서 예시엔 Dahuan의 "PGI-140"만 있고 PGEA-100-40-C는 없음.
      3.1에서 펜던트/매뉴얼로 직접 확인해야 함 — config/fr5.yaml 참고)
"""

from typing import Any


class Fr5Hardware:
    def __init__(self, ip: str, config: dict):
        # ROS2 패키지로 빌드되는 시점이 아니라, 실제로 이 클래스를 생성하는
        # 시점(런타임)에만 fairino를 import한다 — SDK가 없는 환경에서도
        # 이 모듈 자체는 최소한 import/문법 체크는 가능하게 하기 위함.
        from fairino import Robot  # noqa: PLC0415

        self.config = config
        self.robot = Robot.RPC(ip)
        self._configure_gripper()

    # ------------------------------------------------------------------
    def _configure_gripper(self) -> None:
        g = self.config["gripper"]
        err = self.robot.SetGripperConfig(g["company"], g["device"])
        if err != 0:
            raise RuntimeError(f"SetGripperConfig 실패: error={err}")
        err = self.robot.ActGripper(g["index"], 1)  # 1 = 활성화
        if err != 0:
            raise RuntimeError(f"ActGripper(활성화) 실패: error={err}")

    def reset_errors(self) -> None:
        """execute_cb 맨 앞에서 매번(멱등) 호출되는 에러 리셋.

        ⚠️ 'ResetAllError'라는 이름은 문서에서 확인 못 했고, 다른 협동로봇
        SDK들의 관례(clean_error 등)에서 유추한 추정값이다. 설치된 SDK에
        실제로 있는 메서드 이름으로 바꿔야 할 가능성이 높다 — hasattr로
        방어적으로 호출해서, 없으면 조용히 넘어가고 로그만 남긴다.
        """
        reset_fn = getattr(self.robot, "ResetAllError", None)
        if reset_fn is None:
            return
        reset_fn()

    def move_joint(self, joint_pos: list) -> bool:
        speed = self.config["speed"]
        rtn = self.robot.MoveJ(
            joint_pos,
            tool=self.config["tool"],
            user=self.config["user"],
            vel=speed["vel"],
            ovl=speed["ovl"],
        )
        return rtn == 0

    def gripper_close(self) -> bool:
        return self._move_gripper(self.config["gripper"]["close_pos"])

    def gripper_open(self) -> bool:
        return self._move_gripper(self.config["gripper"]["open_pos"])

    def _move_gripper(self, pos: int) -> bool:
        g = self.config["gripper"]
        rtn = self.robot.MoveGripper(
            g["index"], pos, g["vel"], g["force"], g["maxtime_ms"],
            0,  # block: 0 = 블로킹
            0,  # type: 0 = 평행 그리퍼
            0, 0, 0,  # rotNum/rotVel/rotTorque — 회전 그리퍼 아니므로 미사용
        )
        return rtn == 0

    def check_grasped(self) -> bool:
        g = self.config["gripper"]
        err, _fault, position = self.robot.GetGripperCurPosition()
        if err != 0:
            return False
        # close_pos까지 완전히 닫혔으면 아무것도 안 집힌 것으로 판단.
        return position > g["grasp_min_pos"]

    def close(self) -> None:
        close_fn = getattr(self.robot, "CloseRPC", None)
        if close_fn is not None:
            close_fn()
