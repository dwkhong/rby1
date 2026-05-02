import os
import time
import numpy as np
import rby1_sdk as rby
import save_position as sp

MOVE_TIME = 5.0
TRAJ_STEP_TIME = 1.0 / sp.RECORD_HZ  # 녹화 주파수와 동일하게 재생


def show_positions():
    print("\n--- 저장된 위치 목록 ---")
    any_saved = False
    for key, (name, desc) in sp.POSITION_SLOTS.items():
        path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
        if os.path.exists(path):
            pos = np.load(path)
            np.set_printoptions(precision=3, suppress=True)
            if pos.ndim == 2:
                print(f"  [{key}] {name}  ({desc})  → 궤적 {len(pos)} 프레임")
            else:
                print(f"  [{key}] {name}  ({desc})")
                print(f"       {pos}")
            any_saved = True
    if not any_saved:
        print("  저장된 위치가 없습니다.")


def _ensure_control_manager(robot):
    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        print("Control manager 활성화 실패")
        return False
    return True


def _move_single(robot, pos):
    if not _ensure_control_manager(robot):
        return False

    model = robot.model()
    rc = rby.BodyComponentBasedCommandBuilder()

    if hasattr(model, "torso_idx") and len(model.torso_idx) > 0:
        rc.set_torso_command(
            rby.JointPositionCommandBuilder()
            .set_position(pos[model.torso_idx])
            .set_minimum_time(MOVE_TIME)
        )
    rc.set_right_arm_command(
        rby.JointPositionCommandBuilder()
        .set_position(pos[model.right_arm_idx])
        .set_minimum_time(MOVE_TIME)
    )

    cmd = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(rc)
    )
    handler = robot.send_command(cmd)
    handler.get()
    return True


def _move_trajectory(robot, traj):
    if not _ensure_control_manager(robot):
        return

    model = robot.model()
    stream = robot.create_command_stream(priority=1)

    print(f"  궤적 재생: {len(traj)} 프레임")
    for i, pos in enumerate(traj):
        rc = rby.BodyComponentBasedCommandBuilder()

        if hasattr(model, "torso_idx") and len(model.torso_idx) > 0:
            rc.set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(TRAJ_STEP_TIME * 2))
                .set_position(pos[model.torso_idx])
                .set_minimum_time(TRAJ_STEP_TIME)
            )
        rc.set_right_arm_command(
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(TRAJ_STEP_TIME * 2))
            .set_position(pos[model.right_arm_idx])
            .set_minimum_time(TRAJ_STEP_TIME)
        )

        stream.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(rc)
            )
        )
        time.sleep(TRAJ_STEP_TIME)

    print("  재생 완료")


def move_to():
    robot = sp.robot
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    available = {
        key: (name, desc)
        for key, (name, desc) in sp.POSITION_SLOTS.items()
        if os.path.exists(os.path.join(sp.POSITIONS_DIR, name + ".npy"))
    }

    if not available:
        print("저장된 위치가 없습니다. 먼저 5번으로 저장하세요.")
        return

    print("\n--- 이동할 위치 선택 ---")
    for key, (name, desc) in available.items():
        path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
        pos = np.load(path)
        tag = f"궤적 {len(pos)}프레임" if pos.ndim == 2 else "단일 위치"
        print(f"  {key}: {name}  ({desc})  [{tag}]")
    print("  q: 취소")

    choice = input("선택: ").strip()
    if choice == "q":
        print("취소")
        return
    if choice not in available:
        print("없는 번호예요")
        return

    name, desc = available[choice]
    path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
    pos = np.load(path)

    print(f"{desc} 로 이동 중...")

    if pos.ndim == 2:
        _move_trajectory(robot, pos)
    else:
        ok = _move_single(robot, pos)
        if ok:
            print("이동 완료")
