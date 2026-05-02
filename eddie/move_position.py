import os
import time
import numpy as np
import rby1_sdk as rby
import save_position as sp
import mobile as mb
import gripper as gr

MOVE_TIME = 5.0
TRAJ_STEP_TIME = 1.0 / sp.RECORD_HZ


def show_positions():
    print("\n--- 저장된 위치 목록 ---")
    any_saved = False
    for sc_key, sc in sp.SCENARIOS.items():
        sc_has = False
        for key in sc["steps"]:
            name, desc = sp.POSITION_SLOTS[key]
            path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
            if os.path.exists(path):
                if not sc_has:
                    print(f"\n  [{sc['name']}]")
                    sc_has = True
                pos = np.load(path)
                np.set_printoptions(precision=3, suppress=True)
                if key in sp.MOBILE_SLOTS:
                    print(f"    {key}: {name}  ({desc})  → 모바일 {len(pos)} 스텝")
                elif pos.ndim == 2:
                    print(f"    {key}: {name}  ({desc})  → 궤적 {len(pos)} 프레임")
                else:
                    print(f"    {key}: {name}  ({desc})")
                    print(f"         {pos}")
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
    """단일 위치로 이동 (전체 관절 명령)."""
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
    rc.set_left_arm_command(
        rby.JointPositionCommandBuilder()
        .set_position(pos[model.left_arm_idx])
        .set_minimum_time(MOVE_TIME)
    )

    cmd = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(rc)
    )
    handler = robot.send_command(cmd)
    handler.get()
    return True


def _move_trajectory(robot, traj):
    """궤적 재생 (전체 관절 스트리밍)."""
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
        rc.set_left_arm_command(
            rby.JointPositionCommandBuilder()
            .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(TRAJ_STEP_TIME * 2))
            .set_position(pos[model.left_arm_idx])
            .set_minimum_time(TRAJ_STEP_TIME)
        )

        stream.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(rc)
            )
        )
        time.sleep(TRAJ_STEP_TIME)

    print("  재생 완료")


def _do_gripper_action(action):
    """그리퍼 액션 수행."""
    if action == "close_left":
        print("  왼쪽 그리퍼 닫힘...")
        gr.set_percent(gr._cur_right_pct(), 0)
    elif action == "open_left":
        print("  왼쪽 그리퍼 열림...")
        gr.set_percent(gr._cur_right_pct(), 100)
    elif action == "close_both":
        print("  양쪽 그리퍼 닫힘...")
        gr.set_percent(0, 0)
    elif action == "open_both":
        print("  양쪽 그리퍼 열림...")
        gr.set_percent(100, 100)
    time.sleep(1.0)
    print("  그리퍼 동작 완료")


def move_to():
    robot = sp.robot
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    while True:
        available = {
            key: (name, desc)
            for key, (name, desc) in sp.POSITION_SLOTS.items()
            if os.path.exists(os.path.join(sp.POSITIONS_DIR, name + ".npy"))
        }

        if not available:
            print("저장된 위치가 없습니다. 먼저 5번으로 저장하세요.")
            return

        print("\n--- 이동할 위치 선택 ---")
        for key in sorted(available.keys()):
            name, desc = available[key]
            path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
            pos = np.load(path)
            if key in sp.MOBILE_SLOTS:
                tag = f"모바일 {len(pos)}스텝"
            elif pos.ndim == 2:
                tag = f"궤적 {len(pos)}프레임"
            else:
                tag = "단일 위치"
            gripper_tag = ""
            if key in sp.GRIPPER_ACTIONS:
                action = sp.GRIPPER_ACTIONS[key]
                if "close" in action:
                    gripper_tag = " +그리퍼닫힘"
                else:
                    gripper_tag = " +그리퍼열림"
            print(f"  {key}: {name}  ({desc})  [{tag}]{gripper_tag}")
        print("  q: 돌아가기")

        choice = input("선택: ").strip()
        if choice == "q":
            return
        if choice not in available:
            print("없는 번호예요")
            continue

        name, desc = available[choice]
        path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
        pos = np.load(path)

        print(f"{desc} 실행 중...")

        # 모바일 이동
        if choice in sp.MOBILE_SLOTS:
            mb.play_mobile(pos)
            continue

        # 궤적 재생
        if pos.ndim == 2:
            _move_trajectory(robot, pos)
        else:
            ok = _move_single(robot, pos)
            if ok:
                print("이동 완료")

        # 그리퍼 액션
        if choice in sp.GRIPPER_ACTIONS:
            _do_gripper_action(sp.GRIPPER_ACTIONS[choice])


def _execute_step(robot, key):
    """슬롯 하나를 실행. 성공 시 True, 실패 시 False."""
    name, desc = sp.POSITION_SLOTS[key]
    path = os.path.join(sp.POSITIONS_DIR, name + ".npy")

    if not os.path.exists(path):
        print(f"  \u2717 [{key}] {desc} \u2014 저장된 데이터 없음, 건너뜀")
        return False

    pos = np.load(path)
    print(f"\n  \u25b6 [{key}] {desc}")

    # 모바일 이동
    if key in sp.MOBILE_SLOTS:
        mb.play_mobile(pos)
    # 궤적 재생
    elif pos.ndim == 2:
        _move_trajectory(robot, pos)
    # 단일 위치
    else:
        _move_single(robot, pos)

    # 그리퍼 액션
    if key in sp.GRIPPER_ACTIONS:
        _do_gripper_action(sp.GRIPPER_ACTIONS[key])

    print(f"  \u2713 [{key}] 완료")
    return True


def run_scenario():
    robot = sp.robot
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    print("\n--- 시나리오 선택 ---")
    for key, sc in sp.SCENARIOS.items():
        print(f"  {key}: {sc['name']}")
    print("  q: 취소")

    choice = input("선택: ").strip()
    if choice == "q":
        print("취소")
        return
    if choice not in sp.SCENARIOS:
        print("없는 번호예요")
        return

    sc = sp.SCENARIOS[choice]
    steps = sc["steps"]

    # 저장 안 된 슬롯 체크
    missing = []
    for key in steps:
        name, desc = sp.POSITION_SLOTS[key]
        path = os.path.join(sp.POSITIONS_DIR, name + ".npy")
        if not os.path.exists(path):
            missing.append(f"  {key}: {name}  ({desc})")

    if missing:
        print("\n\u26a0 저장되지 않은 슬롯:")
        for m in missing:
            print(m)
        confirm = input("저장 안 된 슬롯은 건너뛰고 진행할까요? (y/n): ").strip()
        if confirm != "y":
            print("취소")
            return

    print(f"\n{'='*40}")
    print(f"  시나리오: {sc['name']}")
    print(f"  총 {len(steps)} 스텝")
    print(f"{'='*40}")

    for i, key in enumerate(steps):
        name, desc = sp.POSITION_SLOTS[key]
        print(f"\n--- [{i+1}/{len(steps)}] {desc} ---")
        _execute_step(robot, key)

    print(f"\n{'='*40}")
    print(f"  시나리오 완료: {sc['name']}")
    print(f"{'='*40}")
