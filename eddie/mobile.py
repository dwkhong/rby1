import numpy as np
import rby1_sdk as rby
import save_position as sp

DEFAULT_LINEAR_SPEED = 0.2    # m/s
DEFAULT_ANGULAR_SPEED = 0.3   # rad/s
CONTROL_HOLD_TIME = 1.0


def record_mobile():
    """모바일 이동 시퀀스를 입력받아 numpy 배열로 반환. 취소 시 None."""
    steps = []
    print("\n--- 모바일 이동 시퀀스 입력 ---")
    print("  방향: f(전진) b(후진) l(좌) r(우) cw(시계회전) ccw(반시계회전)")
    print("  입력: 방향 시간(초) [속도(m/s 또는 rad/s)]")
    print("  예: f 3.0       → 전진 3초 (기본 0.2m/s)")
    print("  예: cw 2.0 0.5  → 시계회전 2초 (0.5rad/s)")
    print("  d: 입력 완료  q: 취소")

    while True:
        line = input(f"  스텝 {len(steps)+1}: ").strip()
        if line == "q":
            return None
        if line == "d":
            break

        parts = line.split()
        if len(parts) < 2:
            print("    방향과 시간을 입력하세요 (예: f 3.0)")
            continue

        direction = parts[0].lower()
        try:
            duration = float(parts[1])
        except ValueError:
            print("    시간은 숫자로 입력하세요")
            continue

        speed = DEFAULT_LINEAR_SPEED
        ang_speed = DEFAULT_ANGULAR_SPEED
        if len(parts) >= 3:
            try:
                val = float(parts[2])
                speed = val
                ang_speed = val
            except ValueError:
                pass

        vx, vy, omega = 0.0, 0.0, 0.0
        if direction == "f":
            vx = speed
        elif direction == "b":
            vx = -speed
        elif direction == "l":
            vy = speed
        elif direction == "r":
            vy = -speed
        elif direction == "cw":
            omega = -ang_speed
        elif direction == "ccw":
            omega = ang_speed
        else:
            print(f"    알 수 없는 방향: {direction}")
            continue

        steps.append([vx, vy, omega, duration])
        print(f"    → vx={vx:.2f} vy={vy:.2f} ω={omega:.2f} t={duration:.1f}s")

    if not steps:
        print("  입력된 스텝이 없습니다")
        return None

    return np.array(steps)


def play_mobile(steps):
    """저장된 모바일 이동 시퀀스를 재생."""
    robot = sp.robot
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return False

    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        print("Control Manager 활성화 실패")
        return False

    print(f"  모바일 이동: {len(steps)} 스텝")
    for i, step in enumerate(steps):
        vx, vy, omega, duration = step[0], step[1], step[2], step[3]
        print(f"  스텝 {i+1}/{len(steps)}: vx={vx:.2f} vy={vy:.2f} ω={omega:.2f} t={duration:.1f}s")

        cmd = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_mobility_command(
                rby.SE2VelocityCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(CONTROL_HOLD_TIME)
                )
                .set_minimum_time(duration)
                .set_velocity([vx, vy], omega)
            )
        )
        robot.send_command(cmd, duration + 5.0).get()

    # 정지 명령
    stop_cmd = rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_mobility_command(
            rby.SE2VelocityCommandBuilder()
            .set_command_header(
                rby.CommandHeaderBuilder().set_control_hold_time(CONTROL_HOLD_TIME)
            )
            .set_minimum_time(0.5)
            .set_velocity([0, 0], 0)
        )
    )
    robot.send_command(stop_cmd, 3.0).get()

    print("  이동 완료")
    return True


def mobile_move():
    """메뉴 7번: 모바일 베이스 수동 이동 (저장 없이 즉시 실행)."""
    robot = sp.robot
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    print("\n--- 모바일 베이스 수동 이동 ---")
    print("  방향: f(전진) b(후진) l(좌) r(우) cw(시계회전) ccw(반시계회전)")
    print("  입력: 방향 시간(초) [속도(m/s 또는 rad/s)]")
    print("  q: 종료")

    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        print("Control Manager 활성화 실패")
        return

    while True:
        line = input("이동: ").strip()
        if line == "q":
            return

        parts = line.split()
        if len(parts) < 2:
            print("  방향과 시간을 입력하세요 (예: f 3.0)")
            continue

        direction = parts[0].lower()
        try:
            duration = float(parts[1])
        except ValueError:
            print("  시간은 숫자로 입력하세요")
            continue

        speed = DEFAULT_LINEAR_SPEED
        ang_speed = DEFAULT_ANGULAR_SPEED
        if len(parts) >= 3:
            try:
                val = float(parts[2])
                speed = val
                ang_speed = val
            except ValueError:
                pass

        vx, vy, omega = 0.0, 0.0, 0.0
        if direction == "f":
            vx = speed
        elif direction == "b":
            vx = -speed
        elif direction == "l":
            vy = speed
        elif direction == "r":
            vy = -speed
        elif direction == "cw":
            omega = -ang_speed
        elif direction == "ccw":
            omega = ang_speed
        else:
            print(f"  알 수 없는 방향: {direction}")
            continue

        confirm = input(f"  vx={vx:.2f} vy={vy:.2f} ω={omega:.2f} t={duration:.1f}s 실행? (y/n): ").strip()
        if confirm != "y":
            continue

        cmd = rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_mobility_command(
                rby.SE2VelocityCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(CONTROL_HOLD_TIME)
                )
                .set_minimum_time(duration)
                .set_velocity([vx, vy], omega)
            )
        )
        print("  이동 중...")
        robot.send_command(cmd, duration + 5.0).get()
        print("  완료")
