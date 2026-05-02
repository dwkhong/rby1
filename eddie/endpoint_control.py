import numpy as np
import rby1_sdk as rby
import save_position as sp

LINEAR_VELOCITY_LIMIT = 0.5   # m/s
ANGULAR_VELOCITY_LIMIT = np.pi  # rad/s
ACCELERATION_LIMIT = 1.0       # m/s²
STOP_POSITION_ERROR = 1e-3     # m
STOP_ORIENTATION_ERROR = 1e-4  # rad
MOVE_TIME = 3.0                # s


def _euler_to_R(rx_deg, ry_deg, rz_deg):
    """ZYX 오일러각 (도) → 3x3 회전행렬 (R = Rz * Ry * Rx)."""
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx


def _quat_to_R(qx, qy, qz, qw):
    """쿼터니언 → 3x3 회전행렬."""
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [  2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
        [  2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])


def _R_to_euler(R):
    """회전행렬 → ZYX 오일러각 (도) 변환 (현재 포즈 표시용)."""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        rx = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    else:
        rx = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        ry = np.degrees(np.arctan2(-R[2, 0], sy))
        rz = 0.0
    return rx, ry, rz


def _input_pose(label):
    """팔 하나의 목표 포즈 입력. 4x4 변환행렬 반환, 취소 시 None."""
    print(f"\n[{label}] 목표 위치 (base 프레임 기준, 단위: m)")
    try:
        parts = input("  x y z: ").strip().split()
        if len(parts) != 3:
            print("  x y z 세 값을 공백으로 구분해 입력하세요")
            return None
        x, y, z = map(float, parts)
    except ValueError:
        print("  숫자를 입력하세요")
        return None

    print(f"[{label}] 목표 자세 입력 방식")
    print("  e: 오일러각  rx ry rz (도, ZYX 순서)")
    print("  q: 쿼터니언  qx qy qz qw")
    print("  i: 단위행렬  (회전 없음)")
    mode = input("  선택 (e/q/i): ").strip().lower()

    try:
        if mode == "i":
            R = np.eye(3)
        elif mode == "e":
            vals = input("  rx ry rz (도): ").strip().split()
            if len(vals) != 3:
                print("  rx ry rz 세 값이 필요합니다")
                return None
            R = _euler_to_R(*map(float, vals))
        elif mode == "q":
            vals = input("  qx qy qz qw: ").strip().split()
            if len(vals) != 4:
                print("  qx qy qz qw 네 값이 필요합니다")
                return None
            R = _quat_to_R(*map(float, vals))
        else:
            print("  없는 선택이에요")
            return None
    except ValueError:
        print("  숫자를 입력하세요")
        return None

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    rx_d, ry_d, rz_d = _R_to_euler(R)
    print(f"  → 위치: x={x:.4f}  y={y:.4f}  z={z:.4f} m")
    print(f"  → 자세: rx={rx_d:.2f}°  ry={ry_d:.2f}°  rz={rz_d:.2f}°")
    return T


def _cartesian_cmd(arm, T_right=None, T_left=None):
    def _arm_cmd(T, ee):
        return (
            rby.CartesianCommandBuilder()
            .add_target("base", ee, T,
                        LINEAR_VELOCITY_LIMIT,
                        ANGULAR_VELOCITY_LIMIT,
                        ACCELERATION_LIMIT)
            .set_minimum_time(MOVE_TIME)
            .set_stop_position_tracking_error(STOP_POSITION_ERROR)
            .set_stop_orientation_tracking_error(STOP_ORIENTATION_ERROR)
        )

    body = rby.BodyComponentBasedCommandBuilder()
    if arm in ("r", "b"):
        body.set_right_arm_command(_arm_cmd(T_right, "ee_right"))
    if arm in ("l", "b"):
        body.set_left_arm_command(_arm_cmd(T_left, "ee_left"))

    return rby.RobotCommandBuilder().set_command(
        rby.ComponentBasedCommandBuilder().set_body_command(body)
    )


def _show_current_ee(robot):
    """현재 엔드이펙터 위치 출력 (tool_flange 이용)."""
    try:
        state = robot.get_state()
        tf_r = state.tool_flange_right
        tf_l = state.tool_flange_left
        np.set_printoptions(precision=4, suppress=True)
        print("  [현재 오른팔 ee]:", np.array(tf_r.transformation)[:3, 3])
        print("  [현재 왼팔  ee]:", np.array(tf_l.transformation)[:3, 3])
    except Exception:
        pass


def endpoint_control():
    robot = sp.robot
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    while True:
        print("\n--- 엔드포인트 Cartesian 제어 ---")
        _show_current_ee(robot)
        print("  r: 오른팔")
        print("  l: 왼팔")
        print("  b: 양팔")
        print("  q: 종료")
        arm = input("선택: ").strip().lower()

        if arm == "q":
            return
        if arm not in ("r", "l", "b"):
            print("없는 선택이에요")
            continue

        T_right = T_left = None

        if arm in ("r", "b"):
            T_right = _input_pose("오른팔")
            if T_right is None:
                continue

        if arm in ("l", "b"):
            T_left = _input_pose("왼팔")
            if T_left is None:
                continue

        confirm = input("\n이동하시겠습니까? (y/n): ").strip().lower()
        if confirm != "y":
            print("취소")
            continue

        robot.reset_fault_control_manager()
        if not robot.enable_control_manager():
            print("Control Manager 활성화 실패")
            continue

        print("이동 중...")
        rv = robot.send_command(_cartesian_cmd(arm, T_right, T_left), 10).get()
        if rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok:
            print("이동 완료")
        else:
            print(f"이동 실패 (코드: {rv.finish_code})")
