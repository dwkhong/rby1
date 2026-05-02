import os
import sys
import numpy as np
import rby1_sdk as rby
import save_position as sp
import gripper as gr

MODEL_PATH = "/home/nvidia/rby1-sdk/models/master_arm/model.urdf"

MA_MIN_Q = np.deg2rad([-360, -30,  0, -135, -90,  35, -360,  -360,  10,  -90, -135, -90,  35, -360])
MA_MAX_Q = np.deg2rad([ 360, -10, 90,  -60,  90,  80,  360,   360,  30,    0,  -60,  90,  80,  360])
MA_TORQUE_LIMIT = np.array([3.5, 3.5, 3.5, 1.5, 1.5, 1.5, 1.5] * 2)
MA_VISCOUS_GAIN = np.array([0.02, 0.02, 0.02, 0.02, 0.01, 0.01, 0.002] * 2)
MA_Q_LIMIT_BARRIER = 0.5

LOOP_PERIOD = 0.01
IMPEDANCE_STIFFNESS = 50
IMPEDANCE_DAMPING = 1.0
IMPEDANCE_TORQUE_LIMIT = 30.0

master_arm = None
_stream = None


def connect():
    global master_arm, _stream

    robot = sp.robot
    if robot is None:
        print("먼저 RPC 연결하세요 (1번)")
        return

    model = robot.model()
    dyn_model = robot.get_dynamics()
    dyn_state = dyn_model.make_state([], model.robot_joint_names)
    robot_max_q = dyn_model.get_limit_q_upper(dyn_state)
    robot_min_q = dyn_model.get_limit_q_lower(dyn_state)
    robot_max_qdot = dyn_model.get_limit_qdot_upper(dyn_state)
    robot_max_qddot = dyn_model.get_limit_qddot_upper(dyn_state)

    robot_q = [None]

    def robot_state_cb(state):
        robot_q[0] = np.array(state.position)

    robot.start_state_update(robot_state_cb, 1 / LOOP_PERIOD)

    robot.reset_fault_control_manager()
    if not robot.enable_control_manager():
        print("Control manager 활성화 실패")
        robot.stop_state_update()
        return

    robot.set_parameter("joint_position_command.cutoff_frequency", "3")

    _stream = robot.create_command_stream(priority=1)

    # 초기 커맨드: 현재 위치 유지 (스트림에 양팔 등록)
    if robot_q[0] is not None:
        init_right = robot_q[0][model.right_arm_idx]
        init_left  = robot_q[0][model.left_arm_idx]
    else:
        init_right = np.zeros(7)
        init_left  = np.zeros(7)

    _stream.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(
                rby.BodyComponentBasedCommandBuilder()
                .set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1e6))
                    .set_position(init_right)
                    .set_minimum_time(1.0)
                )
                .set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1e6))
                    .set_position(init_left)
                    .set_minimum_time(1.0)
                )
            )
        )
    )

    rby.upc.initialize_device(rby.upc.MasterArmDeviceName)
    master_arm = rby.upc.MasterArm(rby.upc.MasterArmDeviceName)
    master_arm.set_model_path(MODEL_PATH)
    master_arm.set_control_period(LOOP_PERIOD)

    active_ids = master_arm.initialize(verbose=True)
    if len(active_ids) != rby.upc.MasterArm.DeviceCount:
        print(f"마스터암 초기화 실패 (감지된 장치 수: {len(active_ids)})")
        master_arm = None
        return

    right_q = [None]
    left_q = [None]
    right_min_t = [1.0]
    left_min_t = [1.0]

    # 트리거 필터 상태
    TRIGGER_ALPHA = 0.1   # EMA 계수 (작을수록 더 부드럽게)
    trig_r = [0.0]
    trig_l = [0.0]

    def control(state: rby.upc.MasterArm.State):
        if right_q[0] is None:
            right_q[0] = state.q_joint[0:7].copy()
        if left_q[0] is None:
            left_q[0] = state.q_joint[7:14].copy()

        # 마스터암 토크 계산 (중력보상 + 관절 한계 + 점성)
        torque = (
            state.gravity_term
            + MA_Q_LIMIT_BARRIER * (
                np.maximum(MA_MIN_Q - state.q_joint, 0)
                + np.minimum(MA_MAX_Q - state.q_joint, 0)
            )
            + MA_VISCOUS_GAIN * state.qvel_joint
        )
        torque = np.clip(torque, -MA_TORQUE_LIMIT, MA_TORQUE_LIMIT)

        ma_input = rby.upc.MasterArm.ControlInput()

        # 항상 CurrentControlMode + 중력보상 (떨림 방지)
        ma_input.target_operating_mode.fill(rby.DynamixelBus.CurrentControlMode)
        ma_input.target_torque = torque * 0.6

        # 버튼 누를 때 추종할 위치 업데이트
        if state.button_right.button == 1:
            right_q[0] = state.q_joint[0:7].copy()
        if state.button_left.button == 1:
            left_q[0] = state.q_joint[7:14].copy()

        # 트리거 EMA 필터 적용 후 deadband 통과 시에만 그리퍼 업데이트
        raw_r = state.button_right.trigger / 1000.0
        raw_l = state.button_left.trigger / 1000.0
        new_r = TRIGGER_ALPHA * raw_r + (1 - TRIGGER_ALPHA) * trig_r[0]
        new_l = TRIGGER_ALPHA * raw_l + (1 - TRIGGER_ALPHA) * trig_l[0]
        gr.set_normalized(new_r, new_l)
        trig_r[0] = new_r
        trig_l[0] = new_l

        # 충돌 감지
        if robot_q[0] is not None:
            q = robot_q[0].copy()
            q[model.right_arm_idx] = right_q[0]
            q[model.left_arm_idx] = left_q[0]
            dyn_state.set_q(q)
            dyn_model.compute_forward_kinematics(dyn_state)
            is_collision = (
                dyn_model.detect_collisions_or_nearest_links(dyn_state, 1)[0].distance < 0.02
            )
        else:
            is_collision = False

        # 로봇 명령 스트리밍 (버튼 눌렸을 때만)
        rc = rby.BodyComponentBasedCommandBuilder()
        has_command = False

        if state.button_right.button and not is_collision:
            right_min_t[0] = max(right_min_t[0] - LOOP_PERIOD, LOOP_PERIOD * 1.01)
            rc.set_right_arm_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1e6))
                .set_position(np.clip(right_q[0], robot_min_q[model.right_arm_idx], robot_max_q[model.right_arm_idx]))
                .set_velocity_limit(robot_max_qdot[model.right_arm_idx])
                .set_acceleration_limit(robot_max_qddot[model.right_arm_idx] * 30)
                .set_minimum_time(right_min_t[0])
            )
            has_command = True
        else:
            right_min_t[0] = 0.8

        if state.button_left.button and not is_collision:
            left_min_t[0] = max(left_min_t[0] - LOOP_PERIOD, LOOP_PERIOD * 1.01)
            rc.set_left_arm_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(1e6))
                .set_position(np.clip(left_q[0], robot_min_q[model.left_arm_idx], robot_max_q[model.left_arm_idx]))
                .set_velocity_limit(robot_max_qdot[model.left_arm_idx])
                .set_acceleration_limit(robot_max_qddot[model.left_arm_idx] * 30)
                .set_minimum_time(left_min_t[0])
            )
            has_command = True
        else:
            left_min_t[0] = 0.8

        if has_command:
            _stream.send_command(
                rby.RobotCommandBuilder().set_command(
                    rby.ComponentBasedCommandBuilder().set_body_command(rc)
                )
            )

        return ma_input

    master_arm.start_control(control)

    # C++ 컨트롤 루프가 fd 1에 avg: ... 을 직접 출력하므로
    # fd 1 → /dev/null 으로 리다이렉트하고 Python stdout만 진짜 터미널에 유지
    _real_stdout_fd = os.dup(1)
    devnull = os.open("/dev/null", os.O_WRONLY)
    os.dup2(devnull, 1)
    os.close(devnull)
    sys.stdout = os.fdopen(_real_stdout_fd, "w", buffering=1)

    print("마스터암 연결 완료 — 버튼 누르면 로봇 팔 추종")


def disconnect():
    global master_arm, _stream

    if master_arm is None:
        print("마스터암이 연결되어 있지 않습니다")
        return

    robot = sp.robot
    master_arm.stop_control()
    master_arm = None
    _stream = None

    if robot is not None:
        robot.stop_state_update()
        robot.cancel_control()
        robot.disable_control_manager()

    print("마스터암 연결 해제 완료")
