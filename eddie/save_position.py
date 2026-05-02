import os
import threading
import time
import numpy as np
import rby1_sdk as rby

ROBOT_ADDRESS = "192.168.30.1:50051"
POSITIONS_DIR = os.path.join(os.path.dirname(__file__), "positions")

POSITION_SLOTS = {
    "1": ("pos_01_home",          "Home 복귀 / 시작 위치"),
    "2": ("pos_02_approach_pre",  "접근 1단계 - 안전 경유 위치"),
    "3": ("pos_03_approach_final","접근 2단계 - 파지 직전 위치"),
    "4": ("pos_04_grasp",         "파지 - 그리퍼 닫힘 위치"),
    "5": ("pos_05_pull",          "당김 - 드릴 하강 / 원호 (궤적 녹화)"),
    "6": ("pos_06_push",          "밀기 - 드릴 상승 / 원호 (pos_05 역순 자동생성)"),
    "7": ("pos_07_depart_pre",    "복귀 1단계"),
    "8": ("pos_08_depart_final",  "복귀 2단계"),
}

TRAJECTORY_SLOTS = {"5"}   # 직접 녹화
AUTO_REVERSE_SLOTS = {"6"}  # pos_05 역순 자동생성
RECORD_HZ = 10              # 녹화 샘플링 주파수

robot = None


def connect():
    global robot
    robot = rby.create_robot(ROBOT_ADDRESS, "a")
    if not robot.connect():
        print("RPC 연결 실패")
        robot = None
        return
    if not robot.is_connected():
        print("RPC 연결 실패")
        robot = None
        return
    print(f"RPC 연결 성공! ({ROBOT_ADDRESS})")


def get_position():
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return
    state = robot.get_state()
    with rby.printoptions(linewidth=10**9, multiline_repr=False):
        np.set_printoptions(precision=3, suppress=True)
        print(f"관절 위치 (rad): {np.array(state.position)}")


MIN_CHANGE = 0.005  # 관절 변화 최소값 (rad), 이 미만이면 중복으로 간주


def _record_trajectory():
    frames = []
    running = [True]

    def _sample():
        interval = 1.0 / RECORD_HZ
        last = None
        while running[0]:
            state = robot.get_state()
            pos = np.array(state.position)
            if last is None or np.max(np.abs(pos - last)) >= MIN_CHANGE:
                frames.append(pos)
                last = pos
            time.sleep(interval)

    t = threading.Thread(target=_sample, daemon=True)
    t.start()
    input("  [녹화 중] 마스터암으로 원호 동작 후 엔터로 종료... ")
    running[0] = False
    t.join(timeout=1.0)
    return np.array(frames)


def save_position():
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    print("\n--- 저장할 위치 선택 ---")
    for key, (name, desc) in POSITION_SLOTS.items():
        path = os.path.join(POSITIONS_DIR, name + ".npy")
        exists = "✓" if os.path.exists(path) else " "
        print(f"  {key}: [{exists}] {name}  ({desc})")
    print("  q: 취소")

    choice = input("선택: ").strip()
    if choice == "q":
        print("취소")
        return
    if choice not in POSITION_SLOTS:
        print("없는 번호예요")
        return

    os.makedirs(POSITIONS_DIR, exist_ok=True)
    name, desc = POSITION_SLOTS[choice]

    # pos_06: pos_05 역순 자동생성
    if choice in AUTO_REVERSE_SLOTS:
        src_path = os.path.join(POSITIONS_DIR, "pos_05_pull.npy")
        if not os.path.exists(src_path):
            print("pos_05_pull 이 먼저 저장돼야 합니다 (5번 선택)")
            return
        traj = np.load(src_path)
        if traj.ndim != 2:
            print("pos_05_pull 이 궤적 형식이 아닙니다. 5번으로 다시 저장해주세요.")
            return
        reversed_traj = traj[::-1]
        save_path = os.path.join(POSITIONS_DIR, name + ".npy")
        np.save(save_path, reversed_traj)
        print(f"저장 완료 (pos_05 역순): {save_path}  [{len(reversed_traj)} 프레임]")
        return

    # pos_05: 궤적 녹화 → 저장 후 pos_06도 자동 생성
    if choice in TRAJECTORY_SLOTS:
        print(f"  대상: {name}  ({desc})")
        input("  엔터를 누르면 녹화 시작... ")
        traj = _record_trajectory()
        if len(traj) < 2:
            print("녹화된 프레임이 너무 적습니다. 다시 시도해주세요.")
            return
        save_path = os.path.join(POSITIONS_DIR, name + ".npy")
        np.save(save_path, traj)
        print(f"저장 완료: {save_path}  [{len(traj)} 프레임 @ {RECORD_HZ}Hz]")

        # pos_06 자동 생성
        rev_name, rev_desc = POSITION_SLOTS["6"]
        rev_path = os.path.join(POSITIONS_DIR, rev_name + ".npy")
        np.save(rev_path, traj[::-1])
        print(f"자동 생성: {rev_path}  ({rev_desc})")
        return

    # 나머지: 단일 위치 스냅샷
    state = robot.get_state()
    pos = np.array(state.position)
    save_path = os.path.join(POSITIONS_DIR, name + ".npy")
    np.save(save_path, pos)
    np.set_printoptions(precision=3, suppress=True)
    print(f"저장 완료: {save_path}")
    print(f"  위치 (rad): {pos}")
