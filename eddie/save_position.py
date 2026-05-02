import os
import threading
import time
import numpy as np
import rby1_sdk as rby

ROBOT_ADDRESS = "192.168.30.1:50051"
POSITIONS_DIR = os.path.join(os.path.dirname(__file__), "positions")

# ── 시나리오 & 슬롯 정의 ──────────────────────────────────────

SCENARIOS = {
    "1": {
        "name": "드릴프레스 작업",
        "steps": ["1-1", "1-2", "1-3", "1-4", "1-5", "1-6", "1-7", "1-8", "1-9"],
    },
    "2": {
        "name": "경신박스픽업",
        "steps": ["2-1", "2-2", "2-3", "2-4", "2-5", "2-6"],
    },
}

POSITION_SLOTS = {
    # ── 시나리오 1: 드릴프레스 작업 ──
    "1-1": ("s1_move_start_to_pickup",    "이동: 시작위치 → 픽업테이블"),
    "1-2": ("s1_left_pickup_approach",    "왼팔: 픽업 접근"),
    "1-3": ("s1_left_pickup_grasp",       "왼팔: 물건 파지 (그리퍼 닫힘)"),
    "1-4": ("s1_move_pickup_to_drill",    "이동: 픽업테이블 → 드릴프레스"),
    "1-5": ("s1_left_drill_place",        "왼팔: 드릴프레스 배치 (그리퍼 열림)"),
    "1-6": ("s1_right_drill_approach",    "오른팔: 드릴 접근"),
    "1-7": ("s1_right_drill_grasp",       "오른팔: 드릴 파지"),
    "1-8": ("s1_right_drill_pull",        "오른팔: 드릴 당김 (궤적)"),
    "1-9": ("s1_right_drill_push",        "오른팔: 드릴 밀기 (역궤적, 자동)"),

    # ── 시나리오 2: 경신박스픽업 ──
    "2-1": ("s2_move_to_box",             "이동: 박스 위치로 이동"),
    "2-2": ("s2_box_approach",            "양팔: 박스 접근"),
    "2-3": ("s2_box_grasp",               "양팔: 박스 파지 (그리퍼 닫힘)"),
    "2-4": ("s2_box_lift",                "양팔: 박스 들기"),
    "2-5": ("s2_box_place",               "양팔: 박스 내려놓기"),
    "2-6": ("s2_box_release",             "양팔: 박스 놓기 (그리퍼 열림)"),
}

MOBILE_SLOTS = {"1-1", "1-4", "2-1"}
TRAJECTORY_SLOTS = {"1-8"}
AUTO_REVERSE_MAP = {"1-9": "1-8"}
GRIPPER_ACTIONS = {
    "1-3": "close_left",
    "1-5": "open_left",
    "2-3": "close_both",
    "2-6": "open_both",
}
RECORD_HZ = 10
MIN_CHANGE = 0.005

robot = None


# ── 연결 & 상태 ───────────────────────────────────────────────

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


# ── 녹화 ──────────────────────────────────────────────────────

def _record_trajectory():
    """마스터암 조작 중 관절 궤적을 녹화."""
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
    input("  [녹화 중] 마스터암으로 동작 후 엔터로 종료... ")
    running[0] = False
    t.join(timeout=1.0)
    return np.array(frames)


# ── 저장 ──────────────────────────────────────────────────────

def _slot_type_tag(key):
    if key in MOBILE_SLOTS:
        return " [모바일]"
    if key in TRAJECTORY_SLOTS:
        return " [궤적]"
    if key in AUTO_REVERSE_MAP:
        return " [자동생성]"
    return ""


def _save_slot(slot_key):
    """슬롯 하나를 저장. 저장 완료 후 슬롯 선택으로 복귀."""
    os.makedirs(POSITIONS_DIR, exist_ok=True)
    name, desc = POSITION_SLOTS[slot_key]

    # ── 모바일 슬롯 ──
    if slot_key in MOBILE_SLOTS:
        import mobile as mb
        steps = mb.record_mobile()
        if steps is None:
            print("취소")
            return
        save_path = os.path.join(POSITIONS_DIR, name + ".npy")
        np.save(save_path, steps)
        print(f"저장 완료: {save_path}  [{len(steps)} 스텝]")
        return

    # ── 자동 역순 생성 ──
    if slot_key in AUTO_REVERSE_MAP:
        src_key = AUTO_REVERSE_MAP[slot_key]
        src_name = POSITION_SLOTS[src_key][0]
        src_path = os.path.join(POSITIONS_DIR, src_name + ".npy")
        if not os.path.exists(src_path):
            src_step = src_key.split("-")[1]
            print(f"{src_name} 이 먼저 저장돼야 합니다 ({src_step}번 선택)")
            return
        traj = np.load(src_path)
        if traj.ndim != 2:
            print(f"{src_name} 이 궤적 형식이 아닙니다. 다시 저장해주세요.")
            return
        reversed_traj = traj[::-1]
        save_path = os.path.join(POSITIONS_DIR, name + ".npy")
        np.save(save_path, reversed_traj)
        print(f"저장 완료 ({src_name} 역순): {save_path}  [{len(reversed_traj)} 프레임]")
        return

    # ── 궤적 녹화 ──
    if slot_key in TRAJECTORY_SLOTS:
        print(f"  대상: {name}  ({desc})")
        input("  엔터를 누르면 녹화 시작... ")
        traj = _record_trajectory()
        if len(traj) < 2:
            print("녹화된 프레임이 너무 적습니다. 다시 시도해주세요.")
            return
        save_path = os.path.join(POSITIONS_DIR, name + ".npy")
        np.save(save_path, traj)
        print(f"저장 완료: {save_path}  [{len(traj)} 프레임 @ {RECORD_HZ}Hz]")

        # 역순 자동 생성
        for rev_key, src_key in AUTO_REVERSE_MAP.items():
            if src_key == slot_key:
                rev_name, rev_desc = POSITION_SLOTS[rev_key]
                rev_path = os.path.join(POSITIONS_DIR, rev_name + ".npy")
                np.save(rev_path, traj[::-1])
                print(f"자동 생성: {rev_path}  ({rev_desc})")
        return

    # ── 단일 위치 스냅샷 (전체 관절) ──
    state = robot.get_state()
    pos = np.array(state.position)
    save_path = os.path.join(POSITIONS_DIR, name + ".npy")
    np.save(save_path, pos)
    np.set_printoptions(precision=3, suppress=True)
    print(f"저장 완료: {save_path}")
    print(f"  위치 (rad): {pos}")


def save_position():
    if robot is None:
        print("먼저 연결하세요 (1번)")
        return

    while True:
        # 1단계: 시나리오 선택
        print("\n--- 저장할 시나리오 선택 ---")
        for sc_key, sc in SCENARIOS.items():
            print(f"  {sc_key}: {sc['name']}")
        print("  q: 돌아가기")

        sc_choice = input("시나리오 선택: ").strip()
        if sc_choice == "q":
            return
        if sc_choice not in SCENARIOS:
            print("없는 번호예요")
            continue

        sc = SCENARIOS[sc_choice]

        # 2단계: 슬롯 선택 (루프)
        while True:
            print(f"\n--- [{sc['name']}] 저장할 위치 선택 ---")
            for key in sc["steps"]:
                name, desc = POSITION_SLOTS[key]
                path = os.path.join(POSITIONS_DIR, name + ".npy")
                exists = "\u2713" if os.path.exists(path) else " "
                step_num = key.split("-")[1]
                print(f"  {step_num}: [{exists}] {name}  ({desc}){_slot_type_tag(key)}")
            print("  q: 돌아가기")

            step_choice = input("선택: ").strip()
            if step_choice == "q":
                break
            slot_key = f"{sc_choice}-{step_choice}"
            if slot_key not in POSITION_SLOTS:
                print("없는 번호예요")
                continue

            _save_slot(slot_key)
