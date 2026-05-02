import time
import threading
import numpy as np
import rby1_sdk as rby
import save_position as sp

# ID 0 = 오른쪽, ID 1 = 왼쪽 (하드웨어에 따라 바뀔 수 있음)
GRIPPER_IDS = [0, 1]
GRIPPER_DIRECTION = True  # True: 트리거 0=min_q(열림), 1=max_q(닫힘)

_gripper = None  # 모듈 싱글톤


class _Gripper:
    def __init__(self):
        self.bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self.bus.open_port()
        self.bus.set_baud_rate(2_000_000)
        self.bus.set_torque_constant([1, 1])
        self.min_q = np.array([np.inf, np.inf])
        self.max_q = np.array([-np.inf, -np.inf])
        self.target_q = None
        self._running = False
        self._thread = None

    def initialize(self):
        ok = True
        for dev_id in GRIPPER_IDS:
            if not self.bus.ping(dev_id):
                print(f"  그리퍼 ID {dev_id} 응답 없음")
                ok = False
            else:
                print(f"  그리퍼 ID {dev_id} 연결됨")
        if ok:
            self.bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in GRIPPER_IDS])
        return ok

    def _set_mode(self, mode):
        self.bus.group_sync_write_torque_enable([(dev_id, 0) for dev_id in GRIPPER_IDS])
        self.bus.group_sync_write_operating_mode([(dev_id, mode) for dev_id in GRIPPER_IDS])
        self.bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in GRIPPER_IDS])

    def homing(self):
        print("  호밍 중... (끝까지 열고 닫힘)")
        self._set_mode(rby.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0, 0], dtype=np.float64)
        prev_q = np.array([0, 0], dtype=np.float64)
        counter = 0
        while direction < 2:
            torque = 0.5 * (1 if direction == 0 else -1)
            self.bus.group_sync_write_send_torque([(dev_id, torque) for dev_id in GRIPPER_IDS])
            rv = self.bus.group_fast_sync_read_encoder(GRIPPER_IDS)
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self.min_q = np.minimum(self.min_q, q)
            self.max_q = np.maximum(self.max_q, q)
            if np.array_equal(prev_q, q):
                counter += 1
            prev_q = q.copy()
            if counter >= 30:
                direction += 1
                counter = 0
            time.sleep(0.1)

        print(f"  호밍 완료  min={self.min_q}  max={self.max_q}")

        # 호밍 후 열린 위치로 이동 (GRIPPER_DIRECTION=True → 열림=min_q)
        open_q = self.min_q if GRIPPER_DIRECTION else self.max_q
        self._set_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque([(dev_id, 5) for dev_id in GRIPPER_IDS])
        self.bus.group_sync_write_send_position(
            [(dev_id, float(open_q[dev_id])) for dev_id in GRIPPER_IDS]
        )
        self.target_q = open_q.copy()
        time.sleep(1.0)
        print("  그리퍼 열림")

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _loop(self):
        self._set_mode(rby.DynamixelBus.CurrentBasedPositionControlMode)
        self.bus.group_sync_write_send_torque([(dev_id, 5) for dev_id in GRIPPER_IDS])
        while self._running:
            if self.target_q is not None:
                self.bus.group_sync_write_send_position(
                    [(dev_id, q) for dev_id, q in enumerate(self.target_q.tolist())]
                )
            time.sleep(0.05)

    def set_normalized(self, right: float, left: float):
        """0.0 = 열림, 1.0 = 닫힘"""
        if not np.isfinite(self.min_q).all() or not np.isfinite(self.max_q).all():
            return
        vals = np.array([right, left])
        if GRIPPER_DIRECTION:
            self.target_q = vals * (self.max_q - self.min_q) + self.min_q
        else:
            self.target_q = (1 - vals) * (self.max_q - self.min_q) + self.min_q


# ── 공개 인터페이스 ─────────────────────────────────────────


def connect():
    global _gripper
    if _gripper is not None:
        print("그리퍼가 이미 연결되어 있습니다")
        return True

    robot = sp.robot
    if robot is None:
        print("먼저 RPC 연결하세요 (1번)")
        return False

    # tool flange 12V 출력 → 그리퍼 모터 전원 공급
    for arm in ["right", "left"]:
        if not robot.set_tool_flange_output_voltage(arm, 12):
            print(f"tool flange 전압 설정 실패 ({arm})")
            return False
    time.sleep(0.5)  # 모터 기동 대기

    g = _Gripper()
    if not g.initialize():
        print("그리퍼 초기화 실패")
        return False
    g.homing()
    g.start()
    _gripper = g
    print("그리퍼 연결 완료")
    return True


def disconnect():
    global _gripper
    if _gripper is None:
        return
    _gripper.stop()
    _gripper = None


def set_normalized(right: float, left: float):
    """내부용: 0.0=열림, 1.0=닫힘"""
    if _gripper is not None:
        _gripper.set_normalized(right, left)


def set_percent(right_pct: float, left_pct: float):
    """사용자용: 0=완전닫힘, 100=완전열림"""
    set_normalized(1.0 - right_pct / 100.0, 1.0 - left_pct / 100.0)


def manual_control():
    if _gripper is None:
        print("그리퍼가 연결되어 있지 않습니다. 먼저 연결해주세요.")
        return

    TARGETS = {
        "1": "오른쪽",
        "2": "왼쪽",
        "3": "양쪽",
    }

    print("\n--- 그리퍼 수동 제어 ---")
    print("  0 = 완전 닫힘  /  100 = 완전 열림")
    for key, label in TARGETS.items():
        print(f"  {key}: {label}")
    print("  q: 취소")

    choice = input("선택: ").strip()
    if choice == "q":
        return
    if choice not in TARGETS:
        print("없는 번호예요")
        return

    try:
        pct = float(input("퍼센트 (0~100): ").strip())
    except ValueError:
        print("숫자를 입력해주세요")
        return

    pct = max(0.0, min(100.0, pct))

    if choice == "1":
        set_percent(pct, _cur_left_pct())
    elif choice == "2":
        set_percent(_cur_right_pct(), pct)
    elif choice == "3":
        set_percent(pct, pct)

    print(f"{TARGETS[choice]} → {pct:.0f}%")


def _normalized(idx: int) -> float:
    if _gripper is None or _gripper.target_q is None:
        return 0.0
    rng = _gripper.max_q[idx] - _gripper.min_q[idx]
    if rng == 0:
        return 0.0
    n = (_gripper.target_q[idx] - _gripper.min_q[idx]) / rng
    return float(n if GRIPPER_DIRECTION else 1 - n)


def _cur_right_pct() -> float:
    return (1.0 - _normalized(0)) * 100.0


def _cur_left_pct() -> float:
    return (1.0 - _normalized(1)) * 100.0


def _cur_right():
    return _normalized(0)


def _cur_left():
    return _normalized(1)
