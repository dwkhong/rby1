import sys
import save_position as sp
import master_arm as ma
import move_position as mp
import gripper as gr

MENU = {
    "1": ("RPC 연결",        sp.connect),
    "2": ("현재 위치 가져오기", sp.get_position),
    "3": ("마스터암 연결",     ma.connect),
    "4": ("마스터암 해제",     ma.disconnect),
    "5": ("현재 위치 저장",    sp.save_position),
    "6": ("저장된 위치 확인",   mp.show_positions),
    "7": ("저장 위치로 이동",   mp.move_to),
    "8": ("그리퍼 연결/호밍",   gr.connect),
    "9": ("그리퍼 수동 제어",   gr.manual_control),
    "q": ("종료",            None),
}


def main():
    while True:
        print("\n--- eddie main ---")
        for key, (desc, _) in MENU.items():
            print(f"  {key}: {desc}")
        choice = input("선택: ").strip()

        if choice == "q":
            print("종료")
            sys.exit(0)
        elif choice in MENU:
            MENU[choice][1]()
        else:
            print("없는 번호예요")


if __name__ == "__main__":
    main()
