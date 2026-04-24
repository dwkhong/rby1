import sys
import rby1_sdk as rby

ROBOT_ADDRESS = "192.168.0.3:50051"

robot = rby.create_robot(ROBOT_ADDRESS, "a")

if not robot.connect():
    print("연결 실패")
    sys.exit(1)

print("연결 성공!")

info = robot.get_robot_info()
print(f"로봇 정보: {info}")

state = robot.get_state()
print(f"관절 위치: {state.position}")
