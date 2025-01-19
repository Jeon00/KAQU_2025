#!/usr/bin/env python
# -*- coding: utf-8 -*-

# bulk_read_write

# 이 파일을 Practice 브랜치에 작업하시거나, 
# Practice 브랜치에서 분기하셔서 본인 이름으로 branch 만들고 Practice를 pull 하시면서 작업해주시면 감사하겠습니다. 
# Team2 브랜치에는 확정된 내용들만 merge하도록 하겠습니다. 

# 개념을 요약하자면 다음과 같습니다. 
# 기존의 Team2 브랜치는 팀의 최종 결정 후에 push하기
# 이 브랜치(Practice)에 작업하기
# 혹은 이 브랜치(Practice)에서 갈라지는 하나의 브랜치를 하나 더 만들어(ex : Youngjin) 
# 거기서 작업 후 push -> 이후 merge할 때는 팀 내의 상의/혹은 팀장의 확인을 받으시면 됩니다. 

# 바로 merge해도 잘 할 수 있다 싶으신 분들은 Practice에서, 
# 약간 쫄린다 싶으시면 본인 branch 만들어서 작업하시고 Practice로 merge하기 전 팀장에게 말씀해주시면 됩니다. 

# 실제로 어떻게 하느냐? 두 가지 경우로 나눠보면, 1. Practice에서 작업하는 경우, 2. 본인 브랜치를 만들어 작업하시는 경우
# 1. Practice에서 작업하는 경우 
# 0) 원격의 Practice를 pull
# 1) 로컬의 Practice 브랜치에서 시작 
# 2) 원격 저장소의 Practice 브랜치를 pull(merge) 
# 3) 작업하기
# 4) 커밋 후 Practice 브랜치에 푸쉬(git push origin Practice)

# 2. 본인 브랜치를 만들어 작업하시는 경우  
# 0) Practice 브랜치를 pull, 여기서 갈라지는 본인 이름 브랜치 생성
# 1) 로컬의 [본인 이름] 브랜치에서 시작 
# 2) 원격 저장소의 Practice 브랜치를 pull(merge) : 본인 이름 브랜치는 아무도 업데이트하지 않으므로
# 3) 작업하기 
# 4) [본인 이름] 브랜치에 push

# 업무 분담
# 1. 하드웨어 통신 관련 : 강동륜, 박정환
# 1월 2주
# - SDK 관련 내용 파악
# - 시리얼 통신 관련 용어 파악
# - dynamixel 데이터 패킷에 데이터 넣는 법 파악
# - 샘플 코드(이 파일) 참고, 빠르면 개발시작
# 1월 3주 
# - 개발시작 : 걍 이 파일의 내용이 돌아가게만 하면 됨
# - openCR에 핀 꼽아서 쓰는 센서값을 어떻게 다루면 되는지 확인하고 usb_to_dxl 코드(openCR 보드에 업로드되는거) 수정
# - 시뮬에서 나오는 내용이 그대로 반영되는지 확인(더미 코드 작성 필요)

# 2. ROS2 통신 관련 : 류희창, 장현호, 정인아
# 1월 2주
# - 이 코드 내에 Bulk_Read_Write 라는 이름의 노드 작성
# - 아래 코드는 while 문으로 작동하고 있는데, 이를 ROS2 노드로 만들고 spin으로 돌아가게 해야 함. 
# --- 즉, 아래 코드를 쭉 보고 어디부터 어디까지를 함수로 둘지, 어떤 것을 콜백으로 만들지 결정해야 함. 
# --- 받는거 : 무릎 모터 기준의 다리 각도 제어값(A1)
# --- 보내는거 : 무릎 모터 기준의 다리 각도 실제값(A2), 바디 좌표계로 변환된 센서값(B)
# 1월 3주
# - 코드 돌아가는 속도 파악, openCR 통신 속도 맞춰보고 Hz 맞추기(더미 코드 작성 필요)
# - 토픽 이름, 메시지 타입 등 다른 팀과 맞춰야 하는 부분 맞추기

# 3. 하드웨어 변환 관련 : 강동륜, 박정환, 정인아
# 1월 2주
# - 필요 변수 파악, 일단 그 변수의 값을 이 코드에 상수로 박아두기
# - 모터 위치변환+각도단위변환 수식 만들기
# - 센서값이 온다 치면 그 센서의 위치(처음에는 그냥 [0, 0, 0]으로 설정)에 따라 보정하는 수식 만들기
# 1월 3주
# - 시뮬에서 잘 돌아가는지 파악(더미 코드 작성 필요)
# - 센서의 실제 위치를 고려, 바디 좌표계로 변환된 센서값(B)만들기

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # team4 QuadrupedControllerNode 노드 참조
from sensor_msgs.msg import Imu 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # team4 QuadrupedControllerNode 노드 참조
from sensor_msgs.msg import Imu 

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

#os.sys.path.append('./../../DynamixelSDK/python/src/dynamixel_sdk/dynamixel_functions_py')             # Path setting
from dynamixel_sdk import *  #다이나믹셀 라이브러리 사용

MY_DXL = 'X_SERIES'  

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_LED_RED                = 65
    LEN_LED_RED                 = 1         # Data Byte Length
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Default setting
FR1_ID                     = 11                             # Dynamixel ID: 1
FR2_ID                     = 12                             # Dynamixel ID: 2
FR3_ID                     = 13
FL1_ID                     = 21
FL2_ID                     = 22
FL3_ID                     = 23
RR1_ID                     = 31
RR2_ID                     = 32
RR3_ID                     = 33
RL1_ID                     = 41
RL2_ID                     = 42
RL3_ID                     = 43

DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

# 아래 내용은 protocol 1에만 있음
# ESC_ASCII_VALUE             = 0x1b
# COMM_SUCCESS                = 0                             # Communication Success result value
# COMM_TX_FAIL                = -1001                         # Communication Tx Failed
# dxl_comm_result = COMM_TX_FAIL                              # Communication result
# dxl_addparam_result = 0                                     # AddParam result
# dxl_getdata_result = 0                                      # GetParam result
# dxl_error = 0                                               # Dynamixel error
# dxl1_present_position = 0                                   # Present position
# dxl2_led_value_read = 0                                     # Dynamixel moving status


# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
flag = 0                          # Threshold 반복문용 변수
dxl_goal_position = [0]*12        # 다이나믹셀 각도로 변환된 Goal position 넣을 곳
dxl_present_position = [0]*12     # 모터에서 Present Position 값 받아올 곳
dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write
dxl_id = [FR1_ID, FR2_ID, FR3_ID, FL1_ID, FL2_ID, FL3_ID, RR1_ID, RR2_ID, RR3_ID, RL1_ID, RL2_ID, RL3_ID]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    print("Press any key to terminate...")
    getch()
    quit()

# 각 모터 토크 켜기
for i in dxl_id:
    dxl_comm_result, dxl_error= packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % i)

# present position에 대한 parameter 저장소 추가
for i in dxl_id:
    dxl_addparam_result = groupBulkRead.addParam(i, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % i)
        quit()

# 여기까지 초기 세팅
# 아래의 while문 내부 내용을 msg 콜백으로 묶어서 써도 될듯
# 하드웨어, 변환 담당 조는 ROS조가 노드를 완성하기 전까지 callback 함수 형태로 만들어 두기

# 값을 보내고 받는 함수들, 이걸 callback으로 하면 될듯.
# while 1:
def timer_callback(self): # (25.01.17) callback_bulkReadWrite -> timer_callback으로 수정 // msg->self로 수정함
                          # timer_period 1/50-1/80 sec로 생각 중 -> 소프2팀이슈 HRI 아래 링크 참고

    # print("Press any key to continue! (or press ESC to quit!)")
    # if getch() == chr(0x1b):
    #     break

    # 214 ~ 230 일단 주석 처리
    # for i in range(len(dxl_goal_position)):
    #     #메시지에서 Goal Position 받아옴
    #     dxl_goal_position[i] = msg.goal_position[i]
    #     #Goal Position 값을 byte단위의 배열로 쪼갬
    #     param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
    #     #Goal position을 BulkWrite parameter 저장소에 추가
    #     dxl_addparam_result = groupBulkWrite.addParam(dxl_id[i], ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
    #     if dxl_addparam_result != True:
    #         print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id[i])
    #         #quit()
    #         return
    
    #     # Bulkwrite Goal Position
    #     dxl_comm_result = groupBulkWrite.txPacket()
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))


    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    #while 1: (25.01.17) timer_callback함수 안에 들어갈 내용이라 while문을 제거함
    # Present Position값 Bulkread
    dxl_comm_result = groupBulkRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for i in range(len(dxl_id)):
        # Bulkread한 데이터가 사용 가능한지 확인
        dxl_getdata_result = groupBulkRead.isAvailable(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % dxl_id[i])
            #quit()
            return

        # Present Position값 가져오기
        dxl_present_position[i] = groupBulkRead.getData(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            
        # Present Position 출력
        print("[ID:%03d] Present Position : %d" % (dxl_id[i], dxl_present_position[i]))
        
    # 현재 모터값과 목표 모터값의 차이가 Threshold보다 크면(모든 모터가 목표 모터값에 도달하면) 반복문을 빠져나감
    flag = 0
    for i in range(len(dxl_present_position)) :
        if abs(dxl_goal_position[i] - dxl_present_position[i]) > DXL_MOVING_STATUS_THRESHOLD:
            flag == 1
            break
    if flag == 0 :
        break

# Bulk_Read_Write 노드
class Bulk_Read_Write(Node):
    def __init__(self):
        super().__init__('bulk_read_write')
        # 여 

        self.dxl_goal_position = dxl_goal_position
        self.dxl_id = dxl_id
        self.groupBulkWrite = groupBulkWrite
        self.groupBulkRead = groupBulkRead

        # 다리 각도 제어값(A1)
        self.control_subscriber = self.create_subscription(
            Float64, 'control_leg_angle', self.control_callback, 10)
        
        # 다리 각도 실제값 (A2), 센서값 (B)
        self.real_angle_publisher = self.create_publisher(
            Float64, 'real_leg_angle', 10)
        self.sensor_data_publisher = self.create_publisher(
            Imu, 'sensor_data', 10)

        self.create_timer(0.1, self.publish_data)


    # 다리 각도 제어값(A1) -> goal position
    def control_callback(self, msg):
        control_angle = msg.data
        self.set_goal_positions(control_angle)

    def set_goal_positions(self, angle):
        for i, motor_id in enumerate(self.dxl_id):
            self.dxl_goal_position[i] = int((angle / 360.0) * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE))
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position[i])),
                DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position[i])),
                DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position[i])),
                DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position[i]))
            ]


    # 다리 각도 실제값 (A2), 센서값 (B) 발행
    def publish_data(self):
        # 모터 현재 위치 읽기
        self.groupBulkRead.txRxPacket()
        total_position = 0
        valid_count = 0

        for motor_id in self.dxl_id:
            if self.groupBulkRead.isAvailable(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                present_position = self.groupBulkRead.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                total_position += present_position
                valid_count += 1
                self.get_logger().info(f"Motor {motor_id} position: {present_position}")
        
        # A2, B 발행 (여기 다시 수정해야함) ---------
        if valid_count > 0: # 정상적으로 모터를 읽어온 경우
            average_position = total_position / valid_count
            angle_msg = Float64()
            angle_msg.data = (average_position / DXL_MAXIMUM_POSITION_VALUE) * 360.0
            self.real_angle_publisher.publish(angle_msg)

        sensor_msg = Imu()
        self.sensor_data_publisher.publish(sensor_msg)
        # ------------------------------------------



def main(args=None):
    rclpy.init(args=args)
    node = Bulk_Read_Write()

    try:
        node.spin_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Bulk_Read_Write Node...")
    finally:
        node.portHandler.closePort()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()

# Disable Dynamixel Torque
for i in dxl_id:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
