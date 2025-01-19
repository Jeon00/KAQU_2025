#!/usr/bin/env python
# -*- coding: utf-8 -*-

# bulk_read_write

# TODO list
# 1. USB 시리얼 번호 찾기

# Tuning
# DXL_MOVING_STATUS_THRESHOLD

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # team4 QuadrupedControllerNode 노드 참조
from sensor_msgs.msg import Imu 

# 이부분 지피티에게 물어보니 입출력 형식을 담당하는 부분이라고 함
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
# 다른 변수가 필요한 경우 e-manual로부터 여기에 적어놓고 시작
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

# DYNAMIXEL Protocol Version 
PROTOCOL_VERSION            = 2.0

# Default setting
FR1_ID                     = 11                           
FR2_ID                     = 12                           
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

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
flag = 0                          # Threshold 반복문용 변수
dxl_goal_position = [0]*12        # 다이나믹셀 각도로 변환된 Goal position 넣을 곳 #주의
dxl_present_position = [0]*12     # 모터에서 Present Position 값 받아올 곳
dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write
dxl_id = [FR1_ID, FR2_ID, FR3_ID, FL1_ID, FL2_ID, FL3_ID, RR1_ID, RR2_ID, RR3_ID, RL1_ID, RL2_ID, RL3_ID]

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
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

# Bulk_Read_Write 노드
class Bulk_Read_Write(Node):
    def __init__(self):
        super().__init__('bulk_read_write')

        # 이 내용은 지속적으로 업데이트 되는 내용이라 내부변수로 뒀습니당
        # self.dxl_goal_position = dxl_goal_position
        
        self.dxl_id = dxl_id
        self.groupBulkWrite = groupBulkWrite
        self.groupBulkRead = groupBulkRead

        self.last_command = [0]*12 #무릎
        self.dxl_goal_position = [0]*12 #엉덩이
        self.last_read_joint = [0]*12 #현재
        self.last_read_sensor = [0]*2 # 일단 뭐가 될지 모르겠는데 배열 형태로 보내면 어떨까

        # 주의 : openCR 연결해보고 timer period 조절해야 함
        data_pub_period = 0.5
        control_period = 0.5

        # 다리 각도 제어값(엉덩이)
        # 주의 : msg타입, 토픽이름 수정해야 함. 
        self.control_subscriber = self.create_subscription(
            Float64, 'control_leg_angle', self.control_callback, 10)
        
        # 다리 각도 실제값 (A2), 센서값 (B)
        # 주의 : msg타입, 토픽이름 수정해야 함. 
        self.present_angle_publisher = self.create_publisher(
            Float64, 'real_leg_angle', 10)
        self.sensor_data_publisher = self.create_publisher(
            Imu, 'sensor_data', 10)
        
        # ROS로 현재 상황을 보내는 퍼블리셔
        self.pos_timer = self.create_timer(data_pub_period, self.publish_data)
        
        # 로봇으로 데이터를 보내는 퍼블리셔
        self.robot_timer = self.create_timer(control_period, self.timer_callback)


    # 다리 각도 제어값(A1) -> goal position
    def control_callback(self, msg):
        # msg 받는곳 주의
        cmd_angle = msg.position
        # cmd_vel = msg.velocity

        # Transform
        real_angle = self.sim_to_real_transform(cmd_angle)
        
        # last command update
        self.last_command = real_angle

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
    def timer_callback(self): # (25.01.17) callback_bulkReadWrite -> timer_callback으로 수정 // msg->self로 수정함
                          # timer_period 1/50-1/80 sec로 생각 중 -> 소프2팀이슈 HRI 아래 링크 참고



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
        pass

    def sim_to_real_transform(cmd_angle):
        real_angle = [0]*12
        # IK 풀기
        # 맞는지 확인
        return real_angle
    def real_to_sim_transform(present_angle):
        sim_angle = [0]*12
        # IK 풀기
        # 맞는지 확인
        return sim_angle


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
