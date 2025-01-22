#!/usr/bin/env python
# -*- coding: utf-8 -*-

# bulk_read_write

# TODO list
# 1. USB 시리얼 번호 찾기

# Tuning
# 움직이기 위한 최소 각도 차이 : DXL_MOVING_STATUS_THRESHOLD
# IK 풀때 에러 : IK_ERROR_RANGE
# timer period

import os
import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, tan, atan2, acos, sqrt, pi
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu 

# 이부분 지피티에게 물어보니 OS가 뭔지 판단하는 부분이라고 함
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

IK_ERROR_RANGE = 0.1

# 주의 : 포지션 모드 확장 모드로 변경 필요

dxl_led_value = [0x00, 0x01]                                                        # Dynamixel LED value for write
dxl_id = [FR1_ID, FR2_ID, FR3_ID, FL1_ID, FL2_ID, FL3_ID, RR1_ID, RR2_ID, RR3_ID, RL1_ID, RL2_ID, RL3_ID]

# 계산에 필요한 하드웨어 스펙
# 이 부분을 코드에 박아둘까요 말까요
l1 = 130.0
l2 = 36.0
l3 = 130.0
l4a = 36.0
lhip = 31.5

# 주의
motor_direction = [0]*12


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
# 주의 : 여기에 현재 위치 읽고 거기를 초기값으로 세팅해야 함.
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
        self.portHandler = portHandler
        self.packetHandler = packetHandler
        self.groupBulkWrite = groupBulkWrite
        self.groupBulkRead = groupBulkRead

        self.last_command = [0]*12 #무릎
        self.goal_position = [0]*12 #엉덩이
        self.last_read_joint_dxl = [0]*12 #현재
        self.last_read_sensor = [0]*3 # 일단 뭐가 될지 모르겠는데 배열 형태로 보내면 어떨까

        # 주의 : openCR 연결해보고 timer period 조절해야 함
        data_pub_period = 0.5
        control_period = 0.5

        # 다리 각도 제어값(엉덩이)
        # 주의 : msg타입, 토픽이름 수정해야 함. 
        self.control_subscriber = self.create_subscription(JointState, 'control_leg_angle', self.control_callback, 10)
        self.present_angle_publisher = self.create_publisher(JointState, 'real_leg_angle', 10)
        self.imu_data_publisher = self.create_publisher(Imu, 'imu_data', 10)
        
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
        goal_position = self.sim_to_real_transform(cmd_angle)
        
        # last command update(다이나믹셀 각도)
        for i in range(goal_position):
            if goal_position[i]<0:
                print("something wrong with transform")
                return
            else:
                pass
        self.last_command = goal_position

    # ROS로 다리 각도 실제값 (A2), 센서값 (B) 발행
    # 주의 : 함수 구조 살펴봐야 함. 
    def publish_data(self):

        angle_msg = JointState()
        imu_msg = Imu()

        # 읽은 값 받아오기
        real_angle = self.real_to_sim_transform(self.last_read_joint_dxl)

        # 모터값 라디안으로 변환하여 넣기
        for i in range(self.last_read_joint_dxl):
            angle_msg.position[i] = real_angle[i]*2*pi/DXL_MAXIMUM_POSITION_VALUE

        # 받아온 센서값 넣기
        
        # 퍼블리시
        self.present_angle_publisher.publish(angle_msg)
        self.imu_data_publisher.publish(imu_msg)

    def timer_callback(self):
        # 읽기 먼저
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        for i in range(len(dxl_id)):
            dxl_getdata_result = self.groupBulkRead.isAvailable(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % dxl_id[i])
                # quit()
                return
            # present pos 가져오기
            self.last_read_joint_dxl[i] = self.groupBulkRead.getData(dxl_id[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            print("[ID:%03d] Present Position : %d" % (dxl_id[i], self.last_read_joint_dxl[i]))
        
        # 쓰기 
        for i in range(len(self.goal_position)):

            # 마지막 명령값 받아오기
            self.goal_position[i] = self.last_command[i]
            # byte 단위의 배열로 쪼개기
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(self.goal_position[i])), 
                                   DXL_HIBYTE(DXL_LOWORD(self.goal_position[i])), 
                                   DXL_LOBYTE(DXL_HIWORD(self.goal_position[i])), 
                                   DXL_HIBYTE(DXL_HIWORD(self.goal_position[i]))]
            # Bulkwrite 파라미터 저장소에 추가
            dxl_addparam_result = self.groupBulkWrite.addParam(dxl_id[i], ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkWrite addparam failed" % dxl_id[i])
                return
            
            # BulkWrite Goal Position
            dxl_comm_result = self.groupBulkWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" %self.packetHandler.getTxRxResult(dxl_comm_result))
        # 파라미터 저장소 비우기
        self.groupBulkWrite.clearParam()
        
        # 주의 : 샘플 코드를 보면 while문이 2중으로 되어 있음
        # 안쪽의 while 문은 모터가 지정한 위치에 도달할 떄까지 기다렸다가 다음 커멘드를 업데이트함. 
        # 지금은 그냥 한번 딱 찍어주고 모터가 알아서 거기까지 가는 내용. 
        # 아래의 코드로 업데이트하는데, 돌려보고 이게 필요한지 확인해봐야 함. 
        # if not (abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD):
        #   break

    # 굳이 self를 넣어야 할까요?
    def sim_to_real_transform(self, cmd_angle):
        real_angle = [0]*12
        # IK 풀기
        for i in range(4):
            roll = cmd_angle[3*i]
            alpha = cmd_angle[3*i+1]
            beta1 = cmd_angle[3*i+2]

            _a = -l1*cos(alpha)-l4a*cos(beta1)
            _b = lhip-l4a*sin(beta1)-l1*sin(alpha)
            _c = (l3**2-l2**2-_a**2-_b**2)/(2*l2)

            # 판별식
            if (_b**2-_c**2+_a**2)<0:
                print("something wrong with %03d th leg IK" %(i+1))
                break
            else:
                pass

            beta2 = 2*atan2(_b-sqrt(_b**2-_c**2+_a**2)/(_a+_c))

            # 말이 되는 각도인지 확인
            if (l2*cos(beta2)-l4a*cos(beta1)-l1*cos(alpha))<-IK_ERROR_RANGE:
                beta2 = 2*atan2(_b+sqrt(_b**2-_c**2+_a**2)/(_a+_c))
                if (l2*cos(beta2)-l4a*cos(beta1)-l1*cos(alpha))<-IK_ERROR_RANGE:
                    print("something wrong with %03d th leg IK" %(i+1))
                    break
                else: 
                    pass
            else:
                pass
            # 각도 넣기(라디안->다이나믹셀 각도)
            # 주의 : 모터 설치 각도를 고려해야 함. 
            real_angle[3*i] = int(roll*DXL_MAXIMUM_POSITION_VALUE/2*pi)
            real_angle[3*i+1] = int(alpha*DXL_MAXIMUM_POSITION_VALUE/2*pi)
            real_angle[3*i+2] = int(beta2*DXL_MAXIMUM_POSITION_VALUE/2*pi)
        # 주의 : 모터 설치 각도 확인하는 부분 넣어야 함. 

        return real_angle
    
    def real_to_sim_transform(self, present_angle):
        rad_angle = [0]*12
        sim_angle = [0]*12
        # 다이나믹셀 각도 -> 라디안으로 변환
        for i in range(len(dxl_id)):
            rad_angle[i] = present_angle[i]*2*pi/DXL_MAXIMUM_POSITION_VALUE
        # 라디안 값 보정
        # dxl_id = [FR1_ID, FR2_ID, FR3_ID, FL1_ID, FL2_ID, FL3_ID, RR1_ID, RR2_ID, RR3_ID, RL1_ID, RL2_ID, RL3_ID]
        for i in range(2): # 0 : Front, 1 : Rearㄴ
            for j in range(2): # 0 : Right, 1: left
                # hip 보정
                # fr일때 -pi/4 00
                # fl일때 -3*pi/4 01
                # rr일때 -3*pi/4 10
                # rl일때 -pi/4 11
                if (i+j)==1:
                    rad_angle[6*i+3*j] = rad_angle[6*i+3*j] -3*pi/4
                else:
                    rad_angle[6*i+3*j] = rad_angle[6*i+3*j] -pi/4
                # alpha 보정
                # 주의 : 반전 어떻게 구현할지?
                # fr일때 그대로 00
                # fl일때 반전 01
                # rr일때 -pi 10
                # rl일때 -pi, 반전 11
                # beta2 보정 : alpha와 같음
                if i ==0 :
                    # 의미는 없는데 나중에 쓸일 있을까봐 넣어둠
                    rad_angle[6*i+3*j+1] = rad_angle[6*i+3*j+1]
                    rad_angle[6*i+3*j+2] = rad_angle[6*i+3*j+2]
                else:
                    rad_angle[6*i+3*j+1] = rad_angle[6*i+3*j+1] -pi
                    rad_angle[6*i+3*j+2] = rad_angle[6*i+3*j+2] -pi
        
        # IK 풀기
        # 주의 : IK 풀기 전에 모터 각도 고려해서 절대좌표계로 바꿔줘야 함
        for i in range(4):
            roll = rad_angle[3*i]
            alpha = rad_angle[3*i+1]
            beta2 = rad_angle[3*i+2]

            _a = l2*cos(beta2)-l1*cos(alpha)
            _b = lhip+l2*sin(beta2)-l1*sin(alpha)
            _c = -(l3**2-l4a**2-_a**2-_b**2)/(2*l4a)
                    # 판별식
            if (_b**2-_c**2+_a**2)<0:
                print("something wrong with %03d th leg IK" %(i+1))
                break
            else:
                pass

            beta1 = 2*atan2(_b-sqrt(_b**2-_c**2+_a**2)/(_a+_c))

            # 말이 되는 각도인지 확인
            if (l2*cos(beta2)-l4a*cos(beta1)-l1*cos(alpha))<-IK_ERROR_RANGE:
                beta2 = 2*atan2(_b+sqrt(_b**2-_c**2+_a**2)/(_a+_c))
                if (l2*cos(beta2)-l4a*cos(beta1)-l1*cos(alpha))<-IK_ERROR_RANGE:
                    print("something wrong with %03d th leg IK" %(i+1))
                    break
                else: 
                    pass
            else:
                pass
            # 각도 넣기(라디안->다이나믹셀 각도)
            # 주의 : 모터 설치 각도를 고려해야 함. 
            sim_angle[3*i] = roll
            sim_angle[3*i+1] = alpha
            sim_angle[3*i+2] = beta1
        # 주의 : 모터 설치 각도 확인하는 부분 넣어야 함. 

        return sim_angle

def main(args=None):
    rclpy.init(args=args)
    node = Bulk_Read_Write()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Bulk_Read_Write Node...")
    finally:
        node.portHandler.closePort()
        rclpy.shutdown()

        # Disable Dynamixel Torque
        for i in dxl_id:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
        # Close port
        portHandler.closePort()
    
if __name__ == '__main__':
    main()