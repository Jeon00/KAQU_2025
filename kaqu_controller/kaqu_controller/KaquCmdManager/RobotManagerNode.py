### Written by TEAM4 "박준형", "정현호", "소재정"

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Joy
from kaqu_controller.KaquCmdManager.KaquParams import RobotCommand, RobotState, BehaviorState
from kaqu_controller.Kaquctrl.TrotGaitController import TrotGaitController
from kaqu_controller.Kaquctrl.RestController import RestController
from std_msgs.msg import Float64MultiArray


class StartController:
    def run(self):
        # Simulate the start logic
        return "Starting"
    
    def updateStateCommand():
        pass


class RobotManager(Node):
    def __init__(self, body, legs, imu):
        super().__init__('robot_manager')

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joystick_callback, 10
        )

        self.angle_publisher = self.create_publisher(Float64MultiArray, '/legpo', 10)

        # 기본 로봇 파라미터 설정
        self.body = body
        self.legs = legs

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.001  # 상수
        self.x_shift_back = -0.001  # 상수
        self.default_height = 0.5

        # 상태 초기화
        self.state = RobotState(self.default_height)
        self.command = RobotCommand(self.default_height)
        self.state.foot_location = self.default_stance()

        # Gait 컨트롤러 초기화
        self.trot_controller = TrotGaitController(self.default_stance(), stance_time = 0.18, swing_time = 0.24, time_step = 0.02, use_imu = imu)
        self.rest_controller = RestController(self.default_stance())
        self.start_controller = StartController()

        # 기본 컨트롤러 설정 (Rest 상태)
        self.current_controller = self.rest_controller
        self.state.behavior_state = BehaviorState.REST



    def default_stance(self):
        """기본 자세를 정의합니다 (4개의 발 위치)."""
        return np.array([
            [self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front,
             -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [0, 0, 0, 0]
        ])

    def joystick_callback(self, msg):
        """조이스틱 입력에 따라 이벤트를 설정."""
        if msg.buttons[0]:  # Start 버튼
            self.command.start_event = True
            self.command.trot_event = False
            self.command.rest_event = False
        elif msg.buttons[1]:  # Trot 버튼
            self.command.start_event = False
            self.command.trot_event = True
            self.command.rest_event = False
        elif msg.buttons[2]:  # Rest 버튼
            self.command.start_event = False
            self.command.trot_event = False
            self.command.rest_event = True

        if self.current_controller == self.rest_controller:
            self.current_controller.updateStateCommand(msg, self.state)
        elif self.current_controller == self.trot_controller:
            self.current_controller.updateStateCommand(msg, self.command)
        elif self.current_controller == self.start_controller:
            self.current_controller.updateStateCommand()

    def gait_changer(self):
        """명령에 따라 행동 상태와 컨트롤러를 변경."""
        if self.command.start_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.START
                self.current_controller = self.start_controller
                self.get_logger().info("Switched to Start Controller")
            self.command.start_event = False

        elif self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.current_controller = self.trot_controller
                self.current_controller.pid_controller.reset()
                self.state.ticks = 0
                self.get_logger().info("Switched to Trot Controller")
            self.command.trot_event = False

        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.current_controller = self.rest_controller
            self.current_controller.pid_controller.reset()
            self.get_logger().info("Switched to Rest Controller")
            self.command.rest_event = False

        print(f"Behavior State: {self.state.behavior_state}, Current Controller: {self.current_controller}")

    def imu_orientation(self, msg):
        quaternion = [msg.axes[0], msg.axes[1], msg.axes[7], 1]
        rotation = R.from_quat(quaternion)
        rpy = rotation.as_euler('xyz', degrees=True)  # false 하면 라디안
        self.state.imu_roll = rpy[0]
        self.state.imu_pitch = rpy[1]

    def run(self):
        # """현재 활성화된 컨트롤러 실행."""
        # if self.current_controller:
        return self.current_controller.run(self.state, self.command)
        # else:
        #     self.get_logger().warn("No active controller set!")
        #     return "No active controller"    

    def publish_angle(self, result):
        """run() 메서드의 결과값을 퍼블리시."""
        try:
            # 결과를 Float64로 변환해 퍼블리시
            angle_msg = Float64MultiArray()
            angle_msg.data = result.flatten().tolist()
            self.angle_publisher.publish(angle_msg)
            self.get_logger().info(f"Published angle: {angle_msg.data}")
        except ValueError:
            self.get_logger().warn(f"Invalid angle result: {result}")
    


def main(args=None):
    rclpy.init(args=args)

    # Body와 Legs의 초기값 설정
    body_dimensions = [100., 100.]  # [length, width]
    leg_dimensions = [130., 36., 130., 36.]  # [length, offset]
    imu = True

    # RobotManager 초기화
    robot_manager = RobotManager(body_dimensions, leg_dimensions, imu)

    try:
        while rclpy.ok():
            # ROS 이벤트 처리
            rclpy.spin_once(robot_manager, timeout_sec=0.1)

            # 상태 전환 처리
            robot_manager.gait_changer()

            # 활성화된 컨트롤러 실행
            result = robot_manager.run()
            robot_manager.get_logger().info(f"Controller result: {result}")
            robot_manager.publish_angle(result)

    except KeyboardInterrupt:
        robot_manager.get_logger().info("Shutting down RobotManager.")

    # 종료 처리
    robot_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()