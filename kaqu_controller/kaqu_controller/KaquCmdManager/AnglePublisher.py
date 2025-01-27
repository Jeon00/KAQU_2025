### Written by TEAM4 "박준형", "진미경", "이나영"

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.KaquCmdManager.RobotManagerNode import RobotManager
from kaqu_controller.KaquCmdManager.KaquParams import RobotState
from std_msgs.msg import Float64MultiArray

class QuadrupedControllerNode(Node):
    def __init__(self):
        super().__init__('quadruped_controller_node')

        body = [100., 100.]  # bodyLength, bodyWidth
        legs = [130., 36., 130., 36.]  # l1, l2, l3, l4
        USE_IMU = True

        self.inverse_kinematics = InverseKinematics(body, legs)
        self.kaquctrl = RobotManager(body, legs, USE_IMU)
        self.state = RobotState(0.5)

        self.foot_subscriber = self.create_subscription(Float64MultiArray, '/legpo', self.sub_call, 10)

        self.joint_publisher = self.create_publisher(
            JointState,
            '/Kaqu_Joint',
            10
        )

        self.joint_names = [
            "fr_mainbodyhip_joint", "fr_hip1_joint", "fr_14_joint",
            "fl_mainbodyhip_joint", "fl_hip1_joint", "fl_14_joint",
            "rr_mainbodyhip_joint", "rr_hip1_joint", "rr_14_joint",
            "rl_mainbodyhip_joint", "rl_hip1_joint", "rl_14_joint",
        ]

        # 주기적으로 main_control 호출
        self.timer = self.create_timer(0.1, self.main_control)  # 0.1초 간격
        self.leg_pos = None

    def sub_call(self, msg):
        self.leg_pos = msg.data

    def main_control(self):

        self.get_logger().info(f"[DEBUG] Current RobotManager state: {self.kaquctrl.__dict__}")

        # Run controller
        leg_position = np.array(self.leg_pos).reshape((3,4))
        
        # Debug log for returned leg positions
        self.get_logger().info(f"Returned leg position from run(): {leg_position}")
        
        # Check if body state is updated
        self.get_logger().info(f"Body position: {self.state.body_local_position}")
        self.get_logger().info(f"Body orientation: {self.state.body_local_orientation}")

        # Call gait changer
        self.kaquctrl.gait_changer()

        # Debug state after gait changer
        self.get_logger().info(f"Updated leg position: {leg_position}")
        
        # Compute inverse kinematics
        dx, dy, dz = self.state.body_local_position
        roll, pitch, yaw = self.state.body_local_orientation

        try:
            pub_angles = self.inverse_kinematics.inverse_kinematics(
                leg_position, dx, dy, dz, roll, pitch, yaw
            )
            self.get_logger().info(f"Calculated joint angles: {pub_angles}")

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = pub_angles

            self.joint_publisher.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"IK calculation failed: {e}")
    

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()