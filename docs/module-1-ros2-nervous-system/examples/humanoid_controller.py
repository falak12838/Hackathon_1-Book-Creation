# ROS 2 Controller Example
# This file would contain the actual Python code for the controller
# For documentation purposes, we'll create a conceptual example

"""
Controller Example for Humanoid Robotics

This example demonstrates a controller node that translates high-level commands
into low-level joint commands for a humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Velocity command subscriber
        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        # Joint trajectory publisher
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            'joint_trajectory',
            10
        )

        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Current joint positions
        self.current_joints = JointState()

        self.get_logger().info('Humanoid Controller initialized')

    def velocity_callback(self, msg):
        """Convert velocity commands to joint movements"""
        self.get_logger().info(f'Received velocity: linear={msg.linear.x}, angular={msg.angular.z}')

        # Calculate joint trajectories based on velocity commands
        trajectory = self.calculate_joint_trajectory(msg)
        self.joint_pub.publish(trajectory)

    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joints = msg

    def calculate_joint_trajectory(self, twist_cmd):
        """Calculate joint trajectory based on twist command"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        point = JointTrajectoryPoint()

        # Simplified calculation - in reality this would involve inverse kinematics
        if twist_cmd.linear.x > 0:
            # Moving forward - adjust leg joints accordingly
            point.positions = [0.1, -0.2, 0.1, 0.1, -0.2, 0.1]
        elif twist_cmd.linear.x < 0:
            # Moving backward
            point.positions = [-0.1, 0.2, -0.1, -0.1, 0.2, -0.1]
        else:
            # Stopped - return to neutral position
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)
        return trajectory


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()