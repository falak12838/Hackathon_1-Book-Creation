# ROS 2 Agent Example
# This file would contain the actual Python code for the agent
# For documentation purposes, we'll create a conceptual example

"""
Agent Example for Humanoid Robotics

This example demonstrates an agent node that receives commands and controls a humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class HumanoidAgent(Node):
    def __init__(self):
        super().__init__('humanoid_agent')

        # Command subscriber - receives high-level commands
        self.command_sub = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        # Velocity publisher - sends movement commands
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Joint state subscriber - monitors robot state
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Humanoid Agent initialized')

    def command_callback(self, msg):
        """Process high-level commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Process command and generate appropriate response
        if command == 'move_forward':
            self.move_forward()
        elif command == 'stop':
            self.stop_robot()

    def joint_state_callback(self, msg):
        """Monitor joint states"""
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')

    def move_forward(self):
        """Send forward movement command"""
        twist = Twist()
        twist.linear.x = 0.5  # Move forward at 0.5 m/s
        self.vel_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    agent = HumanoidAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()