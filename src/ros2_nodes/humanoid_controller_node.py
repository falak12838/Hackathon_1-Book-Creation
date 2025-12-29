#!/usr/bin/env python3

"""
Humanoid Controller Node for the Vision-Language-Action (VLA) system.
Executes action sequences on the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import json
import threading
import time
from typing import Dict, Any, Optional

# Import local modules
try:
    from vla.action_executor import ActionExecutor
except ImportError:
    # If not in the same package, try alternative import
    from src.vla.action_executor import ActionExecutor


class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller_node')

        # Initialize action executor
        self.action_executor = ActionExecutor()

        # Publishers
        self.task_status_publisher = self.create_publisher(
            String,
            'task_status',
            10
        )

        self.robot_state_publisher = self.create_publisher(
            String,
            'robot_state',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            'execution_feedback',
            10
        )

        # Subscribers
        self.action_sequence_subscriber = self.create_subscription(
            String,
            'action_sequence',
            self.action_sequence_callback,
            10
        )

        self.emergency_stop_subscriber = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # Action clients for different robot capabilities
        # These would be specific to your robot's action servers
        # self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.manipulation_client = ActionClient(self, MoveGroup, 'move_group')

        self.get_logger().info('Humanoid Controller Node initialized and ready')

    def action_sequence_callback(self, msg):
        """
        Callback for action sequences.
        """
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Received action sequence with {len(action_sequence)} actions')

            # Execute the sequence in a separate thread to avoid blocking
            execution_thread = threading.Thread(
                target=self.execute_action_sequence,
                args=(action_sequence,)
            )
            execution_thread.start()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding action sequence: {e}')
            self.publish_execution_error(f'Decoding error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in action sequence callback: {e}')
            self.publish_execution_error(f'Callback error: {str(e)}')

    def execute_action_sequence(self, action_sequence: list):
        """
        Execute a sequence of actions with monitoring and error handling.
        """
        try:
            self.get_logger().info(f'Starting execution of {len(action_sequence)} actions')

            # Publish execution start status
            self.publish_task_status({
                'status': 'executing',
                'action_count': len(action_sequence),
                'completed_count': 0
            })

            # Execute using the action executor
            execution_result = self.action_executor.execute_action_sequence(action_sequence)

            # Publish final status
            if execution_result['status'] == 'completed':
                self.get_logger().info('Action sequence completed successfully')
                self.publish_task_status({
                    'status': 'completed',
                    'action_count': len(action_sequence),
                    'completed_count': len(action_sequence),
                    'execution_time': execution_result.get('execution_time', 0)
                })
            else:
                self.get_logger().error(f'Action sequence failed: {execution_result.get("error", "Unknown error")}')
                self.publish_task_status({
                    'status': 'failed',
                    'action_count': len(action_sequence),
                    'completed_count': len(execution_result.get('completed_actions', [])),
                    'error': execution_result.get('error', 'Unknown error'),
                    'failed_action': execution_result.get('failed_action')
                })

            # Publish execution feedback
            feedback_msg = String()
            feedback_msg.data = json.dumps(execution_result)
            self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            self.get_logger().error(f'Error executing action sequence: {e}')
            self.publish_execution_error(f'Execution error: {str(e)}')

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute a single action based on its type.
        """
        try:
            # Use the action executor to handle the action
            result = self.action_executor.execute_single_action(action)
            success = result.get('success', False)

            if success:
                self.get_logger().info(f'Successfully executed action: {action.get("action", "unknown")}')
            else:
                self.get_logger().error(f'Failed to execute action: {result.get("error", "Unknown error")}')

            return success

        except Exception as e:
            self.get_logger().error(f'Error executing single action: {e}')
            return False

    def emergency_stop_callback(self, msg: Bool):
        """
        Handle emergency stop commands.
        """
        if msg.data:  # Emergency stop activated
            self.get_logger().warn('Emergency stop activated')
            self.action_executor.trigger_emergency_stop()
        else:  # Emergency stop cleared
            self.get_logger().info('Emergency stop cleared')
            self.action_executor.clear_emergency_stop()

        # Publish robot state update
        self.publish_robot_state()

    def publish_task_status(self, status_data: Dict[str, Any]):
        """
        Publish task execution status.
        """
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.task_status_publisher.publish(status_msg)

    def publish_robot_state(self):
        """
        Publish current robot state.
        """
        state = self.action_executor.get_execution_status()
        state['timestamp'] = self.get_clock().now().to_msg()

        state_msg = String()
        state_msg.data = json.dumps(state)
        self.robot_state_publisher.publish(state_msg)

    def publish_execution_error(self, error_message: str):
        """
        Publish execution error status.
        """
        error_status = {
            'status': 'error',
            'error': error_message,
            'timestamp': self.get_clock().now().to_msg()
        }

        status_msg = String()
        status_msg.data = json.dumps(error_status)
        self.task_status_publisher.publish(status_msg)

    def get_pose_for_location(self, location_name: str) -> Optional[Pose]:
        """
        Get pose for predefined location.
        """
        # This would typically come from a map or configuration
        location_poses = {
            'kitchen': Pose(position=Point(x=1.0, y=2.0, z=0.0),
                           orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
            'living room': Pose(position=Point(x=0.0, y=0.0, z=0.0),
                              orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
            'office': Pose(position=Point(x=2.0, y=1.0, z=0.0),
                         orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
        }

        return location_poses.get(location_name)

    def execute_navigation_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute navigation action (placeholder for real navigation).
        """
        target_location = action.get('target_location', 'unknown')
        self.get_logger().info(f'Navigating to: {target_location}')

        # Simulate navigation time
        time.sleep(2)

        # In a real implementation, you would use ROS 2 navigation
        # self.send_navigation_goal(target_location)

        return True  # Simulate success

    def execute_manipulation_action(self, action: Dict[str, Any]) -> bool:
        """
        Execute manipulation action (placeholder for real manipulation).
        """
        action_detail = action.get('action', 'unknown')
        target_object = action.get('target_object', 'unknown')
        self.get_logger().info(f'Performing manipulation: {action_detail} {target_object}')

        # Simulate manipulation time
        time.sleep(2)

        # In a real implementation, you would use ROS 2 manipulation
        # self.send_manipulation_goal(action_detail, target_object)

        return True  # Simulate success


def main(args=None):
    rclpy.init(args=args)

    humanoid_controller_node = HumanoidControllerNode()

    try:
        rclpy.spin(humanoid_controller_node)
    except KeyboardInterrupt:
        humanoid_controller_node.get_logger().info('Humanoid Controller Node interrupted')
    finally:
        humanoid_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()