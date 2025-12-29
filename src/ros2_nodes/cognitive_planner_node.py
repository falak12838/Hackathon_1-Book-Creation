#!/usr/bin/env python3

"""
Cognitive Planner Node for the Vision-Language-Action (VLA) system.
Translates high-level commands into sequences of robot actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import json
from typing import Optional, Dict, Any

# Import local modules
try:
    from vla.cognitive_planner import IntegratedCognitivePlanner
except ImportError:
    # If not in the same package, try alternative import
    from src.vla.cognitive_planner import IntegratedCognitivePlanner


class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Initialize cognitive planner
        try:
            self.planner = IntegratedCognitivePlanner()
        except ValueError as e:
            self.get_logger().error(f'Failed to initialize cognitive planner: {e}')
            raise

        # Publishers
        self.action_sequence_publisher = self.create_publisher(
            String,
            'action_sequence',
            10
        )

        # Publishers for intermediate results
        self.planning_status_publisher = self.create_publisher(
            String,
            'planning_status',
            10
        )

        # Subscribers
        self.command_subscription = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10
        )

        # Subscribe to environment context updates
        self.environment_subscription = self.create_subscription(
            String,
            'environment_context',
            self.environment_callback,
            10
        )

        # Store environment context
        self.environment_context = {}

        self.get_logger().info('Cognitive Planner Node initialized and ready')

    def command_callback(self, msg):
        """
        Callback for high-level commands.
        """
        command_text = msg.data
        self.get_logger().info(f'Received high-level command: {command_text}')

        try:
            # Plan the sequence of actions
            action_sequence = self.plan_actions(command_text)

            if action_sequence:
                # Publish the action sequence
                sequence_msg = String()
                sequence_msg.data = json.dumps(action_sequence)
                self.action_sequence_publisher.publish(sequence_msg)
                self.get_logger().info(f'Published action sequence with {len(action_sequence)} actions')
            else:
                self.get_logger().warn(f'Could not plan actions for command: {command_text}')

                # Publish error status
                status_msg = String()
                status_msg.data = json.dumps({
                    'status': 'failed',
                    'command': command_text,
                    'error': 'Could not generate action sequence'
                })
                self.planning_status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error in command processing: {e}')

            # Publish error status
            status_msg = String()
            status_msg.data = json.dumps({
                'status': 'error',
                'command': command_text,
                'error': str(e)
            })
            self.planning_status_publisher.publish(status_msg)

    def environment_callback(self, msg):
        """
        Callback for environment context updates.
        """
        try:
            self.environment_context = json.loads(msg.data)
            self.get_logger().info('Updated environment context')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding environment context: {e}')

    def plan_actions(self, command_text: str) -> Optional[list]:
        """
        Plan sequence of actions based on command text.
        """
        try:
            # Plan using the integrated cognitive planner
            # For now, use LLM-based planning (can be configured)
            action_sequence = self.planner.plan_task(
                command_text,
                self.environment_context,
                use_llm=True
            )

            if action_sequence:
                # Validate the plan
                is_valid = self.planner.validate_plan(action_sequence, self.environment_context)
                if is_valid:
                    self.get_logger().info(f'Successfully planned {len(action_sequence)} actions')

                    # Publish success status
                    status_msg = String()
                    status_msg.data = json.dumps({
                        'status': 'success',
                        'command': command_text,
                        'action_count': len(action_sequence)
                    })
                    self.planning_status_publisher.publish(status_msg)

                    return action_sequence
                else:
                    self.get_logger().warn('Planned sequence failed validation')
                    return None
            else:
                self.get_logger().warn(f'Planning failed for command: {command_text}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error in action planning: {e}')
            return None

    def plan_navigation_task(self, command_text: str) -> list:
        """
        Plan a navigation task.
        """
        # Extract target location from command
        target_location = self.extract_location(command_text)

        return [{
            'action_type': 'navigation',
            'action': 'navigate_to',
            'target_location': target_location,
            'description': f'Navigate to {target_location}'
        }]

    def plan_manipulation_task(self, command_text: str) -> list:
        """
        Plan a manipulation task.
        """
        # Determine the manipulation action
        if 'pick up' in command_text.lower() or 'grasp' in command_text.lower():
            action = 'grasp'
        elif 'put down' in command_text.lower() or 'place' in command_text.lower():
            action = 'place'
        else:
            action = 'manipulate'

        target_object = self.extract_object(command_text)

        return [{
            'action_type': 'manipulation',
            'action': action,
            'target_object': target_object,
            'description': f'{action} the {target_object}'
        }]

    def plan_generic_task(self, command_text: str) -> list:
        """
        Plan a generic task.
        """
        # For complex commands, we might need more sophisticated parsing
        # This is a simplified implementation
        return [{
            'action_type': 'unknown',
            'command': command_text,
            'description': 'Generic task - needs further processing'
        }]

    def extract_location(self, command_text: str) -> str:
        """
        Extract target location from command.
        """
        # Simple keyword matching - in practice, use NLP techniques
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'bathroom', 'dining room']
        for location in locations:
            if location in command_text.lower():
                return location
        return 'unknown'

    def extract_object(self, command_text: str) -> str:
        """
        Extract target object from command.
        """
        # Simple keyword matching - in practice, use NLP techniques
        import re
        # Look for object descriptions like "red cup", "book", "ball"
        words = command_text.lower().split()
        for i, word in enumerate(words):
            if word in ['the', 'a', 'an']:
                if i + 1 < len(words):
                    return words[i + 1]
        return 'unknown'


def main(args=None):
    rclpy.init(args=args)

    cognitive_planner_node = CognitivePlannerNode()

    try:
        rclpy.spin(cognitive_planner_node)
    except KeyboardInterrupt:
        cognitive_planner_node.get_logger().info('Cognitive Planner Node interrupted')
    finally:
        cognitive_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()