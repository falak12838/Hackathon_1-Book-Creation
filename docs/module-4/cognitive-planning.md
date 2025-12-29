# Cognitive Planning: Translating Language to ROS 2 Actions

## Overview

In this module, we'll explore how to implement cognitive planning that translates complex natural language commands into sequences of ROS 2 actions. This builds on the voice-to-action foundation to enable sophisticated robot behaviors through natural language interaction.

## Learning Objectives

By the end of this chapter, you will:
- Understand cognitive planning concepts for robotics
- Implement a language-to-action translation system
- Create action sequencing algorithms for complex tasks
- Integrate with ROS 2 for multi-step robot execution

## Prerequisites

- Understanding of ROS 2 concepts and action architecture
- Basic knowledge of language processing
- Completion of the Voice-to-Action module

## Cognitive Planning Architecture

Cognitive planning in robotics involves breaking down high-level goals into executable action sequences. For our VLA system, this means translating complex natural language commands into sequences of ROS 2 actions that achieve the desired outcome.

### Core Components

1. **Language Parser**: Interprets natural language commands
2. **Action Planner**: Generates sequences of actions
3. **Environment Model**: Maintains awareness of the world state
4. **Action Executor**: Executes the planned sequence

## Implementing the Cognitive Planner

Let's create the cognitive planner ROS 2 node:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
import json

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Subscribe to high-level commands
        self.command_subscription = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10
        )

        # Publisher for action sequences
        self.action_sequence_publisher = self.create_publisher(
            String,
            'action_sequence',
            10
        )

        # Publisher for robot state updates
        self.state_publisher = self.create_publisher(
            String,
            'robot_state',
            10
        )

        # Action clients for different robot capabilities
        # These would be specific to your robot's action servers
        # self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.manipulation_client = ActionClient(self, MoveGroup, 'move_group')

        self.get_logger().info('Cognitive Planner Node initialized')

    def command_callback(self, msg):
        """
        Callback for high-level commands
        """
        command_text = msg.data
        self.get_logger().info(f'Received high-level command: {command_text}')

        # Plan the sequence of actions
        action_sequence = self.plan_actions(command_text)

        if action_sequence:
            # Publish the action sequence
            sequence_msg = String()
            sequence_msg.data = json.dumps(action_sequence)
            self.action_sequence_publisher.publish(sequence_msg)
            self.get_logger().info(f'Published action sequence: {action_sequence}')
        else:
            self.get_logger().warn(f'Could not plan actions for command: {command_text}')

    def plan_actions(self, command_text):
        """
        Plan sequence of actions based on command text
        """
        # This is a simplified example - a real implementation would use
        # more sophisticated NLP and planning algorithms
        command_lower = command_text.lower()

        # Example: "Go to the kitchen and bring me the red cup"
        if 'kitchen' in command_lower and 'bring' in command_lower:
            return self.plan_kitchen_fetch_task(command_text)
        elif 'move' in command_lower or 'go to' in command_lower:
            return self.plan_navigation_task(command_text)
        elif 'pick up' in command_lower or 'grasp' in command_lower:
            return self.plan_manipulation_task(command_text)
        else:
            return self.plan_generic_task(command_text)

    def plan_kitchen_fetch_task(self, command_text):
        """
        Plan a kitchen fetch task
        """
        actions = []

        # 1. Navigate to kitchen
        actions.append({
            'action_type': 'navigation',
            'target_location': 'kitchen',
            'description': 'Navigate to kitchen area'
        })

        # 2. Identify the red cup
        actions.append({
            'action_type': 'object_detection',
            'target_object': 'red cup',
            'description': 'Detect and locate the red cup'
        })

        # 3. Grasp the cup
        actions.append({
            'action_type': 'manipulation',
            'action': 'grasp',
            'target_object': 'red cup',
            'description': 'Grasp the red cup'
        })

        # 4. Navigate back
        actions.append({
            'action_type': 'navigation',
            'target_location': 'origin',
            'description': 'Return to original location'
        })

        # 5. Place the cup
        actions.append({
            'action_type': 'manipulation',
            'action': 'place',
            'location': 'near user',
            'description': 'Place the cup near the user'
        })

        return actions

    def plan_navigation_task(self, command_text):
        """
        Plan a navigation task
        """
        # Extract target location from command
        target_location = self.extract_location(command_text)

        return [{
            'action_type': 'navigation',
            'target_location': target_location,
            'description': f'Navigate to {target_location}'
        }]

    def plan_manipulation_task(self, command_text):
        """
        Plan a manipulation task
        """
        # Determine the manipulation action
        if 'pick up' in command_text.lower():
            action = 'grasp'
        elif 'put down' in command_text.lower():
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

    def plan_generic_task(self, command_text):
        """
        Plan a generic task
        """
        # For complex commands, we might need more sophisticated parsing
        # This is a simplified implementation
        return [{
            'action_type': 'unknown',
            'command': command_text,
            'description': 'Generic task - needs further processing'
        }]

    def extract_location(self, command_text):
        """
        Extract target location from command
        """
        # Simple keyword matching - in practice, use NLP techniques
        locations = ['kitchen', 'living room', 'bedroom', 'office', 'bathroom', 'dining room']
        for location in locations:
            if location in command_text.lower():
                return location
        return 'unknown'

    def extract_object(self, command_text):
        """
        Extract target object from command
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
        pass
    finally:
        cognitive_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Planning with LLM Integration

For more sophisticated cognitive planning, we can integrate with Large Language Models:

```python
import openai
import os
from dotenv import load_dotenv

class LLMCognitivePlanner:
    def __init__(self):
        load_dotenv()
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not found in environment variables")

    def plan_complex_task(self, command_text, environment_context=None):
        """
        Use LLM to plan complex multi-step tasks
        """
        # Define the system prompt for the planning task
        system_prompt = """
        You are a cognitive planning assistant for a humanoid robot.
        Your task is to break down high-level natural language commands
        into sequences of specific robot actions.

        Each action should be one of the following types:
        - navigation: Move to a specific location
        - object_detection: Identify and locate an object
        - manipulation: Grasp, place, or manipulate an object
        - communication: Speak to the user
        - perception: Sense the environment

        Return the plan as a JSON array of action objects with the following structure:
        [
            {
                "action_type": "navigation|object_detection|manipulation|communication|perception",
                "target_location|target_object|message": "specific target",
                "description": "Human-readable description"
            }
        ]
        """

        # Prepare the environment context
        context = environment_context or {}
        environment_info = f"Environment: {context}"

        # Create the user prompt
        user_prompt = f"""
        Command: {command_text}
        {environment_info}

        Plan the sequence of actions to execute this command:
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.1,
                max_tokens=500
            )

            plan_text = response.choices[0].message['content'].strip()

            # Extract JSON from the response (in case it includes other text)
            import re
            json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
            if json_match:
                import json
                plan = json.loads(json_match.group())
                return plan
            else:
                # If no JSON found, return None or handle error
                return None

        except Exception as e:
            print(f"Error in LLM planning: {e}")
            return None
```

## Integration with ROS 2 Actions

To properly execute the planned actions, we need to integrate with ROS 2 action architecture:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, Point, Quaternion

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Action clients for different capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher for executing action sequences
        self.sequence_subscriber = self.create_subscription(
            String,
            'action_sequence',
            self.execute_sequence_callback,
            10
        )

        self.get_logger().info('Action Executor initialized')

    def execute_sequence_callback(self, msg):
        """
        Execute a sequence of actions
        """
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Executing action sequence: {len(action_sequence)} actions')

            for i, action in enumerate(action_sequence):
                self.get_logger().info(f'Executing action {i+1}: {action["description"]}')

                success = self.execute_single_action(action)
                if not success:
                    self.get_logger().error(f'Action failed: {action["description"]}')
                    break

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding action sequence: {e}')

    async def execute_single_action(self, action):
        """
        Execute a single action based on its type
        """
        action_type = action.get('action_type', 'unknown')

        if action_type == 'navigation':
            return await self.execute_navigation_action(action)
        elif action_type == 'manipulation':
            return await self.execute_manipulation_action(action)
        elif action_type == 'object_detection':
            return await self.execute_perception_action(action)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return False

    async def execute_navigation_action(self, action):
        """
        Execute navigation action
        """
        target_location = action.get('target_location', 'unknown')

        # In a real implementation, you'd have predefined locations
        # or a way to determine coordinates from location names
        pose = self.get_pose_for_location(target_location)
        if pose:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            goal_msg.behavior_tree = ''  # Use default behavior tree

            self.get_logger().info(f'Navigating to {target_location}')

            # Send navigation goal
            future = self.nav_client.send_goal_async(goal_msg)
            goal_handle = await future

            if not goal_handle.accepted:
                self.get_logger().error('Navigation goal rejected')
                return False

            result_future = goal_handle.get_result_async()
            result = await result_future

            return result.result.status == GoalStatus.STATUS_SUCCEEDED
        else:
            self.get_logger().error(f'Unknown location: {target_location}')
            return False

    def get_pose_for_location(self, location_name):
        """
        Get pose for predefined location
        """
        # This would typically come from a map or configuration
        location_poses = {
            'kitchen': Pose(position=Point(x=1.0, y=2.0, z=0.0),
                           orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
            'living room': Pose(position=Point(x=0.0, y=0.0, z=0.0),
                              orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)),
        }

        return location_poses.get(location_name)
```

## Environment Context and Perception

For effective planning, the cognitive planner needs to maintain awareness of the environment:

```python
class EnvironmentContext:
    def __init__(self):
        self.objects = {}
        self.locations = {}
        self.robot_state = {}
        self.last_updated = None

    def update_object(self, object_id, pose, properties=None):
        """
        Update information about an object in the environment
        """
        self.objects[object_id] = {
            'pose': pose,
            'properties': properties or {},
            'last_seen': self.get_current_time()
        }

    def get_reachable_objects(self, robot_pose, max_distance=2.0):
        """
        Get objects that are reachable from the robot's current pose
        """
        reachable = []
        for obj_id, obj_data in self.objects.items():
            distance = self.calculate_distance(robot_pose, obj_data['pose'])
            if distance <= max_distance:
                reachable.append(obj_id)
        return reachable

    def calculate_distance(self, pose1, pose2):
        """
        Calculate Euclidean distance between two poses
        """
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return (dx*dx + dy*dy + dz*dz)**0.5

    def get_current_time(self):
        """
        Get current timestamp
        """
        import time
        return time.time()
```

## Planning Algorithm Examples

Let's look at some specific examples of planning algorithms that can be used in cognitive planning systems.

### 1. Hierarchical Task Network (HTN) Planning

HTN planning decomposes high-level tasks into subtasks using predefined methods:

```python
class HTNPlanner:
    def __init__(self):
        # Define methods for decomposing tasks
        self.methods = {
            'fetch_object': [self.decompose_fetch_object],
            'navigate_to_location': [self.decompose_navigate],
            'manipulate_object': [self.decompose_manipulate]
        }

    def decompose_fetch_object(self, task):
        """
        Decompose fetch object task into subtasks
        """
        return [
            {'type': 'navigate_to_location', 'location': task['target_location']},
            {'type': 'locate_object', 'object': task['target_object']},
            {'type': 'grasp_object', 'object': task['target_object']},
            {'type': 'navigate_to_location', 'location': task['return_location']}
        ]

    def decompose_navigate(self, task):
        """
        Decompose navigation task
        """
        return [
            {'type': 'path_planning', 'target': task['location']},
            {'type': 'motion_execution', 'path': task['location']}
        ]

    def plan(self, task):
        """
        Plan a task using HTN decomposition
        """
        if task['type'] in self.methods:
            # Apply decomposition methods
            subtasks = []
            for method in self.methods[task['type']]:
                subtasks.extend(method(task))
            return subtasks
        else:
            # Return the task as is if no decomposition available
            return [task]
```

### 2. Symbolic Planning with STRIPS

STRIPS-style planning uses states and operators to find a sequence of actions:

```python
class STRIPSPlanner:
    def __init__(self):
        # Define operators with preconditions and effects
        self.operators = [
            {
                'name': 'navigate',
                'preconditions': ['at(location1)', 'robot_free'],
                'effects': ['at(location2)', 'not_at(location1)'],
                'cost': 1.0
            },
            {
                'name': 'grasp',
                'preconditions': ['at(object_location)', 'object_reachable', 'gripper_free'],
                'effects': ['holding(object)', 'not_gripper_free'],
                'cost': 0.5
            },
            {
                'name': 'place',
                'preconditions': ['holding(object)', 'surface_free'],
                'effects': ['not_holding(object)', 'object_on_surface', 'gripper_free'],
                'cost': 0.5
            }
        ]

    def plan(self, initial_state, goal_state):
        """
        Plan using STRIPS-style search
        """
        # Use a search algorithm (e.g., A*) to find a plan
        # This is a simplified example
        plan = self.search_plan(initial_state, goal_state)
        return plan

    def search_plan(self, initial_state, goal_state):
        """
        Search for a plan using forward state space search
        """
        # Implementation of search algorithm
        # This would typically use A* or similar
        pass
```

### 3. Behavior Trees for Task Execution

Behavior trees provide a structured way to execute planned sequences:

```python
class BehaviorTreeNode:
    def tick(self, context):
        """
        Execute the behavior and return status
        """
        pass

class SequenceNode(BehaviorTreeNode):
    def __init__(self, children):
        self.children = children

    def tick(self, context):
        """
        Execute children in sequence until one fails
        """
        for child in self.children:
            status = child.tick(context)
            if status != 'SUCCESS':
                return status
        return 'SUCCESS'

class SelectorNode(BehaviorTreeNode):
    def __init__(self, children):
        self.children = children

    def tick(self, context):
        """
        Execute children until one succeeds
        """
        for child in self.children:
            status = child.tick(context)
            if status == 'SUCCESS':
                return status
        return 'FAILURE'

class NavigationNode(BehaviorTreeNode):
    def __init__(self, target_location):
        self.target_location = target_location

    def tick(self, context):
        """
        Execute navigation action
        """
        # Execute ROS 2 navigation action
        # Return SUCCESS if navigation completes, FAILURE if it fails
        try:
            # Execute navigation
            success = context.execute_navigation(self.target_location)
            return 'SUCCESS' if success else 'FAILURE'
        except Exception:
            return 'FAILURE'

class CognitivePlannerWithBehaviorTrees:
    def __init__(self):
        self.planner = HTNPlanner()  # or STRIPSPlanner

    def create_behavior_tree_for_plan(self, plan):
        """
        Convert a plan into a behavior tree
        """
        children = []
        for action in plan:
            if action['type'] == 'navigate':
                children.append(NavigationNode(action['location']))
            # Add other action types as needed
        return SequenceNode(children)
```

### 4. Integration Example: Complete Planning Pipeline

Here's how all these components work together:

```python
class IntegratedCognitivePlanner:
    def __init__(self):
        self.hierarchical_planner = HTNPlanner()
        self.strips_planner = STRIPSPlanner()
        self.behavior_tree_executor = None
        self.llm_planner = LLMCognitivePlanner()  # From earlier example
        self.environment_context = EnvironmentContext()

    def plan_and_execute(self, natural_language_command, environment_context=None):
        """
        Complete pipeline: natural language -> plan -> execute
        """
        # Step 1: Parse natural language command
        high_level_task = self.parse_command(natural_language_command)

        # Step 2: Plan the task using appropriate method
        if self.is_complex_command(natural_language_command):
            # Use LLM for complex commands
            action_sequence = self.llm_planner.plan_complex_task(
                natural_language_command,
                environment_context
            )
        else:
            # Use classical planning for simple commands
            subtasks = self.hierarchical_planner.plan(high_level_task)
            action_sequence = self.convert_to_action_sequence(subtasks)

        # Step 3: Validate the plan
        if not self.validate_plan(action_sequence, environment_context):
            raise ValueError("Plan validation failed")

        # Step 4: Execute the plan using behavior trees
        behavior_tree = self.create_behavior_tree_for_plan(action_sequence)
        self.behavior_tree_executor = behavior_tree
        execution_result = behavior_tree.tick(self.get_execution_context())

        return execution_result

    def parse_command(self, command):
        """
        Parse natural language command into structured task
        """
        # Use NLP techniques to extract task structure
        # This is simplified - in practice, use proper NLP
        if 'fetch' in command.lower() or 'bring' in command.lower():
            return {'type': 'fetch_object', 'target_object': self.extract_object(command)}
        elif 'go to' in command.lower() or 'navigate' in command.lower():
            return {'type': 'navigate_to_location', 'location': self.extract_location(command)}
        else:
            return {'type': 'unknown', 'command': command}

    def validate_plan(self, plan, context):
        """
        Validate the plan against environment constraints
        """
        # Check if all required resources are available
        # Check if all locations are navigable
        # Check if all objects are manipulable
        return True  # Simplified validation

    def get_execution_context(self):
        """
        Get execution context for behavior tree
        """
        return {
            'execute_navigation': self.execute_navigation,
            'execute_manipulation': self.execute_manipulation,
            'environment': self.environment_context
        }

    def execute_navigation(self, target_location):
        """
        Execute navigation in ROS 2
        """
        # Implementation of ROS 2 navigation execution
        pass

    def execute_manipulation(self, action):
        """
        Execute manipulation in ROS 2
        """
        # Implementation of ROS 2 manipulation execution
        pass
```

## Testing the Cognitive Planning System

Here's a test script to verify the cognitive planning functionality:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class CognitivePlannerTester(Node):
    def __init__(self):
        super().__init__('cognitive_planner_tester')

        # Publisher for test commands
        self.command_publisher = self.create_publisher(
            String,
            'high_level_command',
            10
        )

        # Timer to send test commands
        self.timer = self.create_timer(5.0, self.send_test_commands)
        self.command_index = 0
        self.test_commands = [
            "Go to the kitchen",
            "Pick up the red cup",
            "Go to the kitchen and bring me the red cup",
            "Move forward 1 meter"
        ]

        self.get_logger().info('Cognitive Planner Tester initialized')

    def send_test_commands(self):
        """
        Send test commands periodically
        """
        if self.command_index < len(self.test_commands):
            command = self.test_commands[self.command_index]
            msg = String()
            msg.data = command

            self.command_publisher.publish(msg)
            self.get_logger().info(f'Sent test command: {command}')

            self.command_index += 1
        else:
            self.get_logger().info('All test commands sent')

def main(args=None):
    rclpy.init(args=args)
    tester = CognitivePlannerTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Cognitive Planning

1. **Modular Design**: Keep planning, execution, and perception components separate but well-integrated
2. **Error Recovery**: Implement fallback plans when actions fail
3. **Context Awareness**: Continuously update environment model based on sensor data
4. **Human-in-the-Loop**: Allow human intervention during plan execution
5. **Safety First**: Always include safety checks in planned actions
6. **Plan Validation**: Validate action sequences before execution
7. **Performance Optimization**: Use appropriate planning algorithms based on task complexity
8. **Debugging Support**: Include logging and visualization for plan debugging

## Summary

In this chapter, we've implemented a cognitive planning system that translates complex natural language commands into sequences of ROS 2 actions. We've covered the architecture, implementation of planning algorithms, integration with LLMs, and execution frameworks. This enables robots to understand and execute complex, multi-step tasks through natural language interaction.

## Next Steps

- Implement the capstone autonomous humanoid system (next chapter)
- Integrate all VLA components for end-to-end functionality
- Add advanced perception capabilities
- Implement comprehensive safety and error handling