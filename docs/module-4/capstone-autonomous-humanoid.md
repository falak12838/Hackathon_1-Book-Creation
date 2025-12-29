# Capstone: Autonomous Humanoid – Integration & Task Execution

## Overview

This capstone module brings together all components of the Vision-Language-Action (VLA) system to create a fully autonomous humanoid robot capable of understanding voice commands, planning complex tasks, and executing them safely in real-world environments. This represents the complete integration of voice processing, cognitive planning, and robot control.

## Learning Objectives

By the end of this chapter, you will:
- Integrate all VLA system components into a unified architecture
- Implement autonomous task execution with human-in-the-loop oversight
- Create safety protocols and emergency intervention systems
- Test complete end-to-end scenarios with the humanoid robot

## Prerequisites

- Understanding of all previous modules (Voice-to-Action and Cognitive Planning)
- ROS 2 environment with navigation and manipulation capabilities
- Microphone and audio processing setup
- OpenAI API access for Whisper and LLM integration

## System Architecture

The complete VLA system architecture integrates all components into a cohesive autonomous system:

```
User Voice Command
        ↓
[Voice Processing] → [Natural Language Understanding] → [Cognitive Planning]
        ↓                           ↓                           ↓
[Audio Input] → [Whisper API] → [Intent Recognition] → [Action Sequencing]
        ↓
[ROS 2 Command] → [Action Execution] → [Robot Control] → [Task Monitoring]
        ↑                                                   ↓
[Human Oversight] ← [Safety Supervisor] ← [Emergency Override]
```

## Implementing the Unified VLA System

Let's create the main orchestrator node that brings all components together:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
import json
import threading
import time
from enum import Enum

class SystemState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    PLANNING = "planning"
    EXECUTING = "executing"
    SAFETY_OVERRIDE = "safety_override"
    COMPLETED = "completed"
    ERROR = "error"

class VLAMasterNode(Node):
    def __init__(self):
        super().__init__('vla_master_node')

        # Initialize system state
        self.current_state = SystemState.IDLE
        self.current_task = None
        self.task_result = None
        self.emergency_stop = False

        # Publishers
        self.state_publisher = self.create_publisher(String, 'vla_system_state', 10)
        self.command_publisher = self.create_publisher(String, 'high_level_command', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.feedback_publisher = self.create_publisher(String, 'user_feedback', 10)

        # Subscribers
        self.voice_command_subscriber = self.create_subscription(
            String, 'robot_command', self.voice_command_callback, 10)
        self.action_sequence_subscriber = self.create_subscription(
            String, 'action_sequence', self.action_sequence_callback, 10)
        self.task_status_subscriber = self.create_subscription(
            String, 'task_status', self.task_status_callback, 10)
        self.emergency_override_subscriber = self.create_subscription(
            Bool, 'emergency_override', self.emergency_override_callback, 10)

        # Timer for state monitoring
        self.state_timer = self.create_timer(1.0, self.state_monitor_callback)

        # Initialize component interfaces
        self.initialize_components()

        self.get_logger().info('VLA Master Node initialized and ready')

    def initialize_components(self):
        """
        Initialize all VLA system components
        """
        # Initialize speech processor (from voice-to-action module)
        from speech_processor import SpeechProcessor
        self.speech_processor = SpeechProcessor()

        # Initialize command interpreter (from voice-to-action module)
        from command_interpreter import CommandInterpreter
        self.command_interpreter = CommandInterpreter()

        # Initialize cognitive planner (from cognitive planning module)
        from cognitive_planner import LLMCognitivePlanner
        self.cognitive_planner = LLMCognitivePlanner()

        self.get_logger().info('All VLA components initialized')

    def voice_command_callback(self, msg):
        """
        Handle voice commands from the voice-to-action system
        """
        if self.emergency_stop:
            self.get_logger().warn('Ignoring command due to emergency stop')
            return

        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        # Update state
        self.current_state = SystemState.PLANNING
        self.publish_state()

        # Plan the action sequence
        self.plan_and_execute_command(command)

    def plan_and_execute_command(self, command_text):
        """
        Plan and execute a command using cognitive planning
        """
        try:
            # Use cognitive planner to generate action sequence
            action_sequence = self.cognitive_planner.plan_complex_task(
                command_text,
                self.get_environment_context()
            )

            if action_sequence:
                self.get_logger().info(f'Generated action sequence: {action_sequence}')

                # Publish action sequence for execution
                sequence_msg = String()
                sequence_msg.data = json.dumps(action_sequence)
                self.command_publisher.publish(sequence_msg)

                # Update state to executing
                self.current_state = SystemState.EXECUTING
                self.current_task = command_text
                self.publish_state()
            else:
                self.get_logger().error('Failed to generate action sequence')
                self.handle_error('Planning failed')
        except Exception as e:
            self.get_logger().error(f'Error in planning: {e}')
            self.handle_error(f'Planning error: {str(e)}')

    def action_sequence_callback(self, msg):
        """
        Handle action sequences from cognitive planner
        """
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f'Executing action sequence with {len(action_sequence)} actions')

            # Execute the sequence (in a separate thread to avoid blocking)
            execution_thread = threading.Thread(
                target=self.execute_action_sequence,
                args=(action_sequence,)
            )
            execution_thread.start()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding action sequence: {e}')
            self.handle_error(f'Decoding error: {str(e)}')

    def execute_action_sequence(self, action_sequence):
        """
        Execute a sequence of actions with monitoring and error handling
        """
        for i, action in enumerate(action_sequence):
            if self.emergency_stop:
                self.get_logger().warn('Emergency stop during action execution')
                self.current_state = SystemState.SAFETY_OVERRIDE
                self.publish_state()
                return

            self.get_logger().info(f'Executing action {i+1}/{len(action_sequence)}: {action["description"]}')

            success = self.execute_single_action(action)
            if not success:
                self.get_logger().error(f'Action failed: {action["description"]}')
                self.handle_error(f'Action execution failed: {action["description"]}')
                return

        # All actions completed successfully
        self.get_logger().info('All actions completed successfully')
        self.current_state = SystemState.COMPLETED
        self.publish_state()

        # Provide feedback to user
        feedback_msg = String()
        feedback_msg.data = f'Task completed: {self.current_task}'
        self.feedback_publisher.publish(feedback_msg)

    def execute_single_action(self, action):
        """
        Execute a single action based on its type
        """
        try:
            action_type = action.get('action_type', 'unknown')

            if action_type == 'navigation':
                return self.execute_navigation_action(action)
            elif action_type == 'manipulation':
                return self.execute_manipulation_action(action)
            elif action_type == 'object_detection':
                return self.execute_perception_action(action)
            elif action_type == 'communication':
                return self.execute_communication_action(action)
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing action: {e}')
            return False

    def execute_navigation_action(self, action):
        """
        Execute navigation action
        """
        # Implementation would call ROS 2 navigation system
        target_location = action.get('target_location', 'unknown')
        self.get_logger().info(f'Navigating to: {target_location}')

        # Simulate navigation (in real implementation, use ROS 2 navigation)
        time.sleep(2)  # Simulate navigation time

        return True  # Simulate success

    def execute_manipulation_action(self, action):
        """
        Execute manipulation action
        """
        # Implementation would call ROS 2 manipulation system
        action_detail = action.get('action', 'unknown')
        target_object = action.get('target_object', 'unknown')
        self.get_logger().info(f'Performing manipulation: {action_detail} {target_object}')

        # Simulate manipulation (in real implementation, use ROS 2 manipulation)
        time.sleep(2)  # Simulate manipulation time

        return True  # Simulate success

    def execute_perception_action(self, action):
        """
        Execute perception action
        """
        # Implementation would call ROS 2 perception system
        target_object = action.get('target_object', 'unknown')
        self.get_logger().info(f'Detecting object: {target_object}')

        # Simulate perception (in real implementation, use ROS 2 perception)
        time.sleep(1)  # Simulate perception time

        return True  # Simulate success

    def execute_communication_action(self, action):
        """
        Execute communication action
        """
        # Implementation would call text-to-speech system
        message = action.get('message', 'Hello')
        self.get_logger().info(f'Communicating: {message}')

        # Simulate communication (in real implementation, use TTS)
        time.sleep(1)  # Simulate communication time

        return True  # Simulate success

    def task_status_callback(self, msg):
        """
        Handle task status updates
        """
        try:
            status_data = json.loads(msg.data)
            task_id = status_data.get('task_id', 'unknown')
            status = status_data.get('status', 'unknown')

            self.get_logger().info(f'Task {task_id} status: {status}')

            # Update internal state based on task status
            if status == 'completed':
                self.current_state = SystemState.COMPLETED
            elif status == 'failed':
                self.current_state = SystemState.ERROR
            elif status == 'running':
                self.current_state = SystemState.EXECUTING

            self.publish_state()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error decoding task status: {e}')

    def emergency_override_callback(self, msg):
        """
        Handle emergency override commands
        """
        if msg.data:  # Emergency stop activated
            self.emergency_stop = True
            self.current_state = SystemState.SAFETY_OVERRIDE
            self.get_logger().warn('Emergency stop activated')
        else:  # Emergency stop cleared
            self.emergency_stop = False
            self.current_state = SystemState.IDLE
            self.get_logger().info('Emergency stop cleared')

        self.publish_state()

    def state_monitor_callback(self):
        """
        Monitor system state and publish updates
        """
        self.publish_state()

    def publish_state(self):
        """
        Publish current system state
        """
        state_msg = String()
        state_msg.data = json.dumps({
            'state': self.current_state.value,
            'current_task': self.current_task,
            'emergency_stop': self.emergency_stop,
            'timestamp': self.get_clock().now().to_msg()
        })
        self.state_publisher.publish(state_msg)

    def get_environment_context(self):
        """
        Get current environment context for planning
        """
        # In a real implementation, this would gather data from sensors
        # For simulation, return a static context
        return {
            'objects': ['red cup', 'book', 'ball'],
            'locations': ['kitchen', 'living room', 'office'],
            'robot_pose': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery_level': 0.85
        }

    def handle_error(self, error_message):
        """
        Handle system errors
        """
        self.get_logger().error(f'System error: {error_message}')
        self.current_state = SystemState.ERROR
        self.publish_state()

        # Provide error feedback to user
        feedback_msg = String()
        feedback_msg.data = f'Error: {error_message}'
        self.feedback_publisher.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)

    vla_master_node = VLAMasterNode()

    try:
        rclpy.spin(vla_master_node)
    except KeyboardInterrupt:
        vla_master_node.get_logger().info('VLA Master Node interrupted')
    finally:
        vla_master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Safety Supervisor System

Safety is critical in autonomous humanoid systems. Let's implement a safety supervisor:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import threading
import time

class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')

        # Publishers
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_status_publisher = self.create_publisher(String, 'safety_status', 10)
        self.velocity_override_publisher = self.create_publisher(Twist, 'cmd_vel_override', 10)

        # Subscribers
        self.laser_scan_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_scan_callback, 10)
        self.system_state_subscriber = self.create_subscription(
            String, 'vla_system_state', self.system_state_callback, 10)
        self.robot_velocity_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.velocity_callback, 10)

        # Safety parameters
        self.safety_distance = 0.5  # meters
        self.emergency_stop_active = False
        self.system_state = 'idle'
        self.last_scan_time = None
        self.last_velocity_time = None

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_monitor_callback)  # 10Hz

        self.get_logger().info('Safety Supervisor initialized')

    def laser_scan_callback(self, msg):
        """
        Handle laser scan data for obstacle detection
        """
        self.last_scan_time = self.get_clock().now()

        # Check for obstacles within safety distance
        min_distance = min(msg.ranges) if msg.ranges else float('inf')

        if min_distance < self.safety_distance and not self.emergency_stop_active:
            self.trigger_emergency_stop(f'Obstacle detected at {min_distance:.2f}m (threshold: {self.safety_distance}m)')

    def system_state_callback(self, msg):
        """
        Handle system state updates
        """
        try:
            state_data = json.loads(msg.data)
            self.system_state = state_data.get('state', 'idle')
        except json.JSONDecodeError:
            self.system_state = msg.data  # If not JSON, use raw data

    def velocity_callback(self, msg):
        """
        Monitor robot velocity for safety
        """
        self.last_velocity_time = self.get_clock().now()

        # Check for excessive velocities
        linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        angular_speed = (msg.angular.x**2 + msg.angular.y**2 + msg.angular.z**2)**0.5

        max_linear = 1.0  # m/s
        max_angular = 1.0  # rad/s

        if linear_speed > max_linear or angular_speed > max_angular:
            self.get_logger().warn(f'Velocity limit exceeded: linear={linear_speed:.2f}, angular={angular_speed:.2f}')

    def safety_monitor_callback(self):
        """
        Main safety monitoring loop
        """
        current_time = self.get_clock().now()

        # Check for sensor timeouts
        if self.last_scan_time:
            time_since_scan = (current_time - self.last_scan_time).nanoseconds / 1e9
            if time_since_scan > 1.0:  # 1 second timeout
                if not self.emergency_stop_active:
                    self.trigger_emergency_stop(f'Laser scan timeout: {time_since_scan:.2f}s')

        # Publish safety status
        self.publish_safety_status()

    def trigger_emergency_stop(self, reason):
        """
        Trigger emergency stop with reason
        """
        if not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.get_logger().error(f'EMERGENCY STOP: {reason}')

            # Publish emergency stop command
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_publisher.publish(stop_msg)

            # Override velocity to stop robot
            stop_twist = Twist()
            self.velocity_override_publisher.publish(stop_twist)

    def publish_safety_status(self):
        """
        Publish current safety status
        """
        status_msg = String()
        status_msg.data = json.dumps({
            'emergency_stop': self.emergency_stop_active,
            'safety_distance': self.safety_distance,
            'system_state': self.system_state,
            'timestamp': self.get_clock().now().to_msg()
        })
        self.safety_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)

    safety_supervisor = SafetySupervisor()

    try:
        rclpy.spin(safety_supervisor)
    except KeyboardInterrupt:
        safety_supervisor.get_logger().info('Safety Supervisor interrupted')
    finally:
        safety_supervisor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Human-in-the-Loop Interface

For human oversight and intervention, let's create a simple interface:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class HumanInLoopInterface(Node):
    def __init__(self):
        super().__init__('human_in_loop_interface')

        # Publishers
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.user_command_publisher = self.create_publisher(String, 'user_command', 10)

        # Subscribers
        self.safety_status_subscriber = self.create_subscription(
            String, 'safety_status', self.safety_status_callback, 10)
        self.system_state_subscriber = self.create_subscription(
            String, 'vla_system_state', self.system_state_callback, 10)

        # Initialize terminal for keyboard input
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.cbreak(sys.stdin.fileno())

        # Timer for keyboard monitoring
        self.keyboard_timer = self.create_timer(0.1, self.keyboard_monitor_callback)

        self.get_logger().info('Human-in-the-Loop Interface initialized')
        self.get_logger().info('Controls: [e] Emergency Stop, [c] Clear Emergency, [q] Quit')

    def safety_status_callback(self, msg):
        """
        Handle safety status updates
        """
        try:
            status_data = json.loads(msg.data)
            emergency_stop = status_data.get('emergency_stop', False)
            if emergency_stop:
                self.get_logger().warn('SYSTEM IS IN EMERGENCY STOP STATE')
        except json.JSONDecodeError:
            pass

    def system_state_callback(self, msg):
        """
        Handle system state updates
        """
        try:
            state_data = json.loads(msg.data)
            state = state_data.get('state', 'unknown')
            self.get_logger().info(f'System state: {state}')
        except json.JSONDecodeError:
            self.get_logger().info(f'System state: {msg.data}')

    def keyboard_monitor_callback(self):
        """
        Monitor keyboard input
        """
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)

            if key == 'e':  # Emergency stop
                self.get_logger().warn('EMERGENCY STOP ACTIVATED BY USER')
                stop_msg = Bool()
                stop_msg.data = True
                self.emergency_stop_publisher.publish(stop_msg)
            elif key == 'c':  # Clear emergency stop
                self.get_logger().info('Emergency stop cleared by user')
                stop_msg = Bool()
                stop_msg.data = False
                self.emergency_stop_publisher.publish(stop_msg)
            elif key == 'q':  # Quit
                self.get_logger().info('User requested shutdown')
                self.shutdown()
            else:
                # Handle other commands
                self.handle_command(key)

    def handle_command(self, key):
        """
        Handle other user commands
        """
        command_map = {
            'w': 'move forward',
            's': 'move backward',
            'a': 'turn left',
            'd': 'turn right',
            ' ': 'stop'
        }

        if key in command_map:
            command = command_map[key]
            self.get_logger().info(f'User command: {command}')
            cmd_msg = String()
            cmd_msg.data = command
            self.user_command_publisher.publish(cmd_msg)

    def shutdown(self):
        """
        Restore terminal settings and shutdown
        """
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    human_interface = HumanInLoopInterface()

    try:
        rclpy.spin(human_interface)
    except KeyboardInterrupt:
        human_interface.get_logger().info('Human Interface interrupted')
    finally:
        human_interface.shutdown()

if __name__ == '__main__':
    main()
```

## Complete System Launch File

To bring up the entire VLA system, here's a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # VLA Master Node (main orchestrator)
        Node(
            package='vla',
            executable='vla_master_node',
            name='vla_master_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Safety Supervisor
        Node(
            package='vla',
            executable='safety_supervisor',
            name='safety_supervisor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Human-in-the-Loop Interface
        Node(
            package='vla',
            executable='human_interface',
            name='human_interface',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Voice Command Node (from voice-to-action module)
        Node(
            package='vla',
            executable='voice_command_node',
            name='voice_command_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),

        # Cognitive Planner Node (from cognitive planning module)
        Node(
            package='vla',
            executable='cognitive_planner_node',
            name='cognitive_planner_node',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        )
    ])
```

## Testing Complete Scenarios

Here's a comprehensive test script to verify the complete VLA system:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import json

class VLASystemTester(Node):
    def __init__(self):
        super().__init__('vla_system_tester')

        # Publishers
        self.voice_command_publisher = self.create_publisher(String, 'robot_command', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)

        # Subscribers
        self.system_state_subscriber = self.create_subscription(
            String, 'vla_system_state', self.system_state_callback, 10)
        self.feedback_subscriber = self.create_subscription(
            String, 'user_feedback', self.feedback_callback, 10)

        # Test scenarios
        self.test_scenarios = [
            {
                'name': 'Simple Navigation',
                'command': 'Move forward 1 meter',
                'expected_states': ['processing', 'executing', 'completed']
            },
            {
                'name': 'Object Fetch',
                'command': 'Go to the kitchen and bring me the red cup',
                'expected_states': ['processing', 'planning', 'executing', 'completed']
            },
            {
                'name': 'Complex Task',
                'command': 'Navigate to the office, find the book on the desk, and bring it to me',
                'expected_states': ['processing', 'planning', 'executing', 'completed']
            }
        ]

        # Timer to run tests
        self.test_timer = self.create_timer(5.0, self.run_tests)
        self.test_index = 0
        self.current_state = 'idle'

        self.get_logger().info('VLA System Tester initialized')

    def system_state_callback(self, msg):
        """
        Handle system state updates
        """
        try:
            state_data = json.loads(msg.data)
            self.current_state = state_data.get('state', 'unknown')
            self.get_logger().info(f'System state: {self.current_state}')
        except json.JSONDecodeError:
            self.current_state = msg.data

    def feedback_callback(self, msg):
        """
        Handle user feedback
        """
        self.get_logger().info(f'User feedback: {msg.data}')

    def run_tests(self):
        """
        Run test scenarios
        """
        if self.test_index < len(self.test_scenarios):
            scenario = self.test_scenarios[self.test_index]
            self.get_logger().info(f'Running test: {scenario["name"]}')
            self.get_logger().info(f'Command: {scenario["command"]}')

            # Send the test command
            cmd_msg = String()
            cmd_msg.data = scenario['command']
            self.voice_command_publisher.publish(cmd_msg)

            self.test_index += 1
        else:
            self.get_logger().info('All tests completed')
            self.test_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    tester = VLASystemTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('VLA System Tester interrupted')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Optimization and Monitoring

For production use, we need to monitor system performance:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
import psutil
import time
import json

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Publishers
        self.performance_publisher = self.create_publisher(String, 'system_performance', 10)
        self.battery_subscriber = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)

        # Timer for performance monitoring
        self.monitor_timer = self.create_timer(2.0, self.performance_monitor_callback)

        self.get_logger().info('Performance Monitor initialized')

    def performance_monitor_callback(self):
        """
        Monitor system performance
        """
        # Get system metrics
        cpu_percent = psutil.cpu_percent(interval=1)
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        # Get process-specific metrics for this node
        current_process = psutil.Process()
        process_memory = current_process.memory_info().rss / 1024 / 1024  # MB

        performance_data = {
            'timestamp': time.time(),
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'disk_percent': disk_percent,
            'process_memory_mb': process_memory,
            'node_name': self.get_name()
        }

        # Publish performance data
        perf_msg = String()
        perf_msg.data = json.dumps(performance_data)
        self.performance_publisher.publish(perf_msg)

        # Log if performance is concerning
        if cpu_percent > 80 or memory_percent > 80:
            self.get_logger().warn(f'High resource usage - CPU: {cpu_percent}%, Memory: {memory_percent}%')

    def battery_callback(self, msg):
        """
        Handle battery updates
        """
        if msg.percentage < 0.2:  # Less than 20%
            self.get_logger().warn(f'Low battery: {msg.percentage*100:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    monitor = PerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Performance Monitor interrupted')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Deployment Configuration

For deployment, here's a sample configuration file:

```yaml
# config/vla_system.yaml
vla_system:
  master_node:
    qos_override:
      state_publisher:
        depth: 10
        durability: volatile
      command_publisher:
        depth: 10
        durability: volatile
    planning_timeout: 30.0
    execution_timeout: 300.0  # 5 minutes

  safety_supervisor:
    safety_distance: 0.5  # meters
    emergency_stop_timeout: 10.0  # seconds
    velocity_limits:
      linear_max: 1.0
      angular_max: 1.0

  voice_processing:
    whisper_model: "whisper-1"
    confidence_threshold: 0.7
    audio_buffer_size: 1024

  cognitive_planner:
    llm_model: "gpt-3.5-turbo"
    planning_timeout: 15.0
    max_retries: 3

  performance_monitor:
    cpu_threshold: 80.0
    memory_threshold: 80.0
    disk_threshold: 80.0

  environment:
    default_locations:
      - kitchen
      - living_room
      - office
      - bedroom
    object_categories:
      - cup
      - book
      - ball
      - bottle
```

## Best Practices for Autonomous Systems

1. **Safety First**: Always implement redundant safety systems and emergency stops
2. **Human Oversight**: Maintain human-in-the-loop capabilities for critical decisions
3. **Performance Monitoring**: Continuously monitor system resources and performance
4. **Error Recovery**: Implement graceful error handling and recovery mechanisms
5. **Testing**: Extensively test with various scenarios before deployment
6. **Logging**: Maintain comprehensive logs for debugging and analysis
7. **Security**: Protect API keys and implement secure communication
8. **Scalability**: Design for future expansion and additional capabilities

## Summary

In this capstone module, we've integrated all components of the Vision-Language-Action (VLA) system to create a fully autonomous humanoid robot. We've implemented:

- A master orchestrator node that coordinates all VLA components
- A safety supervisor system for emergency intervention
- A human-in-the-loop interface for oversight and control
- Performance monitoring for production deployment
- Comprehensive testing for end-to-end scenarios

The complete system now enables natural voice interaction with humanoid robots, cognitive planning for complex tasks, and safe autonomous execution in real-world environments.

## Integration Examples

Let's look at some complete integration examples showing how all components work together:

### Example 1: Complete Task Flow

Here's a complete example of how a user command flows through the entire VLA system:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import time

class VLAIntegrationExample(Node):
    def __init__(self):
        super().__init__('vla_integration_example')

        # Publishers for the complete VLA pipeline
        self.voice_publisher = self.create_publisher(String, 'audio_input', 10)
        self.command_publisher = self.create_publisher(String, 'robot_command', 10)
        self.high_level_command_publisher = self.create_publisher(String, 'high_level_command', 10)
        self.action_sequence_publisher = self.create_publisher(String, 'action_sequence', 10)

        # Subscribers to monitor the complete pipeline
        self.system_state_subscriber = self.create_subscription(
            String, 'vla_system_state', self.system_state_callback, 10)
        self.feedback_subscriber = self.create_subscription(
            String, 'user_feedback', self.feedback_callback, 10)

        # Timer to run integration example
        self.example_timer = self.create_timer(10.0, self.run_integration_example)

        self.step = 0
        self.get_logger().info('VLA Integration Example initialized')

    def run_integration_example(self):
        """
        Run complete integration example
        """
        examples = [
            "Move forward 1 meter",
            "Turn left 90 degrees",
            "Go to the kitchen and bring me a cup",
            "Navigate to the office"
        ]

        if self.step < len(examples):
            command = examples[self.step]
            self.get_logger().info(f'=== Step {self.step + 1}: Processing command "{command}" ===')

            # Simulate voice input (in real system, this would come from microphone)
            voice_msg = String()
            voice_msg.data = command
            self.voice_publisher.publish(voice_msg)

            self.step += 1
        else:
            self.get_logger().info('All integration examples completed')
            self.example_timer.cancel()

    def system_state_callback(self, msg):
        """
        Monitor system state during integration
        """
        try:
            state_data = json.loads(msg.data)
            state = state_data.get('state', 'unknown')
            task = state_data.get('current_task', 'none')
            self.get_logger().info(f'System state: {state}, Current task: {task}')
        except json.JSONDecodeError:
            self.get_logger().info(f'System state: {msg.data}')

    def feedback_callback(self, msg):
        """
        Handle user feedback during integration
        """
        self.get_logger().info(f'System feedback: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    integration_example = VLAIntegrationExample()

    try:
        rclpy.spin(integration_example)
    except KeyboardInterrupt:
        integration_example.get_logger().info('Integration example interrupted')
    finally:
        integration_example.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 2: Component Integration with Error Handling

This example shows how to integrate components with proper error handling:

```python
class VLAErrorHandlingExample:
    def __init__(self):
        self.speech_processor = SpeechProcessor()
        self.command_interpreter = CommandInterpreter()
        self.cognitive_planner = LLMCognitivePlanner()
        self.action_executor = ActionExecutor()
        self.safety_supervisor = SafetySupervisor()

    def process_command_with_error_handling(self, command_text):
        """
        Process command with comprehensive error handling
        """
        try:
            # Step 1: Process speech
            if self.is_speech_command(command_text):
                # This would involve actual speech processing in a real system
                processed_text = self.validate_command(command_text)
            else:
                processed_text = command_text

            # Step 2: Interpret command
            interpreted_command = self.command_interpreter.interpret(processed_text)
            if not interpreted_command:
                raise ValueError(f"Could not interpret command: {command_text}")

            # Step 3: Plan actions
            action_sequence = self.cognitive_planner.plan_complex_task(
                interpreted_command,
                self.get_environment_context()
            )
            if not action_sequence:
                raise ValueError(f"Could not plan actions for command: {interpreted_command}")

            # Step 4: Validate plan with safety supervisor
            if not self.safety_supervisor.validate_plan(action_sequence):
                raise ValueError("Safety validation failed for planned actions")

            # Step 5: Execute actions
            execution_result = self.action_executor.execute_action_sequence(action_sequence)

            # Step 6: Monitor execution
            success = self.monitor_execution(execution_result)

            return {
                'success': success,
                'command': command_text,
                'execution_result': execution_result
            }

        except Exception as e:
            self.handle_error(e, command_text)
            return {
                'success': False,
                'command': command_text,
                'error': str(e)
            }

    def validate_command(self, command):
        """
        Validate command before processing
        """
        if not command or len(command.strip()) == 0:
            raise ValueError("Empty command received")

        # Additional validation rules
        if len(command) > 200:  # Limit command length
            raise ValueError("Command too long")

        return command

    def get_environment_context(self):
        """
        Get environment context for planning
        """
        # In a real system, this would come from sensors
        return {
            'objects': ['cup', 'book', 'ball'],
            'locations': ['kitchen', 'living room', 'office'],
            'robot_state': {'x': 0.0, 'y': 0.0, 'battery': 0.85},
            'safety_zones': []
        }

    def monitor_execution(self, execution_result):
        """
        Monitor execution and handle any issues
        """
        # Monitor the execution result
        if execution_result.get('status') == 'completed':
            return True
        elif execution_result.get('status') == 'failed':
            # Try recovery
            recovery_result = self.attempt_recovery(execution_result)
            return recovery_result
        else:
            return False

    def attempt_recovery(self, failed_result):
        """
        Attempt to recover from failed execution
        """
        # Implementation of recovery strategies
        # This could include replanning, retrying, or requesting human intervention
        self.get_logger().info(f"Attempting recovery from: {failed_result}")
        return False  # Simplified for example

    def handle_error(self, error, command):
        """
        Handle errors during processing
        """
        self.get_logger().error(f"Error processing command '{command}': {error}")
        # Log error for analysis
        # Potentially trigger safety protocols
        # Report to user
        pass
```

### Example 3: Real-time Monitoring Dashboard

This example shows how to create a real-time monitoring dashboard for the VLA system:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState
import json
import time

class VLAMonitoringNode(Node):
    def __init__(self):
        super().__init__('vla_monitoring_node')

        # Subscribers for monitoring
        self.system_state_subscriber = self.create_subscription(
            String, 'vla_system_state', self.system_state_callback, 10)
        self.safety_status_subscriber = self.create_subscription(
            String, 'safety_status', self.safety_status_callback, 10)
        self.performance_subscriber = self.create_subscription(
            String, 'system_performance', self.performance_callback, 10)
        self.feedback_subscriber = self.create_subscription(
            String, 'user_feedback', self.feedback_callback, 10)
        self.emergency_stop_subscriber = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10)

        # Timer to generate periodic status reports
        self.status_timer = self.create_timer(1.0, self.generate_status_report)

        # Store system state
        self.current_state = {}
        self.safety_status = {}
        self.performance_data = {}
        self.emergency_stop_status = False

        self.get_logger().info('VLA Monitoring Node initialized')

    def system_state_callback(self, msg):
        """
        Handle system state updates
        """
        try:
            self.current_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.current_state = {'state': msg.data, 'error': 'JSON decode failed'}

    def safety_status_callback(self, msg):
        """
        Handle safety status updates
        """
        try:
            self.safety_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.safety_status = {'status': msg.data, 'error': 'JSON decode failed'}

    def performance_callback(self, msg):
        """
        Handle performance updates
        """
        try:
            self.performance_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.performance_data = {'error': 'JSON decode failed'}

    def feedback_callback(self, msg):
        """
        Handle user feedback
        """
        self.get_logger().info(f'USER FEEDBACK: {msg.data}')

    def emergency_stop_callback(self, msg):
        """
        Handle emergency stop updates
        """
        self.emergency_stop_status = msg.data
        status = 'ACTIVE' if msg.data else 'INACTIVE'
        self.get_logger().warn(f'EMERGENCY STOP: {status}')

    def generate_status_report(self):
        """
        Generate and display system status report
        """
        timestamp = time.strftime('%H:%M:%S')

        print(f"\n{'='*60}")
        print(f"VLA System Status Report - {timestamp}")
        print(f"{'='*60}")

        # System State
        state = self.current_state.get('state', 'unknown')
        current_task = self.current_state.get('current_task', 'none')
        print(f"System State: {state.upper()}")
        print(f"Current Task: {current_task}")

        # Safety Status
        safe = not self.safety_status.get('emergency_stop', False)
        safety_status = 'SAFE' if safe else 'UNSAFE (EMERGENCY STOP)'
        print(f"Safe Operation: {safety_status}")

        # Performance
        cpu_usage = self.performance_data.get('cpu_percent', 'N/A')
        mem_usage = self.performance_data.get('memory_percent', 'N/A')
        print(f"CPU Usage: {cpu_usage}%")
        print(f"Memory Usage: {mem_usage}%")

        # Emergency Stop
        emg_status = 'ACTIVE' if self.emergency_stop_status else 'INACTIVE'
        print(f"Emergency Stop: {emg_status}")

        print(f"{'='*60}")

def main(args=None):
    rclpy.init(args=args)
    monitoring_node = VLAMonitoringNode()

    try:
        rclpy.spin(monitoring_node)
    except KeyboardInterrupt:
        monitoring_node.get_logger().info('Monitoring node interrupted')
    finally:
        monitoring_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

- Deploy the system on actual humanoid hardware
- Integrate with advanced perception systems (computer vision, SLAM)
- Add multi-modal capabilities (vision-language-action)
- Implement learning from demonstration capabilities
- Extend to multi-robot coordination scenarios
- Deploy monitoring dashboard for real-time system oversight