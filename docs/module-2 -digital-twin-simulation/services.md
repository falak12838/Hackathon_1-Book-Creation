---
title: ROS 2 Services for Humanoid Robotics
sidebar_position: 6
---

# ROS 2 Services for Humanoid Robotics

## Understanding Services

Services in ROS 2 provide synchronous, request-response communication between nodes. Unlike topics which provide continuous data streams, services are used for discrete actions that have a clear request and response pattern. This makes them ideal for operations that need to return a result or confirm completion.

In humanoid robotics, services are commonly used for:
- Action execution (e.g., "move arm to position", "grasp object")
- System configuration (e.g., "set joint limits", "update parameters")
- Status queries (e.g., "get battery level", "check system status")
- Calibration procedures (e.g., "calibrate sensors", "zero joints")

## Service Architecture in Humanoid Robots

Services follow a client-server pattern:
- **Service Server**: Implements the service and responds to requests
- **Service Client**: Makes requests to the service and receives responses

This synchronous communication is appropriate for operations that need to complete before the calling node can proceed.

## Creating Services in rclpy

Here's an example of a service for humanoid robot joint control:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from sensor_msgs.msg import JointState
import time

class JointControlService(Node):
    def __init__(self):
        super().__init__('joint_control_service')

        # Create service for enabling/disabling joint control
        self.srv = self.create_service(
            SetBool,
            'enable_joint_control',
            self.enable_control_callback
        )

        # Publisher for joint states
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_control_enabled = False

        self.get_logger().info('Joint Control Service initialized')

    def enable_control_callback(self, request, response):
        # Enable or disable joint control based on request
        self.joint_control_enabled = request.data

        if self.joint_control_enabled:
            response.success = True
            response.message = 'Joint control enabled'
            self.get_logger().info('Joint control enabled')
        else:
            response.success = True
            response.message = 'Joint control disabled'
            self.get_logger().info('Joint control disabled')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = JointControlService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Client Example

Here's how to create a client that calls the service:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class JointControlClient(Node):
    def __init__(self):
        super().__init__('joint_control_client')
        self.cli = self.create_client(SetBool, 'enable_joint_control')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = SetBool.Request()

    def send_request(self, enable):
        self.req.data = enable
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)

    client = JointControlClient()
    future = client.send_request(True)  # Enable joint control

    while rclpy.ok():
        rclpy.spin_once(client)
        if future.done():
            try:
                response = future.result()
                client.get_logger().info(
                    f'Result: {response.success}, {response.message}'
                )
            except Exception as e:
                client.get_logger().info(f'Service call failed: {e}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Service Example: Humanoid Movement Service

Here's a more complex service for humanoid-specific movement:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from geometry_msgs.msg import Pose
from std_msgs.msg import String

class HumanoidMovementService(Node):
    def __init__(self):
        super().__init__('humanoid_movement_service')

        # Service for executing predefined movements
        self.move_srv = self.create_service(
            Trigger,
            'execute_movement',
            self.execute_movement_callback
        )

        # Service for setting target poses
        self.pose_srv = self.create_service(
            SetPose,  # This would be a custom message type
            'set_target_pose',
            self.set_target_pose_callback
        )

        # Publisher for movement status
        self.status_publisher = self.create_publisher(String, 'movement_status', 10)

        self.current_movement = None
        self.is_moving = False

        self.get_logger().info('Humanoid Movement Service initialized')

    def execute_movement_callback(self, request, response):
        if self.is_moving:
            response.success = False
            response.message = 'Robot is currently executing a movement'
            return response

        # In a real implementation, this would execute a movement pattern
        movement_name = request.message  # Using message field as movement name

        self.get_logger().info(f'Executing movement: {movement_name}')

        # Execute the movement (simplified)
        success = self.execute_movement_sequence(movement_name)

        if success:
            response.success = True
            response.message = f'Movement {movement_name} completed successfully'
        else:
            response.success = False
            response.message = f'Movement {movement_name} failed'

        return response

    def execute_movement_sequence(self, movement_name):
        # Placeholder for actual movement execution
        # This would contain the logic to execute specific movement patterns
        self.is_moving = True

        # Simulate movement execution
        time.sleep(2)  # Simulate time for movement

        self.is_moving = False
        return True  # Assume success for this example

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidMovementService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Service Messages

For more complex humanoid-specific services, you might need custom message types. Here's an example of a custom service definition:

```
# In a file: srv/HumanoidMove.srv
# Request
string movement_type  # 'walk', 'turn', 'step', 'balance', etc.
float64[] target_position  # Target joint positions
float64 duration  # Expected duration of movement

---
# Response
bool success
string message
float64 actual_duration  # Actual time taken
string[] error_details  # Details if movement failed
```

## Best Practices for Service Usage in Humanoid Robotics

1. **Appropriate Use Cases**: Use services for operations that need a clear response or confirmation
2. **Timeout Handling**: Always implement timeout handling in service clients
3. **Error Handling**: Return meaningful error messages in service responses
4. **Synchronous Nature**: Be aware that services block the calling thread until completion
5. **Security**: Consider authentication and authorization for critical services
6. **Service Discovery**: Use appropriate service naming conventions for easy discovery
7. **Asynchronous Clients**: Use asynchronous service calls when possible to avoid blocking
8. **Resource Management**: Ensure services properly manage resources and handle cleanup