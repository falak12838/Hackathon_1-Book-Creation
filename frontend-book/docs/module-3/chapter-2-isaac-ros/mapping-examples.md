# Mapping and Localization Examples in Isaac ROS

## Overview

This section provides practical examples of mapping and localization using Isaac ROS VSLAM. These examples demonstrate how to create, update, and use maps for humanoid robot navigation in various scenarios. The examples include both theoretical concepts and practical implementation details.

## Basic Mapping Example

### Creating a Simple Map

The simplest mapping scenario involves moving the robot through an environment to build a map:

```python
# Basic mapping example
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np

class BasicMapper(Node):
    def __init__(self):
        super().__init__('basic_mapper')

        # Initialize components
        self.bridge = CvBridge()
        self.map = None
        self.poses = []
        self.is_mapping = False

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/visual_slam/pose',
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

    def image_callback(self, msg):
        """Process incoming image for mapping"""
        if not self.is_mapping:
            return

        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process with Isaac ROS VSLAM (conceptual)
        pose_estimate = self.process_with_vslam(cv_image)

        if pose_estimate is not None:
            self.poses.append(pose_estimate)
            self.update_map(pose_estimate)

    def update_map(self, pose_estimate):
        """Update the occupancy grid map"""
        # Convert VSLAM pose to occupancy grid
        if self.map is None:
            self.initialize_map(pose_estimate)

        # Update map with current pose information
        self.integrate_pose_into_map(pose_estimate)

        # Publish updated map
        occupancy_msg = self.create_occupancy_grid_msg()
        self.map_pub.publish(occupancy_msg)

    def initialize_map(self, initial_pose):
        """Initialize the occupancy grid map"""
        # Create a 20x20 meter map with 5cm resolution
        self.map = {
            'resolution': 0.05,  # 5cm per cell
            'width': 400,        # 20m / 0.05m = 400 cells
            'height': 400,       # 20m / 0.05m = 400 cells
            'origin_x': initial_pose.position.x - 10.0,  # Center map around robot
            'origin_y': initial_pose.position.y - 10.0,
            'data': [-1] * (400 * 400)  # -1 = unknown, 0 = free, 100 = occupied
        }
```

### Mapping with Loop Closure

A more sophisticated example with loop closure detection:

```python
# Advanced mapping with loop closure
class AdvancedMapper(BasicMapper):
    def __init__(self):
        super().__init__()
        self.keyframes = []
        self.loop_closure_detector = LoopClosureDetector()
        self.graph_optimizer = GraphOptimizer()

    def should_create_keyframe(self, current_pose):
        """Determine if we should create a new keyframe"""
        if len(self.keyframes) == 0:
            return True

        last_keyframe = self.keyframes[-1]

        # Create keyframe if moved significantly
        position_diff = np.sqrt(
            (current_pose.position.x - last_keyframe.position.x)**2 +
            (current_pose.position.y - last_keyframe.position.y)**2
        )

        # Create if moved more than 1 meter or rotated more than 15 degrees
        if position_diff > 1.0:
            return True

        return False

    def process_loop_closure(self):
        """Detect and process loop closures"""
        for i, keyframe1 in enumerate(self.keyframes):
            for j, keyframe2 in enumerate(self.keyframes[i+10:], i+10):  # Check with frames at least 10 apart
                if self.loop_closure_detector.is_loop_closure(keyframe1, keyframe2):
                    # Optimize the pose graph
                    self.graph_optimizer.add_constraint(i, j, self.calculate_relative_pose(keyframe1, keyframe2))
                    self.graph_optimizer.optimize()
                    break
```

## Localization Examples

### Initial Pose Estimation

Setting up localization in a known map:

```python
# Localization in existing map
class Localizer(Node):
    def __init__(self):
        super().__init__('vslam_localizer')

        self.map = None
        self.current_pose = None
        self.is_localized = False

        # Subscribe to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_rect_color',
            self.image_callback,
            10
        )

    def map_callback(self, msg):
        """Receive map for localization"""
        self.map = {
            'data': list(msg.data),
            'info': msg.info  # Contains resolution, origin, etc.
        }

    def image_callback(self, msg):
        """Process image for localization"""
        if self.map is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.is_localized:
            # Try to relocalize using the map
            self.current_pose = self.relocalize(cv_image)
            if self.current_pose is not None:
                self.is_localized = True
        else:
            # Track using VSLAM
            self.current_pose = self.track_pose(cv_image, self.current_pose)

    def relocalize(self, image):
        """Relocalize the robot in the map"""
        # Use appearance-based relocalization
        candidate_poses = self.find_candidate_poses(image)

        for pose in candidate_poses:
            # Verify pose by checking consistency with map
            if self.verify_pose_consistency(pose, image):
                return pose

        return None
```

## Humanoid-Specific Mapping Examples

### Mapping with Head Motion Compensation

For humanoid robots, head motion can affect mapping quality:

```python
# Humanoid-specific mapping with head motion compensation
class HumanoidMapper(Localizer):
    def __init__(self):
        super().__init__()
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.head_joints = ['neck_pan', 'neck_tilt']
        self.head_compensation = True

    def joint_state_callback(self, msg):
        """Update head joint positions"""
        for name, position in zip(msg.name, msg.position):
            if name in self.head_joints:
                setattr(self, f'{name}_pos', position)

    def compensate_for_head_motion(self, image, camera_pose):
        """Compensate for head motion when building map"""
        if not self.head_compensation:
            return camera_pose

        # Calculate head motion transform
        head_transform = self.calculate_head_transform()

        # Apply inverse transform to compensate for head motion
        compensated_pose = self.apply_transform_inverse(camera_pose, head_transform)

        return compensated_pose

    def calculate_head_transform(self):
        """Calculate transform due to head joint changes"""
        # Calculate transform based on neck joint positions
        # This is a simplified example
        pan_angle = getattr(self, 'neck_pan_pos', 0.0)
        tilt_angle = getattr(self, 'neck_tilt_pos', 0.0)

        # Create rotation matrix for head motion
        # (Simplified - real implementation would be more complex)
        transform = np.eye(4)
        transform[0, 0] = np.cos(pan_angle)
        transform[0, 2] = np.sin(pan_angle)
        transform[2, 0] = -np.sin(pan_angle)
        transform[2, 2] = np.cos(pan_angle)

        return transform
```

### Multi-Session Mapping

Building maps across multiple sessions:

```python
# Multi-session mapping
class MultiSessionMapper(HumanoidMapper):
    def __init__(self):
        super().__init__()
        self.map_sessions = []
        self.session_maps = []

    def start_new_session(self, session_name):
        """Start a new mapping session"""
        session = {
            'name': session_name,
            'start_time': self.get_clock().now(),
            'poses': [],
            'keyframes': []
        }
        self.map_sessions.append(session)

    def save_session_map(self, session_name):
        """Save the map from a specific session"""
        session = self.get_session_by_name(session_name)
        if session:
            # Create map from session data
            session_map = self.create_map_from_poses(session['poses'])
            self.session_maps.append(session_map)

    def merge_session_maps(self):
        """Merge maps from multiple sessions"""
        if len(self.session_maps) < 2:
            return self.session_maps[0] if self.session_maps else None

        # Use loop closure to align maps from different sessions
        merged_map = self.align_and_merge_maps(self.session_maps)

        # Optimize the merged map
        optimized_map = self.optimize_merged_map(merged_map)

        return optimized_map
```

## Real-World Mapping Scenarios

### Indoor Mapping Example

Mapping an indoor environment:

```python
# Indoor mapping example
class IndoorMapper(MultiSessionMapper):
    def __init__(self):
        super().__init__()
        self.environment_type = 'indoor'
        self.obstacle_detector = ObstacleDetector()

    def explore_indoor_environment(self):
        """Systematic exploration for indoor mapping"""

        # Strategy: move to corners and along walls
        exploration_path = self.plan_exploration_path()

        for waypoint in exploration_path:
            # Navigate to waypoint
            self.navigate_to(waypoint)

            # Perform 360-degree scan at waypoint
            self.perform_360_scan()

            # Check for loop closure opportunities
            self.check_for_loop_closures()

    def plan_exploration_path(self):
        """Plan systematic exploration path"""
        # For indoor environments, plan path along walls and to corners
        # This is a simplified example
        path = []

        # Move to current room boundaries
        current_bounds = self.get_current_map_bounds()
        for corner in current_bounds['corners']:
            path.append({
                'position': corner,
                'orientation': self.calculate_favorable_orientation(corner)
            })

        return path

    def perform_360_scan(self):
        """Perform 360-degree scan to improve mapping"""
        initial_yaw = self.current_pose.orientation.z  # Simplified

        # Rotate to capture 360 degrees
        for angle in np.linspace(0, 2*np.pi, 12):  # 12 positions for 360 degrees
            target_yaw = initial_yaw + angle
            self.rotate_to_yaw(target_yaw)

            # Process image at this orientation
            # (Assuming camera captures images during rotation)
            self.process_current_view()
```

### Dynamic Environment Mapping

Mapping environments with moving objects:

```python
# Dynamic environment mapping
class DynamicMapper(IndoorMapper):
    def __init__(self):
        super().__init__()
        self.dynamic_object_detector = DynamicObjectDetector()
        self.static_map = None
        self.dynamic_objects = []

    def detect_dynamic_objects(self, current_image, previous_image):
        """Detect and handle dynamic objects"""
        # Find differences between current and previous frames
        motion_mask = self.compute_motion_mask(current_image, previous_image)

        # Extract moving objects
        dynamic_regions = self.extract_regions(motion_mask)

        for region in dynamic_regions:
            # Classify if this is a dynamic object
            if self.is_dynamic_object(region):
                # Mark this area as dynamic in the map
                self.mark_as_dynamic(region)

    def update_static_map(self):
        """Update map while ignoring dynamic objects"""
        # Only incorporate static elements into the map
        static_features = self.filter_static_features(self.current_features)

        # Update map with static features only
        self.integrate_static_features(static_features)

    def predict_dynamic_object_motion(self, object_id):
        """Predict motion of dynamic objects for safe navigation"""
        if object_id in self.dynamic_objects:
            # Use tracking history to predict future position
            predicted_position = self.predict_object_position(
                self.dynamic_objects[object_id]['history']
            )
            return predicted_position

        return None
```

## Isaac ROS Integration Examples

### Launch File Configuration

Example launch file for VSLAM mapping:

```xml
<!-- vslam_mapping.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('namespace', default_value='robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Isaac ROS Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            namespace=namespace,
            parameters=[{
                'enable_rectified_pose': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_odom_tf': True,
                'enable_observations_view': True,
                'enable_slam_visualization': True,
                'enable_localization': False,  # Enable mapping mode
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('/visual_slam/image', '/camera/image_rect_color'),
                ('/visual_slam/camera_info', '/camera/camera_info'),
            ]
        ),

        # Isaac ROS Image Proc for rectification
        Node(
            package='isaac_ros_image_proc',
            executable='image_proc',
            name='image_proc',
            namespace=namespace,
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
                ('image_rect_color', '/camera/image_rect_color'),
            ]
        ),

        # Robot state publisher for transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        )
    ])
```

### ROS 2 Service Integration

Example of using ROS 2 services for mapping control:

```python
# Mapping control service client
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose

class MappingController(Node):
    def __init__(self):
        super().__init__('mapping_controller')

        # Create service clients
        self.start_mapping_cli = self.create_client(
            Trigger,
            '/visual_slam/start_mapping'
        )
        self.stop_mapping_cli = self.create_client(
            Trigger,
            '/visual_slam/stop_mapping'
        )
        self.save_map_cli = self.create_client(
            Trigger,
            '/visual_slam/save_map'
        )

    def start_mapping(self):
        """Start the mapping process"""
        while not self.start_mapping_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Mapping service not available, waiting...')

        future = self.start_mapping_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def stop_mapping(self):
        """Stop the mapping process"""
        while not self.stop_mapping_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop mapping service not available, waiting...')

        future = self.stop_mapping_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def save_map(self, filename=None):
        """Save the current map"""
        while not self.save_map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Save map service not available, waiting...')

        future = self.save_map_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        return future.result()
```

## Performance Optimization Examples

### Memory-Efficient Mapping

Optimizing mapping for resource-constrained platforms:

```python
# Memory-efficient mapping
class EfficientMapper(DynamicMapper):
    def __init__(self):
        super().__init__()
        self.max_map_size = 100000  # Maximum number of map points
        self.map_point_buffer = []
        self.keyframe_buffer = []

    def optimize_map_storage(self):
        """Optimize map storage by removing redundant points"""
        # Remove map points that haven't been observed recently
        recent_threshold = self.get_clock().now() - rclpy.time.Duration(seconds=300)  # 5 minutes

        self.map_point_buffer = [
            point for point in self.map_point_buffer
            if point.last_observed > recent_threshold
        ]

        # Limit total number of map points
        if len(self.map_point_buffer) > self.max_map_size:
            # Remove oldest points
            self.map_point_buffer = self.map_point_buffer[-self.max_map_size:]

    def selective_keyframe_creation(self):
        """Create keyframes selectively to save memory"""
        current_pose = self.get_current_pose()

        # Only create keyframe if significant change or important location
        if self.is_significant_change(current_pose) or self.is_important_location(current_pose):
            self.create_keyframe(current_pose)
        else:
            # Just store pose without full keyframe
            self.poses.append(current_pose)
```

## Best Practices and Tips

### Mapping Quality Assurance

```python
# Mapping quality checks
class QualityMapper(EfficientMapper):
    def __init__(self):
        super().__init__()
        self.quality_metrics = {
            'feature_density': 0,
            'tracking_stability': 0,
            'map_coverage': 0
        }

    def check_mapping_quality(self):
        """Check and report mapping quality metrics"""
        metrics = {}

        # Feature density check
        current_features = self.get_current_feature_count()
        expected_features = self.estimate_expected_features()
        metrics['feature_density'] = current_features / expected_features

        # Tracking stability
        tracking_quality = self.get_tracking_quality_score()
        metrics['tracking_stability'] = tracking_quality

        # Map coverage
        coverage_ratio = self.calculate_map_coverage()
        metrics['map_coverage'] = coverage_ratio

        # Report if quality is below threshold
        if any(value < 0.5 for value in metrics.values()):
            self.get_logger().warn(f'Low mapping quality: {metrics}')

        return metrics
```

## Troubleshooting Common Issues

### Poor Mapping Quality

```python
# Diagnose mapping issues
class MappingDiagnostics:
    def __init__(self):
        self.diagnostics = {
            'lighting_issues': False,
            'texture_issues': False,
            'motion_blur': False,
            'feature_richness': 0
        }

    def diagnose_mapping_issues(self, image, pose_data):
        """Diagnose common mapping issues"""
        issues = []

        # Check for poor lighting
        if self.is_low_lighting(image):
            issues.append("Poor lighting conditions - add more light or use illumination")

        # Check for lack of texture
        if self.is_low_texture(image):
            issues.append("Low texture environment - move to area with more features")

        # Check for motion blur
        if self.has_motion_blur(image):
            issues.append("Motion blur detected - move more slowly or use faster shutter")

        # Check feature count
        feature_count = self.count_features(image)
        if feature_count < 50:  # Threshold may vary
            issues.append(f"Insufficient features detected: {feature_count}")

        return issues
```

## Next Steps

With mapping and localization examples understood, you're ready to explore:

- Performance optimization techniques for VSLAM
- Integration with Isaac Sim for simulation and validation
- Navigation system integration with the created maps
- Advanced topics like semantic mapping

Continue to the next section to learn about VSLAM performance optimization.