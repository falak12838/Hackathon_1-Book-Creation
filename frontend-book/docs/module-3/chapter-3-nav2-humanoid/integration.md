# Integration with Isaac Sim and Isaac ROS for Humanoid Navigation

## Overview

The integration of Nav2 with Isaac Sim and Isaac ROS creates a comprehensive simulation-to-reality pipeline for humanoid robot navigation. This integration allows for realistic simulation of navigation behaviors, validation of path planning algorithms, and testing of obstacle avoidance strategies before deployment on physical robots.

## Integration Architecture

### High-Level Integration Diagram

The integration follows a layered architecture:

```
Physical Robot (Optional)
    ↓ (ROS 2 Messages)
Isaac ROS Perception Stack
    ↓ (Processed Data)
Isaac ROS Navigation Interface
    ↓ (Navigation Commands)
Nav2 Navigation Stack
    ↓ (Path Planning & Control)
Isaac Sim Simulation Environment
    ↓ (Sensor Simulation)
Isaac ROS Sensor Simulation
```

### Component Integration Points

#### 1. Isaac Sim → Isaac ROS Bridge
- **USD Scene Graph**: Real-time scene synchronization
- **Sensor Simulation**: Realistic sensor data generation
- **Robot Dynamics**: Accurate physics simulation
- **ROS 2 Bridge**: Seamless message passing

#### 2. Isaac ROS → Nav2 Interface
- **Sensor Data**: Camera, LiDAR, IMU data for navigation
- **Odometry**: Robot state estimation
- **Maps**: Static and dynamic map information
- **Transforms**: TF tree for spatial relationships

## Isaac Sim Setup for Navigation

### Navigation-Specific Environment Configuration

```python
# Isaac Sim navigation environment setup
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.carb import carb_settings_apply
import carb

class NavigationEnvironment:
    def __init__(self, world):
        self.world = world
        self.navigation_map = None
        self.obstacle_manager = ObstacleManager()

    def setup_navigation_environment(self, env_config):
        """Setup environment for navigation testing"""

        # Create navigation-specific elements
        self.create_navigation_boundary(env_config)
        self.setup_navigation_landmarks(env_config)
        self.configure_navigation_lighting(env_config)
        self.setup_dynamic_obstacles(env_config)

    def create_navigation_boundary(self, env_config):
        """Create boundaries for navigation testing"""
        boundary_config = env_config.get('boundary', {})

        # Create perimeter walls
        for wall_info in boundary_config.get('walls', []):
            wall_prim = create_prim(
                prim_path=f"/World/Boundary/{wall_info['name']}",
                prim_type="Capsule",
                position=wall_info['position'],
                scale=wall_info['scale'],
                orientation=wall_info.get('orientation', [0, 0, 0, 1])
            )

        # Create floor plane
        floor_prim = create_prim(
            prim_path="/World/groundPlane",
            prim_type="Plane",
            position=[0, 0, 0],
            scale=[env_config['width'], 1, env_config['depth']],
            orientation=[0.707, 0, 0, 0.707]
        )

    def setup_navigation_landmarks(self, env_config):
        """Setup navigation landmarks for localization"""
        landmarks = env_config.get('landmarks', [])

        for i, landmark in enumerate(landmarks):
            landmark_prim = create_prim(
                prim_path=f"/World/Landmarks/Landmark_{i}",
                prim_type=landmark['type'],
                position=landmark['position'],
                scale=landmark['scale'],
                orientation=landmark.get('orientation', [0, 0, 0, 1])
            )

            # Apply distinctive material for easy detection
            self.apply_navigation_material(landmark_prim, landmark['color'])

    def configure_navigation_lighting(self, env_config):
        """Configure lighting for navigation testing"""
        lighting_config = env_config.get('lighting', {})

        # Add multiple light sources to reduce shadows
        lights = lighting_config.get('lights', [
            {'type': 'DistantLight', 'position': [0, 0, 10], 'intensity': 3000},
            {'type': 'SphereLight', 'position': [5, 5, 5], 'intensity': 1000},
            {'type': 'SphereLight', 'position': [-5, -5, 5], 'intensity': 1000}
        ])

        for i, light_config in enumerate(lights):
            light_prim = create_prim(
                prim_path=f"/World/Lights/NavLight_{i}",
                prim_type=light_config['type'],
                position=light_config['position'],
                attributes={'intensity': light_config['intensity']}
            )

    def setup_dynamic_obstacles(self, env_config):
        """Setup dynamic obstacles for navigation testing"""
        dynamic_obstacles = env_config.get('dynamic_obstacles', [])

        for i, obs_config in enumerate(dynamic_obstacles):
            obstacle = self.obstacle_manager.create_dynamic_obstacle(
                prim_path=f"/World/DynamicObstacles/Obstacle_{i}",
                config=obs_config
            )
```

### Humanoid Robot Configuration in Isaac Sim

```python
# Humanoid robot setup for navigation
class HumanoidNavigationRobot:
    def __init__(self, world, robot_config):
        self.world = world
        self.robot_config = robot_config
        self.sensors = {}
        self.navigation_ready = False

    def setup_navigation_robot(self, robot_usd_path, position, orientation):
        """Setup humanoid robot for navigation in Isaac Sim"""

        # Add humanoid robot to stage
        robot_prim_path = "/World/HumanoidRobot"
        add_reference_to_stage(
            usd_path=robot_usd_path,
            prim_path=robot_prim_path
        )

        # Set initial position and orientation
        self.world.scene.add_ground_plane(
            "ground_plane",
            static_friction=0.6,
            dynamic_friction=0.6,
            restitution=0.1
        )

        # Setup navigation-specific sensors
        self.setup_navigation_sensors(robot_prim_path)

        # Configure robot for navigation
        self.configure_robot_for_navigation()

        self.navigation_ready = True
        return robot_prim_path

    def setup_navigation_sensors(self, robot_prim_path):
        """Setup sensors required for navigation"""

        # RGB-D camera for visual navigation
        self.sensors['camera'] = self.setup_navigation_camera(
            f"{robot_prim_path}/Head_Camera",
            position=[0.1, 0, 0.2],  # Forward and up from head
            orientation=[0.707, 0, 0, 0.707]  # Looking forward
        )

        # LiDAR for 3D mapping and obstacle detection
        self.sensors['lidar'] = self.setup_navigation_lidar(
            f"{robot_prim_path}/LiDAR",
            position=[0.05, 0, 0.3],  # On top of head
            orientation=[1, 0, 0, 0]   # Looking forward
        )

        # IMU for balance and motion detection
        self.sensors['imu'] = self.setup_navigation_imu(
            f"{robot_prim_path}/Torso_IMU",
            position=[0, 0, 0.1],  # In torso
            orientation=[1, 0, 0, 0]
        )

    def setup_navigation_camera(self, prim_path, position, orientation):
        """Setup RGB-D camera for navigation"""
        from omni.isaac.sensor import Camera

        camera = Camera(
            prim_path=prim_path,
            frequency=30,  # 30 Hz for navigation
            resolution=(1280, 720),
            position=position,
            orientation=orientation
        )

        # Configure for navigation use
        camera.config_intrinsic_matrix(
            focal_length_x=640.0,
            focal_length_y=640.0,
            principal_point_x=640.0,
            principal_point_y=360.0
        )

        return camera

    def setup_navigation_lidar(self, prim_path, position, orientation):
        """Setup LiDAR for navigation"""
        from omni.isaac.range_sensor import _range_sensor

        lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

        lidar_config = {
            "lidarPrim": prim_path,
            "sensorPeriod": 0.1,  # 10 Hz
            "samplesPerScan": 1080,  # High resolution
            "rotationFrequency": 10,
            "horizontalOpeningAngle": 6.2832,  # 360 degrees
            "verticalOpeningAngle": 0.5236,  # 30 degrees
            "maxRange": 20.0,
            "minRange": 0.1,
        }

        lidar_path = prim_path
        lidar_interface.create_lidar_sensor(
            lidar_path,
            translation=position,
            orientation=orientation,
            config=lidar_config
        )

        return lidar_path

    def setup_navigation_imu(self, prim_path, position, orientation):
        """Setup IMU for navigation"""
        from omni.isaac.core.sensors import ImuSensor

        imu = ImuSensor(
            prim_path=prim_path,
            position=position,
            orientation=orientation,
            frequency=100  # High frequency for balance
        )

        return imu

    def configure_robot_for_navigation(self):
        """Configure robot for navigation tasks"""
        # Set up articulation controller
        self.articulation_controller = self.world.scene.get_articulation_controller(
            name="HumanoidRobot"
        )

        # Configure joint limits for stable walking
        self.configure_navigation_joint_limits()

        # Set up balance control
        self.balance_controller = BalanceController()
```

## Isaac ROS Integration

### Isaac ROS Navigation Pipeline

```python
# Isaac ROS navigation pipeline
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import TransformBroadcaster
import tf_transformations

class IsaacROSNavigationInterface(Node):
    def __init__(self):
        super().__init__('isaac_ros_navigation_interface')

        # Initialize Isaac ROS components
        self.setup_isaac_ros_pipeline()

        # Create publishers for navigation
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.camera_publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Create subscribers for navigation commands
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.update_navigation_data)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def setup_isaac_ros_pipeline(self):
        """Setup Isaac ROS pipeline for navigation"""
        # Import Isaac ROS packages
        try:
            from isaac_ros_visual_slam import VisualSlamNode
            from isaac_ros_pointcloud_utils import PointCloudConverters
            from isaac_ros_image_proc import ImageProc

            # Initialize Isaac ROS nodes
            self.visual_slam = VisualSlamNode()
            self.image_proc = ImageProc()
            self.pointcloud_converter = PointCloudConverters()

        except ImportError as e:
            self.get_logger().warn(f"Isaac ROS packages not available: {e}")

    def cmd_vel_callback(self, msg):
        """Handle navigation velocity commands from Nav2"""
        # Convert Twist command to Isaac Sim robot control
        linear_vel = msg.linear.x  # Forward/backward
        angular_vel = msg.angular.z  # Turn rate

        # Send command to Isaac Sim robot
        self.send_command_to_robot(linear_vel, angular_vel)

    def update_navigation_data(self):
        """Update navigation data from Isaac Sim"""
        # Get current robot state from Isaac Sim
        robot_state = self.get_robot_state_from_isaac_sim()

        if robot_state:
            # Publish odometry
            self.publish_odometry(robot_state)

            # Publish sensor data
            self.publish_sensor_data(robot_state)

            # Broadcast transforms
            self.broadcast_transforms(robot_state)

    def get_robot_state_from_isaac_sim(self):
        """Get robot state from Isaac Sim"""
        # This would interface with Isaac Sim to get current state
        # In practice, this would use Isaac Sim's API to get robot pose, velocities, etc.
        pass

    def publish_odometry(self, robot_state):
        """Publish odometry data for navigation"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose
        odom_msg.pose.pose.position.x = robot_state['position']['x']
        odom_msg.pose.pose.position.y = robot_state['position']['y']
        odom_msg.pose.pose.position.z = robot_state['position']['z']

        # Set orientation (convert from Isaac Sim format to ROS format)
        quat = self.convert_rotation_to_quaternion(robot_state['rotation'])
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set twist (velocities)
        odom_msg.twist.twist.linear.x = robot_state['linear_velocity']['x']
        odom_msg.twist.twist.linear.y = robot_state['linear_velocity']['y']
        odom_msg.twist.twist.linear.z = robot_state['linear_velocity']['z']

        odom_msg.twist.twist.angular.x = robot_state['angular_velocity']['x']
        odom_msg.twist.twist.angular.y = robot_state['angular_velocity']['y']
        odom_msg.twist.twist.angular.z = robot_state['angular_velocity']['z']

        self.odom_publisher.publish(odom_msg)

    def publish_sensor_data(self, robot_state):
        """Publish sensor data for navigation"""
        # Publish laser scan from LiDAR
        if 'lidar_data' in robot_state:
            scan_msg = self.create_laser_scan_msg(robot_state['lidar_data'])
            self.scan_publisher.publish(scan_msg)

        # Publish IMU data
        if 'imu_data' in robot_state:
            imu_msg = self.create_imu_msg(robot_state['imu_data'])
            self.imu_publisher.publish(imu_msg)

        # Publish camera data
        if 'camera_data' in robot_state:
            camera_msg = self.create_camera_msg(robot_state['camera_data'])
            self.camera_publisher.publish(camera_msg)

    def broadcast_transforms(self, robot_state):
        """Broadcast necessary transforms for navigation"""
        # Broadcast base_link to camera
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.1  # Camera offset from base
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2
        t.transform.rotation.w = 1.0  # No rotation initially

        self.tf_broadcaster.sendTransform(t)

        # Broadcast other necessary transforms
        # (base_link to LiDAR, IMU, etc.)
```

## Nav2 Configuration for Isaac Integration

### Launch File Configuration

```xml
<!-- Isaac Sim and Isaac ROS integration launch file -->
<launch>
  <!-- Arguments -->
  <arg name="use_sim_time" default="true"/>
  <arg name="map" default="turtlebot3_world.yaml"/>
  <arg name="params_file" default="$(find-pkg-share nav2_bringup)/params/nav2_params.yaml"/>
  <arg name="autostart" default="true"/>
  <arg name="use_composition" default="True"/>
  <arg name="container_name" default="nav2_container"/>
  <arg name="container_name_full" default="$(var container_name)_$(var use_composition)"/>

  <!-- Set use_sim_time parameter for all nodes -->
  <group>
    <param name="use_sim_time" value="$(var use_sim_time)"/>

    <!-- Isaac ROS Bridge -->
    <node pkg="omni.isaac.ros_bridge" exec="ros_bridge" name="isaac_ros_bridge">
      <param name="ros_bridge_port" value="8888"/>
    </node>

    <!-- Isaac ROS Visual SLAM -->
    <node pkg="isaac_ros_visual_slam" exec="visual_slam_node" name="visual_slam">
      <param name="enable_rectified_pose" value="true"/>
      <param name="map_frame" value="map"/>
      <param name="odom_frame" value="odom"/>
      <param name="base_frame" value="base_link"/>
      <param name="publish_odom_tf" value="true"/>
      <param name="enable_observations_view" value="true"/>
      <param name="enable_slam_visualization" value="true"/>
      <remap from="image" to="/camera/image_rect_color"/>
      <remap from="camera_info" to="/camera/camera_info"/>
      <remap from="imu" to="/imu/data"/>
    </node>

    <!-- Isaac ROS Image Proc -->
    <node pkg="isaac_ros_image_proc" exec="image_proc" name="image_proc">
      <remap from="image_raw" to="/camera/image_raw"/>
      <remap from="camera_info" to="/camera/camera_info"/>
      <remap from="image_rect_color" to="/camera/image_rect_color"/>
    </node>

    <!-- Map Server -->
    <node pkg="nav2_map_server" exec="map_server" name="map_server">
      <param name="yaml_filename" value="$(var map)"/>
      <param name="topic_name" value="map"/>
      <param name="frame_id" value="map"/>
      <param name="output_format" value="nav_msgs/OccupancyGrid"/>
      <param name="use_sim_time" value="$(var use_sim_time)"/>
    </node>

    <!-- Local Planner for Humanoid -->
    <node pkg="nav2_dwb_controller" exec="dwb_controller" name="dwb_controller">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="min_vel_x" value="0.05"/>      <!-- Minimum forward speed for stable walking -->
      <param name="max_vel_x" value="0.3"/>       <!-- Maximum forward speed -->
      <param name="min_vel_y" value="-0.1"/>     <!-- Minimum lateral speed -->
      <param name="max_vel_y" value="0.1"/>      <!-- Maximum lateral speed -->
      <param name="max_vel_theta" value="0.5"/>  <!-- Maximum angular velocity -->
      <param name="acc_lim_x" value="0.3"/>      <!-- Acceleration limits for stability -->
      <param name="acc_lim_theta" value="0.3"/>
      <param name="decel_lim_x" value="-0.3"/>
      <param name="decel_lim_theta" value="-0.3"/>
    </node>

    <!-- Planner Server -->
    <node pkg="nav2_navfn_planner" exec="navfn_planner" name="planner_server">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="planner_plugin_types" value="nav2_navfn_planner/NavfnPlanner"/>
      <param name="planner_plugin_names" value="GridBased"/>
      <param name="GridBased.movement_cost" value="0.5"/>  <!-- Higher cost for careful planning -->
    </node>

    <!-- Controller Server -->
    <node pkg="nav2_controller" exec="controller_server" name="controller_server">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="controller_frequency" value="20.0"/>
      <param name="min_x_velocity_threshold" value="0.05"/>
      <param name="min_y_velocity_threshold" value="0.05"/>
      <param name="min_theta_velocity_threshold" value="0.05"/>
      <param name="progress_checker_plugin" value="progress_checker"/>
      <param name="goal_checker_plugin" value="goal_checker"/>
      <param name="controller_plugins" value="FollowPath"/>

      <!-- Humanoid-specific controller parameters -->
      <param name="FollowPath.type" value="dwb_core::DWBLocalPlanner"/>
      <param name="FollowPath.traverse_depth" value="2"/>
      <param name="FollowPath.transform_tolerance" value="0.2"/>
      <param name="FollowPath.short_circuit_trajectory_evaluation" value="true"/>
      <param name="FollowPath.debug_trajectory_details" value="false"/>
      <param name="FollowPath.minimum_turning_scale" value="0.5"/>
      <param name="FollowPath.forward_sampling_distance" value="0.5"/>
      <param name="FollowPath.rotate_to_heading_angular_vel" value="1.0"/>
      <param name="FollowPath.max_rotational_vel" value="1.0"/>
      <param name="FollowPath.min_rotational_vel" value="0.4"/>
      <param name="FollowPath.similarity_tolerance" value="0.25"/>
      <param name="FollowPath.max_lateral_acc" value="0.4"/>
      <param name="FollowPath.max_angular_accel" value="3.2"/>
      <param name="FollowPath.max_translational_acc" value="0.8"/>
    </node>

    <!-- Behavior Tree Node -->
    <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="global_frame" value="map"/>
      <param name="robot_base_frame" value="base_link"/>
      <param name="odom_topic" value="/odom"/>
      <param name="default_bt_xml_filename" value="$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_w_replanning_and_recovery.xml"/>
      <param name="plugin_lib_names" value="
        nav2_compute_path_to_pose_action_bt_node
        nav2_follow_path_action_bt_node
        nav2_back_up_action_bt_node
        nav2_spin_action_bt_node
        nav2_wait_action_bt_node
        nav2_clear_costmap_service_bt_node
        nav2_is_stuck_condition_bt_node
        nav2_goal_reached_condition_bt_node
        nav2_goal_updated_condition_bt_node
        nav2_initial_pose_received_condition_bt_node
        nav2_reinitialize_global_localization_service_bt_node
        nav2_rate_controller_bt_node
        nav2_distance_controller_bt_node
        nav2_speed_controller_bt_node
        nav2_truncate_path_action_bt_node
        nav2_goal_updater_node_bt_node
        nav2_recovery_node_bt_node
        nav2_pipeline_sequence_bt_node
        nav2_round_robin_node_bt_node
        nav2_transform_available_condition_bt_node
        nav2_time_expired_condition_bt_node
        nav2_path_expiring_timer_condition
        nav2_distance_traveled_condition_bt_node
        nav2_single_trigger_bt_node
        nav2_is_battery_low_condition_bt_node
        nav2_navigate_through_poses_action_bt_node
        nav2_navigate_to_pose_action_bt_node
        nav2_remove_passed_goals_action_bt_node
        nav2_planner_selector_bt_node
        nav2_controller_selector_bt_node
        nav2_goal_checker_selector_bt_node"/>
    </node>

    <!-- Lifecycle Manager -->
    <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager">
      <param name="use_sim_time" value="$(var use_sim_time)"/>
      <param name="autostart" value="$(var autostart)"/>
      <param name="node_names" value="[map_server, planner_server, controller_server, bt_navigator]"/>
    </node>
  </group>
</launch>
```

### Isaac-Specific Nav2 Configuration

```yaml
# Isaac-specific Nav2 configuration
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::IsaacDifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

amcl_map_client:
  ros__parameters:
    use_sim_time: true

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: true

bt_navigator:
  ros__parameters:
    use_sim_time: true
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.5
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: true
      min_vel_x: 0.05
      max_vel_x: 0.3
      min_vel_y: -0.1
      max_vel_y: 0.1
      max_vel_theta: 0.5
      min_speed_xy: 0.1
      max_speed_xy: 0.3
      min_speed_theta: 0.1
      acc_lim_x: 0.3
      acc_lim_y: 0.1
      acc_lim_theta: 0.3
      decel_lim_x: -0.3
      decel_lim_y: -0.1
      decel_lim_theta: -0.3
      vx_samples: 20
      vy_samples: 5
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.15
      stateful: true
      oscillation_reset_dist: 0.05
      oscillation_reset_angle: 0.2
      prune_plan: true
      prune_distance: 1.0
      debug_expression: "\"\""
      desired_linear_vel: 0.3
      desired_angular_vel: 0.5
      max_vel_theta: 0.5
      min_vel_theta: -0.5

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      resolution: 0.025  # Higher resolution for humanoid precision
      robot_radius: 0.35  # Larger radius for humanoid safety
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 0.7
        cost_scaling_factor: 6.0
      obstacle_layer:
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_range: 3.0
          obstacle_range: 2.5
          origin_z: 0.0
          z_resolution: 0.2
          z_voxels: 4
          unknown_threshold: 15
          mark_threshold: 0
      voxel_layer:
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_range: 3.0
          obstacle_range: 2.5
          origin_z: 0.0
          z_resolution: 0.2
          z_voxels: 4
          unknown_threshold: 15

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 2.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      resolution: 0.1  # Lower resolution for global planning efficiency
      robot_radius: 0.3
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        inflation_radius: 1.0
        cost_scaling_factor: 4.0
      obstacle_layer:
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan
          raytrace_range: 4.0
          obstacle_range: 3.0