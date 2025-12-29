# Nav2 Path Planning for Humanoid Robots

## Overview

Navigation2 (Nav2) is the standard ROS 2 navigation framework that provides path planning, execution, and obstacle avoidance capabilities. This chapter covers adapting Nav2 for humanoid robots, which have different kinematic and dynamic constraints compared to traditional wheeled robots.

## Learning Objectives

By the end of this chapter, you will:
- Understand the Nav2 architecture and components
- Adapt Nav2 for humanoid robot kinematics and dynamics
- Configure custom controllers for bipedal locomotion
- Implement humanoid-specific path planning algorithms
- Integrate Nav2 with Isaac ROS perception outputs
- Handle humanoid-specific navigation challenges like balance and step planning

## Introduction to Navigation2

Navigation2 is a flexible navigation framework that includes:
- **Global Planner**: Creates optimal paths from start to goal
- **Local Planner**: Executes paths while avoiding obstacles
- **Controller**: Generates velocity commands for robot motion
- **Recovery Behaviors**: Handles navigation failures
- **Behavior Trees**: Coordinates navigation tasks

### Key Components
- **Navigation Stack**: Core navigation functionality
- **Plugins**: Extensible architecture for custom algorithms
- **Parameter System**: Configurable behavior tuning
- **Lifecycle Management**: Proper node state management
- **Action Interface**: Asynchronous navigation goals

## Nav2 Architecture for Humanoid Robots

### Standard vs Humanoid Navigation

Traditional Nav2 is designed for wheeled robots with:
- 2D planar movement (x, y, theta)
- Continuous motion capabilities
- Simple kinematic constraints

Humanoid robots require:
- 3D movement considerations (x, y, z, roll, pitch, yaw)
- Discrete step-based locomotion
- Balance and stability constraints
- Complex kinematic chains

### Humanoid-Specific Modifications

Adaptations needed for humanoid robots:
- **Step Planning**: Plan discrete footsteps instead of smooth paths
- **Balance Constraints**: Consider center of mass and stability
- **Kinematic Planning**: Account for joint limits and reachability
- **Dynamic Constraints**: Handle bipedal dynamics
- **Terrain Analysis**: Evaluate walkable surfaces for bipedal locomotion

## Global Path Planning for Humanoids

### Standard Global Planners

Nav2 includes several global planners:
- **NavFn**: Fast-marching Dijkstra-based planner
- **Global Planner**: A* implementation
- **Theta* Planner**: Any-angle path planning
- **SMAC Planner**: Grid-based motion primitives

### Humanoid-Specific Global Planning

For humanoid robots, global planning must consider:
- **Step Reachability**: Ensure each step is physically achievable
- **Balance Transitions**: Plan stable transitions between steps
- **Terrain Traversability**: Evaluate ground for bipedal locomotion
- **Obstacle Clearances**: Account for robot body dimensions

### Custom Global Planner Implementation

Example of a humanoid-aware global planner:

```cpp
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.h"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "geometry_msgs/msg/point.hpp"

class HumanoidGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> global_costmap,
    std::string local_footprint) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Humanoid-specific parameters
  double max_step_length_;
  double balance_margin_;
  double foot_separation_;
  double center_of_mass_height_;

  // Balance checking functions
  bool isStableTransition(
    const geometry_msgs::msg::Point & from,
    const geometry_msgs::msg::Point & to);

  // Step planning functions
  std::vector<geometry_msgs::msg::Point> planSteps(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);
};
```

## Local Path Planning and Execution

### Standard Local Planners

Standard Nav2 local planners include:
- **DWB (Dynamic Window Approach)**: Velocity-based local planning
- **TEB (Timed Elastic Band)**: Trajectory optimization
- **RPP (Recovery Planner)**: Path following with obstacle avoidance

### Humanoid-Specific Local Planning

For humanoid robots, local planning must handle:
- **Step-by-Step Execution**: Execute navigation as discrete steps
- **Balance Maintenance**: Maintain stability during motion
- **Dynamic Adjustments**: Adjust step plan based on obstacles
- **Recovery Behaviors**: Handle balance recovery when needed

### Step-Based Local Planner

Example implementation of a step-based local planner:

```cpp
#include "nav2_core/local_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

class HumanoidLocalPlanner : public nav2_core::LocalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
    std::shared_ptr<nav2_util::LifecycleNode> node) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  bool setPlan(const nav_msgs::msg::Path & path) override;
  bool computeVelocityCommands(geometry_msgs::msg::Twist & cmd_vel) override;
  bool isGoalReached() override;

private:
  // Current step execution
  bool executeNextStep();
  bool checkBalance();
  bool adjustStepForObstacles();

  // Humanoid-specific parameters
  double step_duration_;
  double max_step_width_;
  double balance_threshold_;

  // Step execution state
  std::vector<geometry_msgs::msg::Point> remaining_steps_;
  geometry_msgs::msg::Point current_goal_;
};
```

## Controller Adaptation for Humanoid Robots

### Standard Controllers

Nav2 controllers typically generate velocity commands:
- **DWB Controller**: Velocity-based control
- **PID Controllers**: Proportional-Integral-Derivative control
- **MPC Controllers**: Model Predictive Control

### Humanoid-Specific Controllers

Humanoid robots require controllers that:
- **Generate Step Commands**: Instead of velocity commands
- **Consider Balance**: Account for center of mass
- **Handle Joint Limits**: Respect humanoid joint constraints
- **Coordinate Multiple Limbs**: Synchronize legs and arms

### Balance-Aware Controller

```cpp
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class HumanoidController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap,
    std::shared_ptr<nav2_util::LifecycleNode> node) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  geometry_msgs::msg::Twist velocityCallback(
    nav_msgs::msg::Path::ConstSharedPtr path,
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose,
    geometry_msgs::msg::Twist::ConstSharedPtr velocity) override;

private:
  // Balance control functions
  double calculateZMP(const geometry_msgs::msg::Pose & pose);
  bool isStable(const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Twist generateStepCommand(
    const geometry_msgs::msg::Pose & target_pose);

  // Humanoid parameters
  double com_height_;
  double max_angular_velocity_;
  double balance_margin_;
};
```

## Isaac ROS Integration

### Perception Data Integration

Integrate Isaac ROS perception outputs with Nav2:
- **Obstacle Detection**: Use Isaac ROS object detection for costmap updates
- **Terrain Analysis**: Use Isaac ROS terrain analysis for walkable surface detection
- **Dynamic Obstacles**: Track moving objects for navigation safety
- **Semantic Maps**: Use semantic segmentation for environment understanding

### Costmap Integration

Example of integrating Isaac ROS perception with Nav2 costmaps:

```cpp
#include "nav2_costmap_2d/costmap_2d_ros.h"
#include "isaac_ros_detectnet_interfaces/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class IsaacPerceptionIntegrator
{
public:
  IsaacPerceptionIntegrator(rclcpp::Node * node);

  void detectionCallback(
    const isaac_ros_detectnet_interfaces::msg::Detection2DArray::SharedPtr msg);

  void pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  void updateCostmapWithDetections(
    const isaac_ros_detectnet_interfaces::msg::Detection2DArray::SharedPtr & detections);

  void updateCostmapWithPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & pointcloud);

  rclcpp::Subscription<isaac_ros_detectnet_interfaces::msg::Detection2DArray>::SharedPtr
    detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
```

### Semantic Costmap Updates

```cpp
void IsaacPerceptionIntegrator::updateCostmapWithDetections(
  const isaac_ros_detectnet_interfaces::msg::Detection2DArray::SharedPtr & detections)
{
  // Get costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();

  // Process each detection
  for (const auto & detection : detections->detections) {
    // Get class and confidence
    std::string class_name = detection.results[0].id;
    float confidence = detection.results[0].score;

    // Update costmap based on object class
    if (class_name == "person" && confidence > 0.7) {
      // Add dynamic obstacle with appropriate cost
      addDynamicObstacle(costmap, detection.bbox, 200); // LETHAL_OBSTACLE cost
    } else if (class_name == "chair" && confidence > 0.7) {
      // Add static obstacle with appropriate cost
      addStaticObstacle(costmap, detection.bbox, 254); // INSCRIBED_INFLATED_OBSTACLE cost
    }
  }

  // Update costmap
  costmap_ros_->updateMap();
}
```

## Humanoid-Specific Navigation Challenges

### Balance and Stability

Key challenges for humanoid navigation:
- **Center of Mass Management**: Keep COM within support polygon
- **Zero Moment Point (ZMP)**: Maintain ZMP within stable region
- **Step Timing**: Coordinate steps with balance control
- **Recovery Actions**: Handle near-fall situations

### Terrain Adaptation

Humanoid robots need to consider terrain differently:
- **Surface Traversability**: Not all "navigable" surfaces are walkable
- **Step Height**: Maximum step height limitations
- **Surface Stability**: Ensure surface can support robot weight
- **Slip Detection**: Handle slippery surfaces appropriately

### Multi-Legged Coordination

For humanoid robots with arms:
- **Arm Coordination**: Use arms for balance during navigation
- **Upper Body Planning**: Consider arm positions during movement
- **Manipulation Integration**: Plan navigation that allows manipulation

## Configuration for Humanoid Robots

### Parameter Tuning

Key parameters to tune for humanoid navigation:
- `max_step_length`: Maximum distance for a single step
- `step_duration`: Time allocated for each step
- `balance_margin`: Safety margin for balance maintenance
- `foot_separation`: Desired distance between feet
- `com_height`: Center of mass height for balance calculations

### Costmap Configuration

Example costmap configuration for humanoid robots:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: false
      width: 20
      height: 20
      resolution: 0.05

      # Humanoid-specific inflation
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 1.5  # Higher inflation for humanoid safety
        inflation_radius: 0.8     # Larger radius for humanoid body
        robot_radius: 0.4         # Humanoid robot radius

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 6.0
      height: 6.0
      resolution: 0.05

      # Humanoid-specific local costmap
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2      # Voxel height for humanoid 3D awareness
        z_voxels: 8           # Number of vertical voxels
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
```

### Behavior Tree Configuration

Custom behavior tree for humanoid navigation:

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithRecovery">
      <RateController hz="10">
        <ReactiveSequence>
          <RemovePassedGoals transform_tolerance="0.1" />
          <GoalUpdated/>
          <IsNewPath path="{path}" new_path="{new_path}"/>
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
          <Smoother path="{path}" output="{smoothed_path}" smoother_id="simple_smoother"/>
          <TruncatePath input_path="{smoothed_path}" output_path="{truncated_path}" distance="1.0"/>
          <HumanoidPathToSteps path="{truncated_path}" steps="{steps}"/>
          <FollowSteps steps="{steps}" controller_id="HumanoidController"/>
        </ReactiveSequence>
      </RateController>
    </PipelineSequence>
  </BehaviorTree>

  <BehaviorTree ID="RecoveryNode">
    <ReactiveFallback name="RecoveryFallback">
      <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
      <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
      <BackUp backup_dist="0.15" backup_speed="0.025"/>
      <HumanoidSpin spin_dist="1.57"/>
      <Wait wait_duration="5"/>
    </ReactiveFallback>
  </BehaviorTree>
</root>
```

## Practical Example: Humanoid Navigation Pipeline

### Complete Launch File

```xml
<launch>
  <!-- Navigation2 Stack with Humanoid Adaptations -->
  <group>
    <node pkg="nav2_controller_server" exec="controller_server" name="controller_server">
      <param name="use_sim_time" value="True"/>
      <param name="controller_frequency" value="20.0"/>
      <param name="min_x_velocity_threshold" value="0.001"/>
      <param name="min_y_velocity_threshold" value="0.5"/>
      <param name="min_theta_velocity_threshold" value="0.001"/>
      <param name="progress_checker_plugin" value="progress_checker"/>
      <param name="goal_checker_plugin" value="goal_checker"/>
      <param name="controller_plugins" value="['HumanoidController']"/>

      <!-- Humanoid Controller Parameters -->
      <param name="HumanoidController.type" value="nav2_mppi_controller::Controller"/>
      <param name="HumanoidController.time_steps" value="20"/>
      <param name="HumanoidController.model_dt" value="0.05"/>
      <param name="HumanoidController.batch_size" value="2000"/>
      <param name="HumanoidController.rolling_horizon_window" value="1.0"/>
      <param name="HumanoidController.balance_margin" value="0.1"/>
      <param name="HumanoidController.max_step_length" value="0.3"/>
    </node>

    <node pkg="nav2_planner_server" exec="planner_server" name="planner_server">
      <param name="use_sim_time" value="True"/>
      <param name="planner_plugins" value="['GridBased']"/>
      <param name="GridBased.type" value="nav2_navfn_planner::NavfnPlanner"/>
      <param name="GridBased.scale" value="1.0"/>
      <param name="GridBased.step_size" value="0.05"/>
      <param name="GridBased.step_size_factor" value="2.0"/>
      <param name="GridBased.max_step_length" value="0.3"/>  <!-- Humanoid-specific -->
    </node>

    <node pkg="nav2_recovery_server" exec="recovery_server" name="recovery_server">
      <param name="use_sim_time" value="True"/>
      <param name="recovery_plugins" value="['backup', 'spin', 'wait', 'HumanoidBalanceRecovery']"/>
      <param name="backup.type" value="nav2_behaviors::BackUp"/>
      <param name="spin.type" value="nav2_behaviors::Spin"/>
      <param name="wait.type" value="nav2_behaviors::Wait"/>
      <param name="HumanoidBalanceRecovery.type" value="humanoid_nav2_behaviors::BalanceRecovery"/>
    </node>

    <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator">
      <param name="use_sim_time" value="True"/>
      <param name="global_frame" value="map"/>
      <param name="robot_base_frame" value="base_link"/>
      <param name="odom_topic" value="/odom"/>
      <param name="bt_loop_duration" value="10"/>
      <param name="default_server_timeout" value="20"/>
      <param name="enable_groot_monitoring" value="True"/>
      <param name="groot_zmq_publisher_port" value="1666"/>
      <param name="groot_zmq_server_port" value="1667"/>
      <param name="default_nav_through_poses_bt_xml" value="$(find-pkg-share humanoid_nav2_behaviors)/behavior_trees/navigate_w_replanning_and_humanoid_recovery.xml"/>
      <param name="default_navigate_to_pose_bt_xml" value="$(find-pkg-share humanoid_nav2_behaviors)/behavior_trees/navigate_w_replanning_and_humanoid_recovery.xml"/>
    </node>

    <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager_navigation">
      <param name="use_sim_time" value="True"/>
      <param name="autostart" value="True"/>
      <param name="node_names" value="[controller_server, planner_server, recovery_server, bt_navigator, velocity_smoother, collision_detector]"/>
    </node>
  </group>

  <!-- Isaac ROS Integration -->
  <group>
    <node pkg="isaac_ros_visual_slam" exec="visual_slam_node" name="visual_slam_node">
      <param name="enable_rectified_pose" value="true"/>
      <param name="map_frame" value="map"/>
      <param name="odom_frame" value="odom"/>
      <param name="base_frame" value="base_link"/>
    </node>

    <node pkg="isaac_ros_detectnet" exec="detectnet_node" name="detectnet_node">
      <param name="input_image_topic" value="/camera/image_rect_color"/>
      <param name="input_camera_info_topic" value="/camera/camera_info"/>
      <param name="network_type" value="detectnet"/>
      <param name="model_name" value="ssd-mobilenet-v2"/>
      <param name="input_blob_name" value="input_0"/>
      <param name="output_cvg_blob_name" value="scores"/>
      <param name="output_bbox_blob_name" value="boxes"/>
    </node>
  </group>
</launch>
```

### Running the Navigation System

```bash
# Source workspace
source ~/isaac_ws/install/setup.bash

# Launch the complete system
ros2 launch humanoid_nav2_bringup navigation.launch.py

# Send navigation goal
ros2 run nav2_test nav2_test --ros-args -p goal_x:=1.0 -p goal_y:=1.0
```

## Performance Considerations

### Real-time Requirements

Humanoid navigation has strict timing requirements:
- **Balance Control**: High-frequency balance updates (100Hz+)
- **Step Execution**: Precise timing for step execution
- **Sensor Processing**: Real-time perception processing
- **Path Planning**: Fast re-planning for dynamic obstacles

### Computational Optimization

Optimize for humanoid navigation:
- **Multi-threading**: Separate balance control and navigation threads
- **Prediction**: Predict future states for smoother execution
- **Caching**: Cache pre-computed step patterns
- **Simplification**: Use simplified models for real-time planning

## Troubleshooting Humanoid Navigation

### Common Issues

1. **Balance Failures**:
   - Check center of mass calculations
   - Verify step timing and execution
   - Adjust balance controller parameters

2. **Path Planning Failures**:
   - Verify costmap inflation parameters
   - Check kinematic constraints
   - Validate goal position reachability

3. **Integration Problems**:
   - Ensure TF tree is properly connected
   - Verify message timing and synchronization
   - Check frame conventions match expectations

### Debugging Tools

Use Nav2 tools for debugging:
```bash
# Visualize navigation in RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_namespaced_view.rviz

# Monitor navigation topics
ros2 topic echo /local_costmap/costmap
ros2 topic echo /global_costmap/costmap
ros2 topic echo /plan
ros2 topic echo /cmd_vel
```

## Summary

This chapter covered adapting Navigation2 for humanoid robots, addressing the unique challenges of bipedal locomotion, balance maintenance, and step-based navigation. You learned how to modify global and local planners, adapt controllers for humanoid kinematics, integrate with Isaac ROS perception, and configure Nav2 for humanoid-specific requirements. This completes the AI-Robot Brain module, providing a comprehensive understanding of NVIDIA Isaac ecosystem for humanoid robotics applications.