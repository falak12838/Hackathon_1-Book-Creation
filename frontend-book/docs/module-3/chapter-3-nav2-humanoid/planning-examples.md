# Path Planning Examples with Humanoid Constraints in Nav2

## Overview

This section provides practical examples of path planning in Nav2 specifically adapted for humanoid robots with their unique locomotion constraints. These examples demonstrate how to implement navigation that respects bipedal walking patterns, balance requirements, and step constraints while achieving efficient and safe navigation.

## Basic Path Planning Example

### Simple Path Planning with Step Constraints

```python
# Basic humanoid path planning
import numpy as np
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

class HumanoidPathPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Step constraints for humanoid robot
        self.max_step_length = 0.30  # meters
        self.max_step_height = 0.15  # meters
        self.min_step_length = 0.05  # meters
        self.step_width = 0.25       # meters (lateral distance between feet)

        # Navigation action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

    def plan_path_with_step_constraints(self, start_pose, goal_pose):
        """Plan path with humanoid step constraints"""
        # Get current costmap
        costmap = self.get_current_costmap()

        # Plan initial path using standard algorithm (e.g., A*)
        raw_path = self.plan_raw_path(start_pose, goal_pose, costmap)

        # Validate and adjust path for step constraints
        constrained_path = self.validate_and_adjust_path(raw_path)

        return constrained_path

    def validate_and_adjust_path(self, raw_path):
        """Validate path against step constraints and adjust if needed"""
        constrained_path = [raw_path[0]]  # Start with first pose

        for i in range(1, len(raw_path)):
            current_pose = raw_path[i]
            previous_pose = constrained_path[-1]

            # Calculate step distance
            dx = current_pose.pose.position.x - previous_pose.pose.position.x
            dy = current_pose.pose.position.y - previous_pose.pose.position.y
            step_distance = np.sqrt(dx*dx + dy*dy)

            # Check if step is within constraints
            if step_distance <= self.max_step_length:
                # Step is acceptable, add to path
                constrained_path.append(current_pose)
            else:
                # Step is too long, need intermediate waypoints
                intermediate_poses = self.generate_intermediate_poses(
                    previous_pose, current_pose, step_distance
                )
                constrained_path.extend(intermediate_poses)
                constrained_path.append(current_pose)  # Add the original target

        return constrained_path

    def generate_intermediate_poses(self, start_pose, end_pose, total_distance):
        """Generate intermediate poses to break up a long step"""
        num_intermediate = int(np.ceil(total_distance / self.max_step_length))
        intermediate_poses = []

        for i in range(1, num_intermediate):
            ratio = i / num_intermediate

            # Interpolate position
            intermediate_pose = PoseStamped()
            intermediate_pose.header = start_pose.header
            intermediate_pose.pose.position.x = (
                start_pose.pose.position.x +
                ratio * (end_pose.pose.position.x - start_pose.pose.position.x)
            )
            intermediate_pose.pose.position.y = (
                start_pose.pose.position.y +
                ratio * (end_pose.pose.position.y - start_pose.pose.position.y)
            )
            intermediate_pose.pose.position.z = (
                start_pose.pose.position.z +
                ratio * (end_pose.pose.position.z - start_pose.pose.position.z)
            )

            # Interpolate orientation (simplified)
            intermediate_pose.pose.orientation = start_pose.pose.orientation

            intermediate_poses.append(intermediate_pose)

        return intermediate_poses
```

## Advanced Path Planning Examples

### Footstep-Aware Path Planning

```python
# Advanced footstep-aware path planning
class FootstepAwarePlanner(HumanoidPathPlanner):
    def __init__(self):
        super().__init__()
        self.footstep_planner = FootstepPlanner()
        self.balance_checker = BalanceChecker()

    def plan_path_with_footsteps(self, start_pose, goal_pose):
        """Plan path and generate corresponding footstep plan"""
        # Plan continuous path first
        continuous_path = self.plan_path_with_step_constraints(start_pose, goal_pose)

        # Generate footstep plan from continuous path
        footsteps = self.footstep_planner.generate_footsteps_from_path(
            continuous_path, start_pose
        )

        # Validate footsteps for balance
        valid_footsteps = self.validate_footsteps_for_balance(footsteps)

        return continuous_path, valid_footsteps

    def validate_footsteps_for_balance(self, footsteps):
        """Validate footstep sequence for balance"""
        validated_footsteps = []
        current_support_polygon = self.calculate_initial_support_polygon()

        for i, footstep in enumerate(footsteps):
            # Check if footstep maintains balance
            new_support_polygon = self.update_support_polygon(
                current_support_polygon, footstep
            )

            # Calculate ZMP (Zero Moment Point) with new footstep
            zmp = self.calculate_zmp_with_footstep(footstep, i)

            # Check if ZMP is within support polygon
            if self.balance_checker.is_zmp_stable(zmp, new_support_polygon):
                validated_footsteps.append(footstep)
                current_support_polygon = new_support_polygon
            else:
                # Adjust footstep to maintain balance
                adjusted_footstep = self.balance_checker.adjust_footstep_for_balance(
                    footstep, current_support_polygon, zmp
                )
                validated_footsteps.append(adjusted_footstep)
                current_support_polygon = self.update_support_polygon(
                    current_support_polygon, adjusted_footstep
                )

        return validated_footsteps

class FootstepPlanner:
    def __init__(self):
        self.foot_separation = 0.25  # meters between feet
        self.nominal_step_length = 0.25  # meters
        self.step_timing = 1.0  # seconds per step

    def generate_footsteps_from_path(self, path, start_pose):
        """Generate footstep plan from continuous path"""
        footsteps = []

        # Determine initial foot positions
        left_foot_pose = self.calculate_initial_left_foot(start_pose)
        right_foot_pose = self.calculate_initial_right_foot(start_pose)

        # Assume starting with left foot support
        current_left_support = True

        # Process path in segments
        for i in range(len(path) - 1):
            path_segment = self.extract_path_segment(path, i, i+1)

            # Calculate required footsteps for this segment
            segment_footsteps = self.plan_footsteps_for_segment(
                path_segment, left_foot_pose, right_foot_pose, current_left_support
            )

            footsteps.extend(segment_footsteps)

            # Update support foot for next segment
            if len(segment_footsteps) % 2 == 1:  # If odd number of steps
                current_left_support = not current_left_support

        return footsteps

    def calculate_initial_left_foot(self, robot_pose):
        """Calculate initial left foot position based on robot pose"""
        left_foot = PoseStamped()
        left_foot.header = robot_pose.header

        # Place left foot slightly to the left of robot center
        left_foot.pose.position.x = robot_pose.pose.position.x
        left_foot.pose.position.y = robot_pose.pose.position.y + self.foot_separation / 2
        left_foot.pose.position.z = robot_pose.pose.position.z

        left_foot.pose.orientation = robot_pose.pose.orientation

        return left_foot

    def calculate_initial_right_foot(self, robot_pose):
        """Calculate initial right foot position based on robot pose"""
        right_foot = PoseStamped()
        right_foot.header = robot_pose.header

        # Place right foot slightly to the right of robot center
        right_foot.pose.position.x = robot_pose.pose.position.x
        right_foot.pose.position.y = robot_pose.pose.position.y - self.foot_separation / 2
        right_foot.pose.position.z = robot_pose.pose.position.z

        right_foot.pose.orientation = robot_pose.pose.orientation

        return right_foot

    def plan_footsteps_for_segment(self, path_segment, left_pose, right_pose, left_support):
        """Plan footstep sequence for a path segment"""
        footsteps = []

        # Calculate the direction and distance of the path segment
        start_pos = path_segment[0].pose.position
        end_pos = path_segment[-1].pose.position
        dx = end_pos.x - start_pos.x
        dy = end_pos.y - start_pos.y
        distance = np.sqrt(dx*dx + dy*dy)

        # Calculate number of steps needed
        num_steps = max(1, int(np.ceil(distance / self.nominal_step_length)))

        # Generate footsteps based on current support foot
        current_left_pose = left_pose
        current_right_pose = right_pose
        current_left_support = left_support

        for step_idx in range(num_steps):
            if current_left_support:
                # Right foot moves
                next_right_pose = self.calculate_next_foot_pose(
                    current_right_pose, start_pos, end_pos, step_idx, num_steps
                )
                footsteps.append({
                    'foot': 'right',
                    'pose': next_right_pose,
                    'step_index': step_idx,
                    'timestamp': step_idx * self.step_timing
                })
                current_right_pose = next_right_pose
                current_left_support = False
            else:
                # Left foot moves
                next_left_pose = self.calculate_next_foot_pose(
                    current_left_pose, start_pos, end_pos, step_idx, num_steps
                )
                footsteps.append({
                    'foot': 'left',
                    'pose': next_left_pose,
                    'step_index': step_idx,
                    'timestamp': step_idx * self.step_timing
                })
                current_left_pose = next_left_pose
                current_left_support = True

        return footsteps

    def calculate_next_foot_pose(self, current_pose, start_pos, end_pos, step_idx, total_steps):
        """Calculate next foot pose based on path direction"""
        # Calculate target position along the path
        ratio = (step_idx + 1) / total_steps
        target_x = start_pos.x + ratio * (end_pos.x - start_pos.x)
        target_y = start_pos.y + ratio * (end_pos.y - start_pos.y)
        target_z = start_pos.z + ratio * (end_pos.z - start_pos.z)

        next_pose = PoseStamped()
        next_pose.header = current_pose.header
        next_pose.pose.position.x = target_x
        next_pose.pose.position.y = target_y
        next_pose.pose.position.z = target_z
        next_pose.pose.orientation = current_pose.pose.orientation

        return next_pose
```

### Terrain-Aware Path Planning

```python
# Terrain-aware path planning for humanoid robots
class TerrainAwarePathPlanner(FootstepAwarePlanner):
    def __init__(self):
        super().__init__()
        self.terrain_classifier = TerrainClassifier()
        self.gait_selector = GaitSelector()

    def plan_path_with_terrain_awareness(self, start_pose, goal_pose, terrain_map):
        """Plan path considering terrain characteristics"""
        # Classify terrain along potential paths
        terrain_aware_path = self.find_terrain_appropriate_path(
            start_pose, goal_pose, terrain_map
        )

        # Adjust gait pattern based on terrain
        gait_plan = self.plan_gait_for_terrain(terrain_aware_path, terrain_map)

        # Generate footstep plan with appropriate gait
        footsteps = self.footstep_planner.generate_footsteps_with_gait(
            terrain_aware_path, gait_plan
        )

        return terrain_aware_path, footsteps, gait_plan

    def find_terrain_appropriate_path(self, start_pose, goal_pose, terrain_map):
        """Find path that avoids unsuitable terrain for humanoid navigation"""
        # Use a modified path planning algorithm that considers terrain costs
        path = self.terrain_aware_a_star(start_pose, goal_pose, terrain_map)
        return path

    def terrain_aware_a_star(self, start_pose, goal_pose, terrain_map):
        """A* algorithm modified to consider terrain costs"""
        start_idx = self.world_to_grid(start_pose)
        goal_idx = self.world_to_grid(goal_pose)

        open_set = [(0, start_idx)]
        came_from = {}
        g_score = {start_idx: 0}
        f_score = {start_idx: self.terrain_heuristic(start_idx, goal_idx, terrain_map)}

        import heapq
        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_idx:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_terrain_valid_neighbors(current, terrain_map):
                # Calculate cost considering terrain type
                terrain_cost = self.get_terrain_cost(current, neighbor, terrain_map)
                tentative_g_score = g_score[current] + terrain_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.terrain_heuristic(neighbor, goal_idx, terrain_map)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def get_terrain_valid_neighbors(self, pos, terrain_map):
        """Get neighbors that are valid for humanoid navigation based on terrain"""
        x, y = pos
        neighbors = []

        # Check 8-connected neighborhood
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                nx, ny = x + dx, y + dy

                if (0 <= nx < terrain_map.width and 0 <= ny < terrain_map.height):
                    # Check if terrain at neighbor is passable for humanoid
                    terrain_type = terrain_map.get_terrain_type(nx, ny)
                    if self.is_terrain_passable_for_humanoid(terrain_type):
                        neighbors.append((nx, ny))

        return neighbors

    def is_terrain_passable_for_humanoid(self, terrain_type):
        """Check if terrain type is passable for humanoid"""
        passable_terrain_types = [
            'flat', 'slightly_rough', 'grass', 'asphalt', 'indoor_floor'
        ]
        return terrain_type in passable_terrain_types

    def get_terrain_cost(self, from_pos, to_pos, terrain_map):
        """Get movement cost considering terrain type"""
        terrain_type = terrain_map.get_terrain_type(to_pos[0], to_pos[1])

        # Define terrain-specific costs
        terrain_costs = {
            'flat': 1.0,
            'slightly_rough': 1.2,
            'grass': 1.3,
            'asphalt': 1.0,
            'indoor_floor': 1.0,
            'rough': 2.0,
            'sloped': 1.8,
            'stair': 3.0
        }

        base_cost = terrain_costs.get(terrain_type, 2.5)  # Default cost for unknown terrain

        # Add distance cost
        distance_cost = self.calculate_distance_cost(from_pos, to_pos)

        return base_cost + distance_cost

    def plan_gait_for_terrain(self, path, terrain_map):
        """Plan appropriate gait pattern based on terrain"""
        gait_plan = []

        for i in range(len(path) - 1):
            current_pos = self.world_to_grid(path[i])
            terrain_type = terrain_map.get_terrain_type(current_pos[0], current_pos[1])

            # Select gait based on terrain
            gait_params = self.gait_selector.select_gait_for_terrain(terrain_type)
            gait_plan.append({
                'segment': i,
                'terrain_type': terrain_type,
                'gait_params': gait_params
            })

        return gait_plan

class GaitSelector:
    def __init__(self):
        self.gait_parameters = {
            'flat': {
                'step_length': 0.30,
                'step_height': 0.05,
                'step_time': 1.0,
                'balance_margin': 0.1
            },
            'slightly_rough': {
                'step_length': 0.20,
                'step_height': 0.10,
                'step_time': 1.2,
                'balance_margin': 0.15
            },
            'grass': {
                'step_length': 0.25,
                'step_height': 0.08,
                'step_time': 1.1,
                'balance_margin': 0.12
            },
            'sloped': {
                'step_length': 0.20,
                'step_height': 0.15,
                'step_time': 1.3,
                'balance_margin': 0.18
            }
        }

    def select_gait_for_terrain(self, terrain_type):
        """Select appropriate gait parameters for terrain"""
        if terrain_type in self.gait_parameters:
            return self.gait_parameters[terrain_type]
        else:
            # Default to conservative gait for unknown terrain
            return self.gait_parameters['slightly_rough']
```

## Obstacle Avoidance Examples

### Humanoid-Aware Obstacle Avoidance

```python
# Humanoid-aware obstacle avoidance
class HumanoidObstacleAvoidance:
    def __init__(self, costmap, step_constraints):
        self.costmap = costmap
        self.step_constraints = step_constraints
        self.obstacle_detector = ObstacleDetector()

    def plan_path_with_obstacle_avoidance(self, start_pose, goal_pose, obstacles):
        """Plan path avoiding obstacles while respecting humanoid constraints"""
        # First, try to plan path without obstacles
        initial_path = self.plan_initial_path(start_pose, goal_pose)

        # Check for obstacles in path
        obstacle_free_path = self.avoid_obstacles_in_path(initial_path, obstacles)

        # Validate path for humanoid constraints
        validated_path = self.validate_path_for_humanoid(obstacle_free_path)

        return validated_path

    def avoid_obstacles_in_path(self, path, obstacles):
        """Modify path to avoid obstacles"""
        if not obstacles:
            return path

        modified_path = []
        i = 0

        while i < len(path):
            current_pose = path[i]
            modified_path.append(current_pose)

            # Check if next segment has obstacles
            if i + 1 < len(path):
                next_pose = path[i + 1]

                # Check for obstacles between current and next pose
                obstacles_in_segment = self.find_obstacles_in_segment(
                    current_pose, next_pose, obstacles
                )

                if obstacles_in_segment:
                    # Plan local avoidance around obstacles
                    avoidance_path = self.plan_local_avoidance(
                        current_pose, next_pose, obstacles_in_segment
                    )

                    if avoidance_path:
                        # Add avoidance path (excluding first point which is current pose)
                        modified_path.extend(avoidance_path[1:])
                        # Skip the original next pose since we've added an alternative
                        i += len(avoidance_path)
                    else:
                        # Local avoidance failed, try global replanning
                        global_path = self.plan_global_avoidance(
                            current_pose, path[-1], obstacles
                        )
                        if global_path:
                            # Replace rest of path with global avoidance path
                            modified_path = modified_path[:-1]  # Remove last added pose
                            modified_path.extend(global_path[1:])
                            break
                        else:
                            # Both local and global avoidance failed
                            # Return original path with warning
                            return path
                else:
                    i += 1
            else:
                i += 1

        return modified_path

    def plan_local_avoidance(self, start_pose, end_pose, obstacles):
        """Plan local avoidance around obstacles"""
        # Create a local costmap around the problematic area
        local_costmap = self.create_local_costmap(start_pose, end_pose, obstacles)

        # Plan path in local costmap using A*
        avoidance_path = self.local_a_star(start_pose, end_pose, local_costmap)

        return avoidance_path

    def create_local_costmap(self, start_pose, end_pose, obstacles):
        """Create local costmap for obstacle avoidance"""
        # Calculate bounding box for area of interest
        min_x = min(start_pose.pose.position.x, end_pose.pose.position.x)
        max_x = max(start_pose.pose.position.x, end_pose.pose.position.x)
        min_y = min(start_pose.pose.position.y, end_pose.pose.position.y)
        max_y = max(start_pose.pose.position.y, end_pose.pose.position.y)

        # Add buffer around the area
        buffer = 2.0  # 2 meters buffer
        min_x -= buffer
        max_x += buffer
        min_y -= buffer
        max_y += buffer

        # Create local costmap with appropriate resolution
        resolution = self.costmap.resolution
        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)

        local_costmap = LocalCostmap(width, height, resolution, [min_x, min_y, 0])

        # Mark obstacles in local costmap
        for obstacle in obstacles:
            self.mark_obstacle_in_costmap(local_costmap, obstacle, min_x, min_y)

        return local_costmap

    def local_a_star(self, start_pose, goal_pose, local_costmap):
        """A* algorithm on local costmap"""
        # Convert poses to local costmap coordinates
        start_idx = local_costmap.world_to_map(
            start_pose.pose.position.x,
            start_pose.pose.position.y
        )
        goal_idx = local_costmap.world_to_map(
            goal_pose.pose.position.x,
            goal_pose.pose.position.y
        )

        # Run A* on local costmap
        path_indices = self.run_a_star(start_idx, goal_idx, local_costmap)

        # Convert back to world coordinates
        world_path = []
        for idx in path_indices:
            world_x, world_y = local_costmap.map_to_world(idx[0], idx[1])
            pose = PoseStamped()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = start_pose.pose.position.z  # Maintain z
            world_path.append(pose)

        return world_path

class LocalCostmap:
    def __init__(self, width, height, resolution, origin):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.data = [0] * (width * height)  # Initialize with free space

    def world_to_map(self, x_world, y_world):
        """Convert world coordinates to map indices"""
        x_idx = int((x_world - self.origin[0]) / self.resolution)
        y_idx = int((y_world - self.origin[1]) / self.resolution)
        return x_idx, y_idx

    def map_to_world(self, x_idx, y_idx):
        """Convert map indices to world coordinates"""
        x_world = x_idx * self.resolution + self.origin[0]
        y_world = y_idx * self.resolution + self.origin[1]
        return x_world, y_yworld
```

## Integration with Nav2

### Nav2 Path Planning Configuration

```python
# Nav2 integration example
class Nav2HumanoidIntegrator:
    def __init__(self):
        self.path_planner = HumanoidPathPlanner()
        self.footstep_planner = FootstepAwarePlanner()
        self.terrain_aware_planner = TerrainAwarePathPlanner()

    def integrate_with_nav2(self):
        """Integrate humanoid-aware planning with Nav2"""
        # This would typically involve creating custom plugins for Nav2
        pass

# Example Nav2 plugin for humanoid path planning
from nav2_core.costmap import Costmap
from nav2_core.planner import Planner
from geometry_msgs.msg import PoseStamped
from typing import List

class HumanoidGlobalPlanner(Planner):
    def __init__(self):
        self.initialized = False
        self.step_constraints = None
        self.terrain_map = None

    def configure(self, tf_buffer, costmap_ros, plugin_name):
        """Configure the planner"""
        self.name = plugin_name
        self.costmap = costmap_ros.get_costmap()
        self.tf_buffer = tf_buffer

        # Initialize humanoid-specific parameters
        self.step_constraints = HumanoidStepConstraints()
        self.terrain_map = TerrainMap()

        self.initialized = True
        self.get_logger().info(f'{self.name} plugin initialized')

    def cleanup(self):
        """Clean up resources"""
        self.get_logger().info(f'{self.name} plugin cleaned up')

    def set_costmap(self, costmap):
        """Set costmap for the planner"""
        self.costmap = costmap

    def create_plan(self, start: PoseStamped, goal: PoseStamped) -> List[PoseStamped]:
        """Create plan respecting humanoid constraints"""
        if not self.initialized:
            self.get_logger().error('Planner is not initialized')
            return []

        try:
            # Plan path with humanoid constraints
            path = self.plan_path_with_constraints(start, goal)

            # Validate path for humanoid navigation
            if self.validate_humanoid_path(path):
                return path
            else:
                # Try alternative planning approach
                return self.plan_alternative_path(start, goal)

        except Exception as e:
            self.get_logger().error(f'Error in planning: {str(e)}')
            return []

    def plan_path_with_constraints(self, start: PoseStamped, goal: PoseStamped) -> List[PoseStamped]:
        """Plan path with humanoid-specific constraints"""
        # Use the path planning methods developed above
        planner = HumanoidPathPlanner()
        return planner.plan_path_with_step_constraints(start, goal)

    def validate_humanoid_path(self, path: List[PoseStamped]) -> bool:
        """Validate path for humanoid navigation requirements"""
        if len(path) < 2:
            return False

        for i in range(len(path) - 1):
            # Check step constraints
            step_distance = self.calculate_step_distance(path[i], path[i+1])
            if step_distance > self.step_constraints.max_step_length:
                return False

            # Check if path segment is in passable terrain
            if not self.is_terrain_passable(path[i], path[i+1]):
                return False

        return True

    def calculate_step_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate distance between two poses"""
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return np.sqrt(dx*dx + dy*dy)

    def is_terrain_passable(self, pose1: PoseStamped, pose2: PoseStamped) -> bool:
        """Check if terrain between poses is passable"""
        # Check terrain type along the path segment
        return True  # Simplified - in practice, check terrain map
```

## Complete Example: Humanoid Navigation System

```python
# Complete humanoid navigation example
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class HumanoidNavigationSystem(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_system')

        # Initialize components
        self.path_planner = HumanoidPathPlanner()
        self.footstep_planner = FootstepAwarePlanner()
        self.obstacle_avoider = HumanoidObstacleAvoidance(None, None)
        self.terrain_aware_planner = TerrainAwarePathPlanner()

        # Navigation action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Subscribe to sensor data
        self.terrain_map_sub = self.create_subscription(
            # Define appropriate message type for terrain map
            'terrain_map',
            self.terrain_map_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            # Define appropriate message type for obstacles
            'obstacles',
            self.obstacle_callback,
            10
        )

    def navigate_to_pose(self, goal_pose):
        """Navigate to pose with humanoid-aware planning"""
        # Get current robot pose
        current_pose = self.get_current_pose()

        # Plan path with all constraints
        path, footsteps, gait_plan = self.plan_humanoid_path(
            current_pose, goal_pose
        )

        # Execute navigation
        success = self.execute_navigation_with_path(path, footsteps, gait_plan)

        return success

    def plan_humanoid_path(self, start_pose, goal_pose):
        """Plan complete humanoid navigation path"""
        # Get terrain information
        terrain_map = self.get_current_terrain_map()

        # Get obstacle information
        obstacles = self.get_current_obstacles()

        # Plan terrain-aware path with obstacle avoidance
        path = self.terrain_aware_planner.plan_path_with_terrain_awareness(
            start_pose, goal_pose, terrain_map
        )

        # Add obstacle avoidance if needed
        if obstacles:
            path = self.obstacle_avoider.plan_path_with_obstacle_avoidance(
                start_pose, goal_pose, obstacles
            )

        # Generate footstep plan
        footsteps = self.footstep_planner.generate_footsteps_from_path(
            path, start_pose
        )

        # Plan appropriate gait
        gait_plan = self.terrain_aware_planner.plan_gait_for_terrain(
            path, terrain_map
        )

        return path, footsteps, gait_plan

    def execute_navigation_with_path(self, path, footsteps, gait_plan):
        """Execute navigation following planned path and footsteps"""
        try:
            # Send navigation goal to Nav2
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = path[-1]  # Use last pose as goal

            self.nav_to_pose_client.wait_for_server()
            future = self.nav_to_pose_client.send_goal_async(goal_msg)

            # Monitor navigation progress and adjust based on humanoid constraints
            return self.monitor_and_adjust_navigation(future, footsteps, gait_plan)

        except Exception as e:
            self.get_logger().error(f'Navigation execution failed: {str(e)}')
            return False

    def monitor_and_adjust_navigation(self, future, footsteps, gait_plan):
        """Monitor navigation and make adjustments as needed"""
        # This would involve real-time monitoring of robot state
        # and making adjustments based on balance, obstacles, etc.
        pass

    def terrain_map_callback(self, msg):
        """Handle terrain map updates"""
        pass

    def obstacle_callback(self, msg):
        """Handle obstacle updates"""
        pass

    def get_current_pose(self):
        """Get current robot pose"""
        # This would interface with localization system
        pass

    def get_current_terrain_map(self):
        """Get current terrain map"""
        # Return current terrain map
        pass

    def get_current_obstacles(self):
        """Get current obstacles"""
        # Return current obstacle information
        pass

def main(args=None):
    rclpy.init(args=args)

    navigator = HumanoidNavigationSystem()

    # Example usage
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 5.0
    goal_pose.pose.position.z = 0.0

    success = navigator.navigate_to_pose(goal_pose)

    if success:
        print("Navigation completed successfully!")
    else:
        print("Navigation failed.")

    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### Path Planning Guidelines

1. **Constraint Validation**: Always validate paths against humanoid constraints
2. **Multi-Resolution Planning**: Use different resolutions for different planning needs
3. **Real-time Adaptation**: Adapt plans based on real-time sensor data
4. **Safety Margins**: Include appropriate safety margins in planning
5. **Performance Optimization**: Optimize algorithms for real-time execution

### Implementation Tips

- **Modular Design**: Keep path planning components modular and testable
- **Parameter Tuning**: Make parameters easily configurable
- **Logging**: Include comprehensive logging for debugging
- **Fallback Systems**: Implement fallback behaviors for plan failures
- **Validation**: Continuously validate plan feasibility

## Next Steps

With path planning examples understood, you're ready to explore:

- Integration with Isaac Sim for validation and simulation
- Advanced navigation strategies for complex humanoid scenarios
- Balance and stability considerations in navigation
- Real-world deployment considerations

Continue to the next section to learn about integration with Isaac Sim and Isaac ROS.