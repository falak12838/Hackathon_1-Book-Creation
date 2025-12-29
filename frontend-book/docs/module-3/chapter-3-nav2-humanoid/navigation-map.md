# Navigation Map Entities in Nav2 for Humanoid Robots

## Overview

Navigation maps in Nav2 for humanoid robots represent the environment in a format suitable for path planning and navigation. These maps must account for the specific requirements of bipedal locomotion, including step constraints, balance considerations, and terrain passability. This section details the navigation map entities and their role in humanoid robot navigation.

Based on the data model from our specification, navigation maps contain various components that must be properly configured for effective humanoid robot navigation.

## Core Navigation Map Entities

### Map Representation Types

#### Occupancy Grid Maps
- **Grid Structure**: 2D grid representing free, occupied, and unknown spaces
- **Resolution**: Grid cell size determining navigation precision
- **Coordinates**: World coordinates mapping to grid indices
- **Probabilities**: Occupancy probabilities for each cell

```python
# Occupancy grid map structure
class OccupancyGridMap:
    def __init__(self, width, height, resolution, origin):
        self.width = width      # Number of cells in x direction
        self.height = height    # Number of cells in y direction
        self.resolution = resolution  # Meters per cell
        self.origin = origin    # Origin in world coordinates [x, y, theta]
        self.data = [0] * (width * height)  # Occupancy data (-1: unknown, 0: free, 100: occupied)

    def world_to_map(self, x_world, y_world):
        """Convert world coordinates to map indices"""
        x_idx = int((x_world - self.origin[0]) / self.resolution)
        y_idx = int((y_world - self.origin[1]) / self.resolution)
        return x_idx, y_idx

    def map_to_world(self, x_idx, y_idx):
        """Convert map indices to world coordinates"""
        x_world = x_idx * self.resolution + self.origin[0]
        y_world = y_idx * self.resolution + self.origin[1]
        return x_world, y_world

    def is_cell_passable(self, x_idx, y_idx):
        """Check if a cell is passable for humanoid navigation"""
        if not (0 <= x_idx < self.width and 0 <= y_idx < self.height):
            return False  # Out of bounds

        occupancy = self.data[y_idx * self.width + x_idx]

        # For humanoid robots, consider additional constraints
        if occupancy > 50:  # Occupied or highly uncertain
            return False

        return True

    def get_cell_cost(self, x_idx, y_idx):
        """Get navigation cost for a cell considering humanoid constraints"""
        if not (0 <= x_idx < self.width and 0 <= y_idx < self.height):
            return float('inf')  # Infinite cost for out-of-bounds

        base_cost = self.data[y_idx * self.width + x_idx]

        # Apply humanoid-specific cost modifications
        # Consider step constraints, balance, and terrain
        humanoid_cost = self.calculate_humanoid_cost_modifiers(x_idx, y_idx, base_cost)

        return base_cost + humanoid_cost
```

#### Costmap Extensions for Humanoid Robots

```python
# Humanoid-specific costmap
class HumanoidCostmap(OccupancyGridMap):
    def __init__(self, width, height, resolution, origin):
        super().__init__(width, height, resolution, origin)
        self.step_constraints = HumanoidStepConstraints()
        self.footprint_radius = 0.2  # Humanoid foot circular approximation
        self.inflation_radius = 0.5  # Safety margin inflation

    def calculate_humanoid_cost_modifiers(self, x_idx, y_idx, base_cost):
        """Calculate humanoid-specific cost modifiers"""
        modifier = 0

        # Consider terrain roughness
        terrain_cost = self.estimate_terrain_roughness_cost(x_idx, y_idx)
        modifier += terrain_cost

        # Consider step feasibility to adjacent cells
        step_feasibility_cost = self.estimate_step_feasibility_cost(x_idx, y_idx)
        modifier += step_feasibility_cost

        # Consider balance requirements
        balance_cost = self.estimate_balance_cost(x_idx, y_idx)
        modifier += balance_cost

        return modifier

    def estimate_terrain_roughness_cost(self, x_idx, y_idx):
        """Estimate cost based on terrain roughness"""
        # Analyze local terrain characteristics
        # Higher cost for rough or uneven terrain
        pass

    def estimate_step_feasibility_cost(self, x_idx, y_idx):
        """Estimate cost based on step feasibility from adjacent cells"""
        # Calculate how feasible it is to step to this cell from neighbors
        # considering humanoid step constraints
        pass

    def estimate_balance_cost(self, x_idx, y_idx):
        """Estimate cost based on balance requirements"""
        # Consider balance requirements for this location
        # Higher cost for locations that would compromise balance
        pass
```

### Path Planning Considerations

#### Humanoid-Specific Path Constraints
- **Step Length Limits**: Maximum distance between consecutive waypoints
- **Step Height Limits**: Maximum height difference between waypoints
- **Turning Constraints**: Turning radius and angular constraints
- **Footprint Planning**: Consider robot's physical footprint

```python
# Path planning with humanoid constraints
class HumanoidPathPlanner:
    def __init__(self, costmap):
        self.costmap = costmap
        self.step_constraints = HumanoidStepConstraints()

    def plan_path(self, start, goal):
        """Plan path considering humanoid constraints"""
        # Use A* or other path planning algorithm
        # with humanoid-specific cost function
        path = self.a_star_search(start, goal)
        return self.validate_and_smooth_path(path)

    def a_star_search(self, start, goal):
        """A* search with humanoid constraints"""
        start_idx = self.costmap.world_to_map(start.x, start.y)
        goal_idx = self.costmap.world_to_map(goal.x, goal.y)

        # Initialize open and closed sets
        open_set = [(0, start_idx)]  # (f_score, position)
        came_from = {}
        g_score = {start_idx: 0}
        f_score = {start_idx: self.heuristic(start_idx, goal_idx)}

        import heapq
        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_idx:
                # Reconstruct path
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors_with_constraints(current):
                tentative_g_score = g_score[current] + self.step_cost(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def get_neighbors_with_constraints(self, pos):
        """Get valid neighbors considering humanoid constraints"""
        x, y = pos
        neighbors = []

        # Check 8-connected neighborhood
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current cell

                nx, ny = x + dx, y + dy

                # Check if neighbor is within bounds
                if not (0 <= nx < self.costmap.width and 0 <= ny < self.costmap.height):
                    continue

                # Check if step to neighbor is feasible for humanoid
                if self.is_step_feasible(pos, (nx, ny)):
                    neighbors.append((nx, ny))

        return neighbors

    def is_step_feasible(self, from_pos, to_pos):
        """Check if step from one position to another is feasible for humanoid"""
        from_world = self.costmap.map_to_world(*from_pos)
        to_world = self.costmap.map_to_world(*to_pos)

        # Calculate step distance
        step_distance = math.sqrt(
            (to_world[0] - from_world[0])**2 + (to_world[1] - from_world[1])**2
        )

        # Check step length constraint
        if step_distance > self.step_constraints.max_step_length:
            return False

        # Check if destination cell is passable
        if not self.costmap.is_cell_passable(*to_pos):
            return False

        # Check cost threshold
        cell_cost = self.costmap.get_cell_cost(*to_pos)
        if cell_cost > self.step_constraints.max_passable_cost:
            return False

        return True
```

## Humanoid-Specific Map Features

### Step-Aware Map Representation

#### Discrete Footstep Planning
- **Footstep Graph**: Graph representation for discrete footstep planning
- **Support Regions**: Areas where feet can be placed for stability
- **ZMP Constraints**: Zero Moment Point constraints for balance
- **Gait Patterns**: Precomputed gait pattern feasibility

```python
# Step-aware map representation
class StepAwareMap:
    def __init__(self, base_map, humanoid_config):
        self.base_map = base_map
        self.humanoid_config = humanoid_config
        self.footstep_graph = self.build_footstep_graph()
        self.support_regions = self.compute_support_regions()
        self.zmp_constraints = self.compute_zmp_constraints()

    def build_footstep_graph(self):
        """Build graph for discrete footstep planning"""
        graph = {}

        # For each passable cell in the base map, determine if it's a valid footstep location
        for y in range(self.base_map.height):
            for x in range(self.base_map.width):
                if self.base_map.is_cell_passable(x, y):
                    # Check if this cell can support a footstep
                    if self.is_valid_footstep_location(x, y):
                        neighbors = self.get_valid_step_neighbors(x, y)
                        graph[(x, y)] = neighbors

        return graph

    def is_valid_footstep_location(self, x, y):
        """Check if a location is valid for footstep placement"""
        # Check if the cell and surrounding area are suitable for foot placement
        # Consider: surface flatness, obstacle clearance, terrain type

        # Check base map passability
        if not self.base_map.is_cell_passable(x, y):
            return False

        # Check for obstacles in foot area
        foot_radius = self.humanoid_config['foot_radius']
        for dx in range(-int(foot_radius/self.base_map.resolution),
                        int(foot_radius/self.base_map.resolution) + 1):
            for dy in range(-int(foot_radius/self.base_map.resolution),
                            int(foot_radius/self.base_map.resolution) + 1):
                if dx*dx + dy*dy <= (foot_radius/self.base_map.resolution)**2:
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.base_map.width and 0 <= ny < self.base_map.height):
                        if self.base_map.data[ny * self.base_map.width + nx] > 50:
                            return False  # Obstacle in foot area

        return True

    def get_valid_step_neighbors(self, x, y):
        """Get valid step neighbors for footstep planning"""
        neighbors = []

        # Consider all possible step locations within step constraints
        max_steps = int(self.humanoid_config['max_step_length'] / self.base_map.resolution)

        for dx in range(-max_steps, max_steps + 1):
            for dy in range(-max_steps, max_steps + 1):
                if dx == 0 and dy == 0:
                    continue  # Skip current position

                step_distance = math.sqrt(dx*dx + dy*dy) * self.base_map.resolution
                if step_distance <= self.humanoid_config['max_step_length']:
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.base_map.width and 0 <= ny < self.base_map.height):
                        if self.is_valid_footstep_location(nx, ny):
                            neighbors.append((nx, ny))

        return neighbors

    def compute_support_regions(self):
        """Compute regions that provide stable support for humanoid"""
        # Identify areas where the humanoid can maintain stable support
        # This involves analyzing terrain for balance constraints
        pass

    def compute_zmp_constraints(self):
        """Compute Zero Moment Point constraints for the map"""
        # Calculate ZMP constraints for different regions of the map
        # This helps determine where the humanoid can maintain balance
        pass
```

### Terrain Classification Map

#### Terrain Types for Humanoid Navigation
- **Flat Ground**: Normal walking terrain
- **Rough Terrain**: Uneven surfaces requiring careful navigation
- **Sloped Terrain**: Inclined surfaces with balance challenges
- **Narrow Passages**: Areas requiring precise foot placement
- **Stairs/Steps**: Vertical obstacles requiring special handling

```python
# Terrain classification map
class TerrainClassificationMap:
    def __init__(self, base_map):
        self.base_map = base_map
        self.terrain_types = self.classify_terrain()

    def classify_terrain(self):
        """Classify terrain types across the map"""
        terrain_map = [[None for _ in range(self.base_map.width)] for _ in range(self.base_map.height)]

        for y in range(self.base_map.height):
            for x in range(self.base_map.width):
                terrain_map[y][x] = self.classify_cell_terrain(x, y)

        return terrain_map

    def classify_cell_terrain(self, x, y):
        """Classify terrain type for a specific cell"""
        # Analyze local neighborhood for terrain characteristics
        neighborhood = self.get_neighborhood_elevation(x, y)

        # Calculate terrain roughness
        roughness = self.calculate_roughness(neighborhood)

        # Calculate slope
        slope = self.calculate_slope(neighborhood)

        # Classify based on characteristics
        if slope < 0.1 and roughness < 0.05:
            return 'flat'
        elif slope < 0.3 and roughness < 0.15:
            return 'slightly_rough'
        elif slope < 0.5 and roughness < 0.2:
            return 'rough'
        elif slope >= 0.5:
            return 'sloped'
        else:
            return 'obstacle'  # Too rough to navigate

    def get_neighborhood_elevation(self, x, y, radius=2):
        """Get elevation data for neighborhood around cell"""
        # This would typically come from elevation map or 3D sensor data
        # For now, we'll simulate it
        neighborhood = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.base_map.width and 0 <= ny < self.base_map.height:
                    # Simulate elevation data
                    elevation = self.simulate_elevation(nx, ny)
                    neighborhood.append(elevation)

        return neighborhood

    def calculate_roughness(self, neighborhood):
        """Calculate terrain roughness from elevation data"""
        if len(neighborhood) < 2:
            return 0

        mean_elevation = sum(neighborhood) / len(neighborhood)
        variance = sum((e - mean_elevation)**2 for e in neighborhood) / len(neighborhood)
        return math.sqrt(variance)

    def calculate_slope(self, neighborhood):
        """Calculate terrain slope from elevation data"""
        # Simplified slope calculation
        # In practice, this would use more sophisticated surface normal calculation
        if len(neighborhood) < 2:
            return 0

        # Calculate maximum height difference in neighborhood
        max_height = max(neighborhood)
        min_height = min(neighborhood)

        # Normalize by neighborhood size (simplified)
        return (max_height - min_height) / 2.0  # Adjust normalization as needed

    def simulate_elevation(self, x, y):
        """Simulate elevation data for demonstration"""
        # This would normally come from actual elevation data
        # For simulation, create some variation
        import random
        return random.uniform(0, 0.2)  # Random elevation up to 20cm
```

## Map Integration with Navigation

### Multi-Layer Map System

#### Global and Local Maps
- **Global Map**: Large-scale map for global path planning
- **Local Map**: High-resolution map for local navigation
- **Dynamic Updates**: Real-time updates from sensors
- **Multi-Resolution**: Different resolutions for different purposes

```python
# Multi-layer map system
class MultiLayerMapSystem:
    def __init__(self):
        self.global_map = None
        self.local_map = None
        self.elevation_map = None
        self.dynamic_objects_map = None

    def update_maps(self, sensor_data):
        """Update all map layers with new sensor data"""
        # Update global map with new information
        self.update_global_map(sensor_data)

        # Update local map with high-resolution data
        self.update_local_map(sensor_data)

        # Update elevation map if available
        if 'elevation' in sensor_data:
            self.update_elevation_map(sensor_data['elevation'])

        # Update dynamic objects map
        self.update_dynamic_objects_map(sensor_data)

    def update_global_map(self, sensor_data):
        """Update global map with new sensor information"""
        # Process sensor data to update global map
        # This might involve map registration, loop closure, etc.
        pass

    def update_local_map(self, sensor_data):
        """Update local map with high-resolution sensor data"""
        # Update local costmap with latest sensor readings
        # This is typically done by Nav2's costmap_2d
        pass

    def update_elevation_map(self, elevation_data):
        """Update elevation map with new elevation data"""
        # Process elevation data to update 3D terrain representation
        pass

    def update_dynamic_objects_map(self, sensor_data):
        """Update map of dynamic objects"""
        # Track and predict movement of dynamic obstacles
        pass

    def get_navigation_map(self, robot_pose, planning_type='global'):
        """Get appropriate map for navigation planning"""
        if planning_type == 'global':
            return self.global_map
        elif planning_type == 'local':
            return self.local_map
        elif planning_type == 'footstep':
            return self.get_footstep_suitable_map()
        else:
            return self.global_map

    def get_footstep_suitable_map(self):
        """Get map suitable for footstep planning"""
        # Combine multiple map layers to create footstep planning map
        # Consider terrain type, elevation, obstacles, and stability
        pass
```

### Map Validation and Quality Assessment

#### Map Quality Metrics
- **Completeness**: Coverage of navigable areas
- **Accuracy**: Precision of obstacle and free space representation
- **Consistency**: Temporal consistency of map features
- **Resolution**: Adequacy of map resolution for humanoid navigation

```python
# Map quality assessment
class MapQualityAssessment:
    def __init__(self, map_system):
        self.map_system = map_system
        self.quality_metrics = {}

    def assess_map_quality(self):
        """Assess quality of navigation maps"""
        metrics = {}

        # Assess global map quality
        metrics['global_completeness'] = self.assess_completeness(self.map_system.global_map)
        metrics['global_accuracy'] = self.assess_accuracy(self.map_system.global_map)
        metrics['global_resolution'] = self.assess_resolution_suitability(self.map_system.global_map)

        # Assess local map quality
        metrics['local_accuracy'] = self.assess_local_accuracy()
        metrics['local_consistency'] = self.assess_temporal_consistency()

        # Assess humanoid-specific suitability
        metrics['step_feasibility'] = self.assess_step_feasibility()
        metrics['balance_suitability'] = self.assess_balance_suitability()

        self.quality_metrics = metrics
        return metrics

    def assess_completeness(self, map_data):
        """Assess how complete the map is"""
        unknown_cells = sum(1 for cell in map_data.data if cell == -1)
        total_cells = len(map_data.data)
        return 1.0 - (unknown_cells / total_cells)

    def assess_accuracy(self, map_data):
        """Assess accuracy of map representation"""
        # This would involve comparison with ground truth or sensor validation
        # For now, return a placeholder
        return 0.9  # Placeholder accuracy score

    def assess_resolution_suitability(self, map_data):
        """Assess if map resolution is suitable for humanoid navigation"""
        # Humanoid robots typically need higher resolution than wheeled robots
        # due to precise footstep requirements
        required_resolution = 0.05  # 5cm resolution typically needed
        return map_data.resolution <= required_resolution

    def assess_step_feasibility(self):
        """Assess if map provides sufficient information for step planning"""
        # Check if map contains terrain classification and elevation data
        # needed for humanoid navigation
        pass

    def assess_balance_suitability(self):
        """Assess if map provides information for balance planning"""
        # Check if map contains information about stable surfaces
        # and support regions
        pass
```

## Best Practices

### Map Configuration Guidelines

1. **Resolution Selection**: Choose appropriate resolution for humanoid navigation
2. **Cost Function Design**: Design cost functions that reflect humanoid constraints
3. **Dynamic Updates**: Ensure maps update in real-time for dynamic environments
4. **Validation**: Regularly validate map quality and accuracy
5. **Integration**: Properly integrate maps with path planning and control systems

### Implementation Considerations

- **Memory Management**: Efficiently manage large map data
- **Update Frequency**: Balance update frequency with computational requirements
- **Multi-Resolution**: Use appropriate resolution for different navigation tasks
- **Sensor Fusion**: Properly integrate data from multiple sensors
- **Real-time Performance**: Ensure maps can be updated and queried in real-time

## Integration with Nav2

### Nav2 Map Configuration

```yaml
# Example Nav2 map configuration for humanoid robots
map_server:
  ros__parameters:
    # Map server parameters
    yaml_filename: "humanoid_map.yaml"
    frame: "map"
    topic_name: "map"
    qos_overrides./parameter_events.publisher:
      durability: volatile
      reliability: best_effort

map_saver:
  ros__parameters:
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65
    map_subscribe_transient_local: true

# Humanoid-specific costmap configuration
local_costmap:
  ros__parameters:
    # Higher resolution for precise footstep planning
    resolution: 0.025  # 2.5cm resolution
    # Larger footprint for humanoid safety
    robot_radius: 0.35
    # Extended update frequency for dynamic obstacle tracking
    update_frequency: 10.0
    publish_frequency: 5.0
    width: 6.0
    height: 6.0
    plugins:
      - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
      - {name: voxel_layer, type: "nav2_costmap_2d::VoxelLayer"}
      - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
    inflation_layer:
      # Larger inflation radius for humanoid safety
      inflation_radius: 0.7
      cost_scaling_factor: 6.0

global_costmap:
  ros__parameters:
    # Lower resolution for global planning efficiency
    resolution: 0.1  # 10cm resolution
    robot_radius: 0.3
    update_frequency: 2.0
    publish_frequency: 1.0
    width: 100.0
    height: 100.0
    plugins:
      - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
      - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
      - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
```

## Next Steps

With navigation map entities understood, you're ready to explore:

- Path planning examples with humanoid constraints
- Integration with Isaac Sim for validation
- Advanced navigation strategies for humanoid robots
- Balance and stability considerations in navigation

Continue to the next section to learn about path planning examples for humanoid robots.