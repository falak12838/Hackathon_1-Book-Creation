# Sensor Configuration in Isaac Sim

## Overview of Sensor Types

Isaac Sim provides comprehensive sensor simulation capabilities that are essential for humanoid robot perception. Proper sensor configuration ensures that synthetic data closely matches real-world sensor outputs, enabling effective training of perception algorithms.

### Available Sensor Types

Isaac Sim supports various sensor types:

- **RGB Cameras**: Visual perception and image capture
- **Depth Cameras**: 3D reconstruction and depth estimation
- **LiDAR**: 3D mapping and obstacle detection
- **IMU (Inertial Measurement Unit)**: Orientation and acceleration
- **Force/Torque Sensors**: Contact detection and manipulation
- **GPS**: Global positioning (in outdoor scenarios)
- **Semantic Segmentation**: Pixel-level object classification

## Camera Configuration

### RGB Camera Setup

Configuring RGB cameras for photorealistic output:

```python
from omni.isaac.sensor import Camera
import numpy as np

# Create a camera with specific parameters
camera = Camera(
    prim_path="/World/Robot/Camera",
    frequency=30,  # Hz
    resolution=(1280, 720),  # HD resolution
    position=np.array([0.0, 0.0, 0.5]),  # Position relative to robot
    orientation=np.array([0, 0, 0, 1])  # Quaternion (w, x, y, z)
)

# Configure camera intrinsic parameters
camera.config_intrinsic_matrix(
    focal_length_x=640.0,  # pixels
    focal_length_y=640.0,  # pixels
    principal_point_x=640.0,  # pixels
    principal_point_y=360.0   # pixels
)

# Configure distortion parameters (if needed)
camera.config_distortion(
    k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0  # No distortion
)
```

### Depth Camera Configuration

Depth cameras are crucial for 3D perception:

```python
from omni.isaac.sensor import Camera

# Create depth camera
depth_camera = Camera(
    prim_path="/World/Robot/DepthCamera",
    frequency=30,
    resolution=(640, 480),
    position=np.array([0.0, 0.0, 0.5])
)

# Enable depth data capture
depth_camera.add_data_listener(
    "depth",
    lambda data: process_depth_data(data)
)

def process_depth_data(depth_data):
    """Process depth data from camera"""
    # Convert to depth in meters
    depth_in_meters = depth_data * 100.0  # Scale factor may vary
    return depth_in_meters
```

### Camera Attachment to Robot

Properly attach cameras to robot models:

```python
# Attach camera to robot link
camera.attach(prim_path="/World/Robot/base_link")

# Configure joint transforms if needed
camera.set_world_pose(
    position=np.array([0.1, 0.0, 0.3]),  # Offset from base
    orientation=np.array([0.707, 0, 0, 0.707])  # Looking forward
)
```

## LiDAR Configuration

### 3D LiDAR Setup

Configuring 3D LiDAR for comprehensive environment mapping:

```python
from omni.isaac.range_sensor import _range_sensor
import omni

# Create LiDAR sensor
lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
lidar_config = {
    "lidarPrim": "/World/Robot/LiDAR",
    "sensorPeriod": 0.1,  # 10Hz
    "samplesPerScan": 1080,  # Points per revolution
    "rotationFrequency": 10,  # Hz
    "horizontalOpeningAngle": 3.14159,  # 180 degrees
    "verticalOpeningAngle": 0.5236,  # 30 degrees
    "maxRange": 25.0,  # Maximum range in meters
    "minRange": 0.1,   # Minimum range in meters
}

# Create the LiDAR sensor
lidar_path = "/World/Robot/LiDAR"
lidar_interface.create_lidar_sensor(
    lidar_path,
    translation=(0.0, 0.0, 0.5),  # Position
    orientation=(1.0, 0.0, 0.0, 0.0),  # Rotation
    config=lidar_config
)
```

### 2D LiDAR Configuration

For simpler navigation tasks:

```python
# 2D LiDAR configuration
lidar_2d_config = {
    "lidarPrim": "/World/Robot/LiDAR2D",
    "sensorPeriod": 0.05,  # 20Hz
    "samplesPerScan": 360,  # 360 points for 360-degree scan
    "rotationFrequency": 10,
    "horizontalOpeningAngle": 6.2832,  # 360 degrees
    "verticalOpeningAngle": 0.01,  # Very narrow vertical beam
    "maxRange": 20.0,
    "minRange": 0.05,
}
```

## IMU Configuration

### IMU Sensor Setup

Configuring IMU sensors for orientation and motion detection:

```python
from omni.isaac.core.sensors import ImuSensor

# Create IMU sensor
imu = ImuSensor(
    prim_path="/World/Robot/IMU",
    position=np.array([0.0, 0.0, 0.2]),  # Mount on robot body
    frequency=100  # 100Hz for high-frequency updates
)

# IMU provides:
# - Linear acceleration
# - Angular velocity
# - Orientation (if enabled)
```

### IMU Data Processing

Processing IMU data for humanoid robot applications:

```python
def process_imu_data(imu_sensor):
    """Process IMU data for humanoid robot"""
    # Get linear acceleration
    linear_acc = imu_sensor.get_linear_acceleration()

    # Get angular velocity
    angular_vel = imu_sensor.get_angular_velocity()

    # Get orientation (if available)
    orientation = imu_sensor.get_orientation()

    # Process for balance control
    return {
        'linear_acceleration': linear_acc,
        'angular_velocity': angular_vel,
        'orientation': orientation
    }
```

## Multi-Sensor Integration

### Synchronizing Multiple Sensors

Properly synchronize multiple sensors for coherent perception:

```python
import time

class MultiSensorFusion:
    def __init__(self):
        self.camera_data = None
        self.lidar_data = None
        self.imu_data = None
        self.timestamp = None

    def update_sensors(self):
        """Update all sensors and synchronize timestamps"""
        current_time = time.time()

        # Get data from all sensors
        self.camera_data = self.get_camera_data()
        self.lidar_data = self.get_lidar_data()
        self.imu_data = self.get_imu_data()

        self.timestamp = current_time

        # Verify temporal coherence
        return self.synchronize_data()

    def synchronize_data(self):
        """Ensure all sensor data is properly synchronized"""
        # Implement synchronization logic
        # This might involve interpolation or buffering
        return {
            'timestamp': self.timestamp,
            'camera': self.camera_data,
            'lidar': self.lidar_data,
            'imu': self.imu_data
        }
```

### Sensor Mounting and Positioning

Proper sensor placement on humanoid robots:

```python
# Typical sensor placement for humanoid robots
sensor_configurations = {
    'head_camera': {
        'position': [0.0, 0.0, 0.8],  # On head/upper body
        'orientation': [0.707, 0, 0, 0.707],  # Looking forward
        'type': 'rgb_camera'
    },
    'chest_lidar': {
        'position': [0.0, 0.0, 0.5],  # On chest area
        'orientation': [1.0, 0, 0, 0],  # Looking forward
        'type': 'lidar'
    },
    'imu_torso': {
        'position': [0.0, 0.0, 0.4],  # Center of mass area
        'orientation': [1.0, 0, 0, 0],
        'type': 'imu'
    }
}
```

## Sensor Data Processing Pipeline

### Real-time Data Processing

Setting up real-time sensor data processing:

```python
from omni.isaac.core.utils.prims import get_prim_at_path

class SensorDataProcessor:
    def __init__(self, camera, lidar, imu):
        self.camera = camera
        self.lidar = lidar
        self.imu = imu

        # Set up data listeners
        self.camera.add_data_listener("rgb", self.process_rgb_data)
        self.lidar.add_data_listener("lidar", self.process_lidar_data)
        self.imu.add_data_listener("imu", self.process_imu_data)

    def process_rgb_data(self, data):
        """Process RGB camera data"""
        image = data.get("data")
        # Apply perception algorithms
        # Detect objects, estimate depth, etc.
        return image

    def process_lidar_data(self, data):
        """Process LiDAR point cloud data"""
        points = data.get("data")
        # Process point cloud
        # Extract features, detect obstacles, etc.
        return points

    def process_imu_data(self, data):
        """Process IMU data for balance"""
        imu_reading = data.get("data")
        # Calculate orientation, detect falls, etc.
        return imu_reading
```

## Sensor Validation

### Comparing Synthetic to Real Data

Validate sensor configurations by comparing to real sensor data:

```python
def validate_sensor_data(synthetic_data, real_data):
    """Validate synthetic sensor data against real data"""

    # Compare statistical properties
    synthetic_stats = calculate_statistics(synthetic_data)
    real_stats = calculate_statistics(real_data)

    # Calculate similarity metrics
    similarity = calculate_similarity(synthetic_stats, real_stats)

    # Validate within acceptable thresholds
    if similarity > 0.8:  # 80% similarity threshold
        return True
    else:
        return False

def calculate_statistics(data):
    """Calculate statistical properties of sensor data"""
    return {
        'mean': np.mean(data),
        'std': np.std(data),
        'min': np.min(data),
        'max': np.max(data),
        'histogram': np.histogram(data, bins=50)
    }
```

## Performance Optimization

### Sensor Performance Considerations

Optimize sensor performance for real-time applications:

1. **Resolution Management**: Balance quality with performance
2. **Update Frequency**: Match sensor frequency to application needs
3. **Data Processing**: Optimize data processing pipelines
4. **GPU Utilization**: Leverage GPU for sensor simulation

### Efficient Data Handling

```python
# Use buffering for efficient data handling
import queue

class SensorBuffer:
    def __init__(self, buffer_size=10):
        self.buffer = queue.Queue(maxsize=buffer_size)

    def add_data(self, data):
        """Add sensor data to buffer"""
        if not self.buffer.full():
            self.buffer.put(data)
        else:
            # Remove oldest data if buffer full
            self.buffer.get()
            self.buffer.put(data)

    def get_latest_data(self):
        """Get most recent sensor data"""
        if not self.buffer.empty():
            # Get latest data by clearing buffer
            latest = None
            while not self.buffer.empty():
                latest = self.buffer.get()
            return latest
        return None
```

## Best Practices

### Sensor Configuration Best Practices

1. **Match Real Sensors**: Configure synthetic sensors to match real hardware specifications
2. **Calibration**: Ensure proper calibration of all sensors
3. **Validation**: Regularly validate synthetic data against real data
4. **Documentation**: Document all sensor configurations for reproducibility

### Troubleshooting Common Issues

- **Data Synchronization**: Ensure all sensors have consistent timestamps
- **Coordinate Systems**: Verify consistent coordinate system usage
- **Performance**: Monitor simulation performance with multiple sensors active
- **Noise Modeling**: Include realistic noise models in sensor simulation

## Next Steps

With sensor configuration understood, you're ready to explore:

- Perception algorithm training using synthetic sensor data
- Sensor fusion techniques for humanoid robots
- Validation of perception systems in simulation
- Integration with navigation and path planning systems

Continue to the next section to learn about simulation environment modeling.