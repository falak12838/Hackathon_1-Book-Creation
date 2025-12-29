# Isaac Sim Setup and Installation

## System Requirements

Before installing Isaac Sim, ensure your system meets the following requirements:

### Hardware Requirements
- **CPU**: Intel/AMD multi-core processor (8+ cores recommended)
- **GPU**: NVIDIA RTX series GPU with CUDA support (RTX 3080/4080 or better recommended)
- **RAM**: 32GB+ system memory
- **Storage**: 50GB+ available disk space
- **OS**: Ubuntu 22.04 LTS (recommended) or Windows 10/11

### Software Requirements
- NVIDIA GPU drivers (520+ recommended)
- CUDA 11.8 or 12.x
- Isaac Sim compatible with your ROS 2 distribution (Humble Hawksbill)

## Installing Isaac Sim

### Option 1: Isaac Sim Docker (Recommended)

The easiest way to get started with Isaac Sim is using the pre-built Docker containers:

```bash
# Pull the latest Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:latest

# Run Isaac Sim with GPU support
docker run --gpus all -it --rm \
  --network=host \
  --env "NVIDIA_VISIBLE_DEVICES=all" \
  --env "NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume $HOME/isaac-sim-cache:/isaac-sim/cache/kit \
  --volume $HOME/isaac-sim-logs:/isaac-sim/logs \
  --volume $HOME/isaac-sim-assets:/isaac-sim/assets \
  --volume $HOME/isaac-sim-examples:/isaac-sim/examples \
  nvcr.io/nvidia/isaac-sim:latest
```

### Option 2: Isaac Sim Standalone

For more advanced use cases, you can install Isaac Sim standalone:

1. **Download Isaac Sim** from the [NVIDIA Developer website](https://developer.nvidia.com/isaac-sim)
2. **Extract and Install** following the official installation guide
3. **Activate License** using your NVIDIA Developer account

## ROS 2 Integration Setup

To integrate Isaac Sim with ROS 2 Humble Hawksbill:

### Install Isaac ROS Packages

```bash
# Create a new workspace for Isaac ROS
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Clone Isaac ROS packages
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git src/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git src/isaac_ros_visual_slam
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git src/isaac_ros_image_pipeline

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

### Source the Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/isaac_ws/install/setup.bash
```

## Verification

To verify your Isaac Sim installation:

1. **Launch Isaac Sim**:
   - If using Docker, Isaac Sim should start automatically
   - If standalone, launch Isaac Sim from your applications menu

2. **Test Basic Simulation**:
   - Create a new stage in Isaac Sim
   - Add a simple robot (e.g., a cube)
   - Run the simulation to ensure physics are working

3. **Test ROS 2 Connection**:
   ```bash
   # In a new terminal
   source ~/isaac_ws/install/setup.bash
   ros2 topic list
   ```

## Common Issues and Troubleshooting

### GPU/CUDA Issues
- Ensure NVIDIA drivers are properly installed
- Verify CUDA version compatibility
- Check that GPU is properly detected with `nvidia-smi`

### Isaac Sim Licensing
- Ensure your Isaac Sim license is active
- Verify your NVIDIA Developer account credentials

### ROS 2 Connection Issues
- Check that ROS_DOMAIN_ID is consistent across all terminals
- Verify network configuration if running Isaac Sim in Docker

## Next Steps

Once you have Isaac Sim properly installed and configured, you can proceed to:

- Explore basic simulation concepts
- Configure sensors for your humanoid robot
- Create your first photorealistic environment
- Run perception training scenarios

Continue to the next section to learn about basic simulation concepts in Isaac Sim.