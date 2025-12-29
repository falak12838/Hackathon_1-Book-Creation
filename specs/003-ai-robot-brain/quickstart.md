# Quickstart Guide: AI-Robot Brain (NVIDIA Isaac)

## Prerequisites

### Hardware Requirements
- NVIDIA GPU (RTX series or GTX 10xx/20xx/30xx/40xx)
- 16GB+ RAM (32GB recommended)
- 50GB+ free disk space
- Ubuntu 22.04 LTS

### Software Requirements
- NVIDIA GPU drivers (535 or newer)
- CUDA 11.8 or later
- Docker and Docker Compose
- Git

## Setup Steps

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install ROS 2 Humble packages
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Isaac Sim
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow NVIDIA's installation guide for your specific platform
# Ensure Isaac Sim is properly configured with your GPU
```

### 3. Install Isaac ROS Packages
```bash
# Create ROS 2 workspace
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws

# Install Isaac ROS dependencies
sudo apt update
sudo apt install -y ros-humble-isaac-ros-dev-tools

# Clone Isaac ROS packages
cd src
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detectnet.git
git clone -b ros2-humble https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

# Build the workspace
cd ~/isaac_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Install Navigation2
```bash
# Install Nav2 packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Clone Nav2 custom configurations for humanoid robots (if needed)
cd ~/isaac_ws/src
git clone https://github.com/ros-planning/navigation2_tutorials.git
```

### 5. Verify Installation
```bash
# Test Isaac Sim (run a simple simulation)
# Launch Isaac Sim and verify GPU acceleration is working

# Test Isaac ROS packages
cd ~/isaac_ws
source install/setup.bash
ros2 launch isaac_ros_visual_slam visual_slam_node.launch.py

# Test Nav2
cd ~/isaac_ws
source install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

## Running the Examples

### Chapter 1: Isaac Sim & Synthetic Data
```bash
# Navigate to the example directory
cd ~/isaac_ws/src/isaac_ros_examples/synthetic_data_generation

# Run the synthetic data generation example
source ~/isaac_ws/install/setup.bash
python3 generate_synthetic_data.py
```

### Chapter 2: Isaac ROS Perception & Navigation
```bash
# Launch Isaac ROS perception pipeline
source ~/isaac_ws/install/setup.bash
ros2 launch isaac_ros_examples perception_pipeline.launch.py
```

### Chapter 3: Nav2 Humanoid Path Planning
```bash
# Launch Nav2 with humanoid-specific configurations
source ~/isaac_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## Troubleshooting

### Common Issues
- **GPU not detected**: Ensure NVIDIA drivers and CUDA are properly installed
- **Isaac Sim fails to launch**: Check GPU compatibility and driver version
- **ROS packages not found**: Verify workspace is sourced properly
- **Performance issues**: Ensure sufficient RAM and GPU memory

### Verification Commands
```bash
# Check GPU
nvidia-smi

# Check ROS 2 installation
ros2 topic list

# Check Isaac ROS packages
ros2 pkg list | grep isaac
```

## Next Steps
1. Follow the three chapters in sequence
2. Complete the hands-on tutorials
3. Experiment with custom environments
4. Integrate with your humanoid robot platform