# Gazebo Setup Guide

## Prerequisites

Before setting up Gazebo for physics simulation, ensure you have the following installed:

- Ubuntu 22.04 LTS (recommended) or equivalent Linux distribution
- ROS 2 Humble Hawksbill installed
- At least 8GB of RAM
- A graphics card that supports OpenGL 2.1 or higher
- Sufficient disk space (at least 2GB for Gazebo and models)

## Installing Gazebo

Gazebo Harmonic is the recommended version for this module. You can install it using the following steps:

### Step 1: Add the ROS 2 Gazebo Repository

```bash
# Add the Gazebo repository
sudo apt update && sudo apt install -y wget
sudo wget -O /usr/share/keyrings/gazebo-humble-focal.gpg https://packages.osrfoundation.org/gazebo.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-humble-focal.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
```

### Step 2: Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic
```

### Step 3: Verify Installation

```bash
gz sim --version
```

You should see the version information for Gazebo Harmonic.

## Setting up the Environment

### ROS 2 Integration

For proper integration with ROS 2 Humble Hawksbill:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Environment Variables

Add the following to your `~/.bashrc` file for persistent setup:

```bash
# Gazebo settings
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-6/plugins
export GZ_SIM_RESOURCE_PATH=/usr/share/gz-sim-6/worlds
```

## Basic Gazebo Launch

To test your installation, launch Gazebo with the empty world:

```bash
gz sim
```

This should open the Gazebo GUI with an empty world. You can use the Insert tab to add models to the world.

## Troubleshooting Common Issues

### Issue: Gazebo fails to start with graphics errors

**Solution**: Ensure your graphics drivers are properly installed. For NVIDIA cards:
```bash
sudo apt install nvidia-driver-XXX  # Replace XXX with your driver version
```

### Issue: Performance is slow or laggy

**Solution**:
- Reduce the physics update rate in the Gazebo settings
- Close other applications to free up RAM
- Check that hardware acceleration is enabled

### Issue: ROS 2 plugins not loading

**Solution**: Verify that you've sourced the ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

## Next Steps

Once you have successfully set up Gazebo, proceed to the [Basic Humanoid Model Creation](./basic-models.md) tutorial to learn how to create and simulate humanoid robots in the environment.