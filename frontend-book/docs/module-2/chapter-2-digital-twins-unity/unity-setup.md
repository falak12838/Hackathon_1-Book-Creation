# Unity Setup Guide

## Overview

This guide will walk you through setting up Unity for creating high-fidelity digital twins and Human-Robot Interaction (HRI) scenarios. We'll focus on Unity 2022.3 LTS, which provides long-term support and stability for robotics applications.

## Prerequisites

Before installing Unity, ensure your system meets the following requirements:

### System Requirements
- **Operating System**: Windows 10 (64-bit) 1809 or later, macOS 10.15.6+, or Ubuntu 20.04 LTS
- **Processor**: 64-bit processor with SSE2 instruction set support
- **Memory**: 8 GB RAM minimum, 16 GB or more recommended
- **Graphics**: Graphics card with DX10 (shader model 4.0) capabilities
- **Disk Space**: At least 6 GB for Unity Hub and a single Unity Editor installation
- **Additional**: Internet connection for activation and package downloads

### Software Requirements
- .NET Framework 4.7.1 or later (Windows)
- Xcode command line tools (macOS)
- Git version control system

## Installing Unity Hub

Unity Hub is the recommended way to manage multiple Unity versions and projects:

### Step 1: Download Unity Hub
1. Visit the Unity website: https://unity.com/download
2. Download Unity Hub for your operating system
3. Run the installer and follow the on-screen instructions

### Step 2: Create Unity Account
1. Launch Unity Hub
2. Click "Sign in" and create a Unity account if you don't have one
3. The free Personal plan is sufficient for educational purposes

## Installing Unity Editor

### Step 1: Install Unity 2022.3 LTS
1. In Unity Hub, click the "Installs" tab
2. Click "Add" to install a new Unity version
3. Select "2022.3.20f1" (or the latest 2022.3 LTS version)
4. In the installer, select the following modules:
   - **Unity Editor**: Standard or Pro (Standard is free for educational use)
   - **Visual Studio** or **Visual Studio Code** as your code editor
   - **Android Build Support** (if you plan to build for Android)
   - **iOS Build Support** (if you plan to build for iOS)
   - **Windows Build Support** (for Windows builds)
   - **macOS Build Support** (for macOS builds)

### Step 2: Verify Installation
1. After installation completes, click "New Project"
2. Choose the "3D (Built-in Render Pipeline)" template
3. Name your project "DigitalTwinSimulation"
4. Create the project in a suitable location

## Setting Up Robotics-Specific Packages

### Step 1: Install Unity Robotics Package
Unity provides robotics-specific packages that facilitate robot simulation:

1. In Unity, go to Window → Package Manager
2. In the Package Manager window, click the gear icon and select "Add package from git URL..."
3. Enter: `com.unity.robotics.ros-tcp-connector`
4. Click "Add" to install the package

### Step 2: Install Additional Useful Packages
Consider installing these additional packages:
- **Universal Render Pipeline (URP)**: For more advanced rendering
- **Cinemachine**: For advanced camera control
- **ProBuilder**: For quick 3D modeling in Unity

## Configuring Unity for Robotics

### Step 1: Set Physics Time Settings
For accurate physics simulation that matches Gazebo:
1. Go to Edit → Project Settings → Time
2. Set "Fixed Timestep" to 0.0167 (60 FPS) or 0.0083 (120 FPS) to match Gazebo's default
3. Set "Maximum Allowed Timestep" to 0.333
4. Set "Resolution" to 1

### Step 2: Configure Physics Engine
1. Go to Edit → Project Settings → Physics
2. Set "Solver Iterations" to 10-20 for stability
3. Set "Solver Velocity Iterations" to 10-20
4. Adjust "Sleep Threshold" and "Default Contact Offset" as needed

### Step 3: Configure Input System
Unity's new Input System provides better control for HRI:
1. Go to Edit → Project Settings → Input
2. Or install the "Input System" package from Package Manager
3. Configure input actions for robot control (keyboard, mouse, joystick)

## Setting Up Development Environment

### Visual Studio Code Configuration
If using Visual Studio Code, install these extensions:
- **C#** by Microsoft
- **Unity Debug** by Unity Technologies
- **C# Extensions** by Microsoft
- **Unity Snippets** by Sebastian Bierman

### Project Structure
Organize your Unity project with these folders:
```
Assets/
├── Models/           # 3D models for robots and environments
├── Materials/        # Material definitions
├── Textures/         # Texture images
├── Scripts/          # C# scripts for robot control
├── Scenes/           # Unity scene files
├── Prefabs/          # Reusable robot components
├── Plugins/          # External libraries
└── StreamingAssets/  # Files copied to build output
```

## Testing Your Setup

### Step 1: Basic Scene Test
1. Create a new 3D scene (File → New Scene)
2. Add a cube (GameObject → 3D Object → Cube)
3. Add a camera and lighting
4. Play the scene to verify everything works

### Step 2: Robotics Package Test
1. Create a new scene
2. Add the ROS TCP Connector component to your camera or a new GameObject
3. Verify the package is correctly imported and accessible

## Troubleshooting Common Issues

### Issue: Unity fails to start or crashes
**Solution**:
- Update your graphics drivers
- Check that your system meets the minimum requirements
- Try running Unity as administrator (Windows)
- Verify sufficient disk space is available

### Issue: Package installation fails
**Solution**:
- Check your internet connection
- Verify Unity account is properly authenticated
- Try using a different network if behind a corporate firewall

### Issue: Physics simulation is unstable
**Solution**:
- Reduce the Fixed Timestep in Time settings
- Increase Solver Iterations in Physics settings
- Ensure your robot models have proper mass and collision properties

## Next Steps

Once you have successfully set up Unity with the required packages, proceed to the [Digital Twin Creation Tutorial](./digital-twin-creation.md) to learn how to create your first high-fidelity digital twin of a humanoid robot.