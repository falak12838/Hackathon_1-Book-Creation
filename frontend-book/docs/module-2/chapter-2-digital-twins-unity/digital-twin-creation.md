# Digital Twin Creation Tutorial

## Overview

This tutorial will guide you through creating a high-fidelity digital twin of a humanoid robot in Unity. You'll learn to build a virtual representation that mirrors the physical properties and behaviors established in Chapter 1.

## Prerequisites

- Unity 2022.3 LTS installed and configured
- Basic understanding of Unity interface and GameObjects
- Completion of the Unity Setup Guide

## Creating the Basic Humanoid Structure

### Step 1: Setting Up the Scene

1. Create a new 3D scene in Unity
2. Remove the default "SampleScene" GameObjects (Main Camera, Directional Light)
3. Add a new camera: GameObject → 3D Object → Camera
4. Position the camera at (0, 2, -5) to get a good view of the robot
5. Add lighting: GameObject → Light → Directional Light
6. Set the light rotation to (50, -30, 0) for good illumination

### Step 2: Creating the Robot Body

We'll create the humanoid structure using basic 3D primitives:

1. Create the torso:
   - GameObject → 3D Object → Cube
   - Name it "Torso"
   - Scale it to (0.3, 0.5, 0.2)
   - Position it at (0, 0.5, 0)

2. Create the head:
   - GameObject → 3D Object → Sphere
   - Name it "Head"
   - Scale it to (0.2, 0.2, 0.2)
   - Position it at (0, 1.0, 0)
   - Make it a child of the Torso by dragging it onto the Torso in the Hierarchy

3. Create the left leg:
   - GameObject → 3D Object → Capsule
   - Name it "LeftLeg"
   - Rotate it 90 degrees on the X-axis (to make it vertical)
   - Scale it to (0.15, 0.3, 0.15) - adjust the Y scale to change length
   - Position it at (-0.15, -0.3, 0)
   - Make it a child of the Torso

4. Create the right leg (mirror the left leg):
   - GameObject → 3D Object → Capsule
   - Name it "RightLeg"
   - Rotate it 90 degrees on the X-axis
   - Scale it to (0.15, 0.3, 0.15)
   - Position it at (0.15, -0.3, 0)
   - Make it a child of the Torso

5. Create the left arm:
   - GameObject → 3D Object → Capsule
   - Name it "LeftArm"
   - Rotate it 90 degrees on the X-axis
   - Scale it to (0.1, 0.25, 0.1)
   - Position it at (0, 0.3, -0.1)
   - Make it a child of the Torso

6. Create the right arm (mirror the left arm):
   - GameObject → 3D Object → Capsule
   - Name it "RightArm"
   - Rotate it 90 degrees on the X-axis
   - Scale it to (0.1, 0.25, 0.1)
   - Position it at (0, 0.3, 0.1)
   - Make it a child of the Torso

### Step 3: Adding Materials and Visual Polish

1. Create materials for different body parts:
   - In the Project window, right-click → Create → Material
   - Create materials named "HeadMaterial", "TorsoMaterial", "LimbMaterial"
   - Set colors: Head (white), Torso (blue), Limbs (red)

2. Apply materials to the GameObjects:
   - Select each body part
   - In the Inspector, find the Mesh Renderer component
   - Drag the appropriate material to the "Materials" slot

## Adding Physics Properties

### Step 1: Adding Colliders

For realistic physics simulation, add colliders to each body part:

1. Select the Torso GameObject
2. Add a Box Collider: Component → Physics → Box Collider
3. Adjust the center and size to match the cube's dimensions

4. Select the Head GameObject
5. Add a Sphere Collider: Component → Physics → Sphere Collider

6. Select each leg GameObject
7. Add a Capsule Collider: Component → Physics → Capsule Collider
8. Adjust the direction to Z-axis and set appropriate radius and height

9. Select each arm GameObject
10. Add a Capsule Collider: Component → Physics → Capsule Collider
11. Adjust the direction and dimensions appropriately

### Step 2: Adding Rigidbodies

Add Rigidbodies to make the parts physically interactive:

1. Select the Torso GameObject
2. Add a Rigidbody: Component → Physics → Rigidbody
3. Set the mass appropriately (e.g., 10 for the torso)
4. Adjust drag and angular drag as needed

5. For connected parts, you may want to use Joints instead of individual Rigidbodies
6. Add Fixed Joints between connected parts if needed

## Creating a Robot Prefab

### Step 1: Convert to Prefab

1. Select the Torso GameObject (which contains all other parts as children)
2. Drag it from the Hierarchy to the Project window to create a prefab
3. Name it "HumanoidRobot.prefab"

### Step 2: Testing the Prefab

1. Delete the original Torso GameObject from the scene
2. Drag the "HumanoidRobot.prefab" from the Project window into the scene
3. Test that all parts are properly connected and physics works

## Adding Basic Animation

### Step 1: Creating Simple Animation

1. Create a new C# script called "SimpleRobotAnimation.cs":
```csharp
using UnityEngine;

public class SimpleRobotAnimation : MonoBehaviour
{
    public float animationSpeed = 1.0f;
    public float movementRange = 0.1f;

    private Vector3 initialPosition;
    private Vector3 initialHeadRotation;

    void Start()
    {
        initialPosition = transform.position;
        if (transform.Find("Head") != null)
        {
            initialHeadRotation = transform.Find("Head").localEulerAngles;
        }
    }

    void Update()
    {
        // Simple up/down movement
        float newY = initialPosition.y + Mathf.Sin(Time.time * animationSpeed) * movementRange;
        transform.position = new Vector3(initialPosition.x, newY, initialPosition.z);

        // Simple head rotation
        if (transform.Find("Head") != null)
        {
            Transform head = transform.Find("Head");
            float headY = initialHeadRotation.y + Mathf.Sin(Time.time * animationSpeed * 0.5f) * 10f;
            head.localEulerAngles = new Vector3(initialHeadRotation.x, headY, initialHeadRotation.z);
        }
    }
}
```

2. Attach the script to the Torso GameObject
3. Adjust the animationSpeed and movementRange values in the Inspector

## Creating the Digital Twin Bridge

### Step 1: Setting Up ROS Communication

To connect with the Gazebo simulation from Chapter 1, we'll use the ROS TCP Connector:

1. Add the ROS TCP Connector to your scene:
   - GameObject → Create Empty
   - Name it "ROSBridge"
   - Add Component → Unity Robotics → ROS TCP Connector

2. Configure the connector:
   - Set the ROS IP to your ROS 2 master IP (usually 127.0.0.1 for local)
   - Set the port to the appropriate ROS port (usually 10000)

### Step 2: Creating a Robot State Synchronizer

Create a script to synchronize robot state between Gazebo and Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class RobotStateSynchronizer : MonoBehaviour
{
    [SerializeField] private string robotName = "simple_humanoid";
    [SerializeField] private string jointStatesTopic = "/joint_states";

    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.JointStateMsg>(jointStatesTopic, OnJointStateReceived);
    }

    void OnJointStateReceived(Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor.JointStateMsg jointState)
    {
        // This is a simplified example - in practice you'd map joint names to Unity joints
        // and update the positions accordingly
    }
}
```

## Exercise: Enhancing the Digital Twin

1. Improve the visual appearance by adding more detailed 3D models instead of primitives
2. Add textures and more realistic materials
3. Implement more complex animations (walking, waving)
4. Add a simple UI to control the robot's movements
5. Create a simple environment for the robot to interact with

## Troubleshooting Common Issues

### Issue: Parts are not moving together properly
**Solution**: Check parenting relationships and use Joints instead of just parenting for physical connections

### Issue: Physics simulation is unstable
**Solution**: Adjust Rigidbody settings, ensure proper mass distribution, and check collision layer settings

### Issue: ROS connection fails
**Solution**: Verify ROS network settings, check that ROS master is running, and confirm IP/port settings

## Next Steps

After creating your basic digital twin, proceed to learn about [HRI Fundamentals](./hri-fundamentals.md) to understand how to create meaningful interactions between humans and robots in the Unity environment.