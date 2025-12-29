# Humanoid Model Integration

## Overview

This guide covers the integration of sophisticated humanoid models into Unity for digital twin applications. We'll focus on creating realistic humanoid robots that can be controlled and animated properly within Unity.

## Prerequisites

- Completion of Digital Twin Creation Tutorial
- Understanding of HRI fundamentals
- Basic knowledge of Unity's animation system
- Familiarity with humanoid model standards (URDF, SDF)

## Humanoid Model Standards

### Understanding Humanoid Anatomy for Simulation

A standard humanoid robot typically includes:

- **Torso**: Main body containing core systems
- **Head**: Contains cameras, sensors, and display elements
- **Arms**: With shoulder, elbow, and wrist joints
- **Legs**: With hip, knee, and ankle joints
- **Hands/Feet**: End effectors for interaction

### Joint Configuration

Standard humanoid joint configuration includes:
- 6 DOF (Degrees of Freedom) for each arm
- 6 DOF for each leg
- 3 DOF for the head/neck
- 3 DOF for the torso (waist)

## Importing 3D Models

### Step 1: Preparing Your 3D Model

Before importing, ensure your 3D model meets these requirements:
- Properly rigged with a skeleton (armature)
- Correct joint hierarchy
- Appropriate scale (1 unit = 1 meter)
- Separate meshes for different body parts if needed

### Step 2: Importing into Unity

1. Place your model file (FBX, OBJ, etc.) in the Assets/Models folder
2. Select the model in the Project window
3. In the Inspector, under Model tab:
   - Set Scale Factor to appropriate value (usually 1.0 for meter-scale models)
   - Check "Import Animation" if your model has animations
   - Set Animation Type to "Humanoid" for humanoid models
   - Click "Configure..." to map joints if needed

### Step 3: Configuring Humanoid Rig

Unity provides automatic humanoid rig configuration:

1. With your model selected, in the Inspector go to Model tab
2. Set Animation Type to "Humanoid"
3. Click "Configure..." to open the Avatar configuration
4. Unity will attempt to automatically map joints
5. Manually adjust any incorrectly mapped joints using the mapping interface
6. Click "Apply" to save the configuration

## Setting Up Physics for Humanoid Models

### Adding Articulation Bodies

For realistic physics simulation, replace Rigidbodies with Articulation Bodies:

```csharp
using UnityEngine;

public class HumanoidPhysicsSetup : MonoBehaviour
{
    [System.Serializable]
    public class JointConfig
    {
        public Transform jointTransform;
        public ArticulationJoint joint;
        public ArticulationDrive drive;
    }

    [SerializeField] private JointConfig[] jointConfigs;

    void Start()
    {
        SetupArticulationBodies();
    }

    void SetupArticulationBodies()
    {
        foreach (var jointConfig in jointConfigs)
        {
            if (jointConfig.jointTransform != null)
            {
                // Add ArticulationBody if not present
                ArticulationBody body = jointConfig.jointTransform.GetComponent<ArticulationBody>();
                if (body == null)
                    body = jointConfig.jointTransform.gameObject.AddComponent<ArticulationBody>();

                // Configure the articulation body
                body.mass = GetAppropriateMass(jointConfig.jointTransform.name);
                body.linearDamping = 0.05f;
                body.angularDamping = 0.05f;
                body.jointFriction = 0.05f;
                body.useGravity = true;

                // Configure the joint if it exists
                if (jointConfig.joint != null)
                {
                    body.jointType = jointConfig.joint.jointType;

                    // Configure drive if needed
                    if (jointConfig.drive != null)
                    {
                        body.linearLockX = jointConfig.drive.forceLimit > 0 ? ArticulationDofLock.Locked : ArticulationDofLock.Free;
                        // Additional drive configuration...
                    }
                }
            }
        }
    }

    float GetAppropriateMass(string jointName)
    {
        // Return appropriate mass based on joint type
        switch (jointName.ToLower())
        {
            case "head":
                return 2.0f;
            case "torso":
                return 10.0f;
            case "upperarm":
            case "upper_leg":
                return 3.0f;
            case "lowerarm":
            case "lower_leg":
                return 2.0f;
            default:
                return 1.0f;
        }
    }
}
```

### Configuring Joint Limits

Proper joint limits are essential for realistic movement:

```csharp
using UnityEngine;

public class JointLimitConfiguration : MonoBehaviour
{
    [System.Serializable]
    public class JointLimit
    {
        public Transform jointTransform;
        public float lowerLimit = -45f;
        public float upperLimit = 45f;
        public float swingLimit = 45f;
    }

    [SerializeField] private JointLimit[] jointLimits;

    void Start()
    {
        ConfigureJointLimits();
    }

    void ConfigureJointLimits()
    {
        foreach (var limit in jointLimits)
        {
            if (limit.jointTransform != null)
            {
                ArticulationBody body = limit.jointTransform.GetComponent<ArticulationBody>();
                if (body != null)
                {
                    // Configure joint limits based on joint type
                    ArticulationReducedSpaceJointPose limitConfig = new ArticulationReducedSpaceJointPose();
                    limitConfig.x = new ArticulationDriveLimit() { lower = limit.lowerLimit, upper = limit.upperLimit };
                    limitConfig.y = new ArticulationDriveLimit() { lower = -limit.swingLimit, upper = limit.swingLimit };
                    limitConfig.z = new ArticulationDriveLimit() { lower = -limit.swingLimit, upper = limit.swingLimit };

                    body.jointMotion = ArticulationJointMotion.Locked; // This is a simplified example
                    // In practice, you'd configure based on specific joint type
                }
            }
        }
    }
}
```

## Animation and Control Systems

### Setting Up the Animation Controller

Create an animation controller for your humanoid:

1. Right-click in Project window → Create → Animator Controller
2. Name it "HumanoidAnimatorController"
3. In the Animator window, create states for different animations:
   - Idle
   - Walk
   - Gesture (wave, nod, etc.)
   - Interaction states

### Implementing Animation Control Script

```csharp
using UnityEngine;

public class HumanoidAnimationController : MonoBehaviour
{
    [SerializeField] private Animator animator;
    [SerializeField] private string speedParameter = "Speed";
    [SerializeField] private string gestureParameter = "Gesture";

    private static readonly int SpeedHash = Animator.StringToHash("Speed");
    private static readonly int GestureHash = Animator.StringToHash("Gesture");

    void Start()
    {
        if (animator == null)
            animator = GetComponent<Animator>();
    }

    public void SetMovementSpeed(float speed)
    {
        if (animator != null)
            animator.SetFloat(SpeedHash, speed);
    }

    public void TriggerGesture(string gestureName)
    {
        if (animator != null)
        {
            animator.SetTrigger(GestureHash);
            // In a more complex system, you might have different trigger parameters for different gestures
        }
    }

    public void SetGesture(string gestureName)
    {
        if (animator != null)
        {
            animator.SetInteger("Gesture", GetGestureIndex(gestureName));
        }
    }

    int GetGestureIndex(string gestureName)
    {
        switch (gestureName.ToLower())
        {
            case "wave": return 1;
            case "nod": return 2;
            case "shake": return 3;
            case "point": return 4;
            default: return 0; // idle
        }
    }
}
```

## ROS Integration for Real-time Control

### Setting Up ROS Communication

To integrate with ROS for real-time control:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

public class HumanoidROSController : MonoBehaviour
{
    [SerializeField] private string jointStatesTopic = "/robot/joint_states";
    [SerializeField] private string cmdVelTopic = "/robot/cmd_vel";

    private ROSConnection ros;
    private HumanoidJointMapper jointMapper;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(cmdVelTopic);
        ros.Subscribe<JointStateMsg>(jointStatesTopic, OnJointStateReceived);

        jointMapper = GetComponent<HumanoidJointMapper>();
    }

    void OnJointStateReceived(JointStateMsg jointState)
    {
        if (jointMapper != null)
        {
            jointMapper.UpdateJoints(jointState.name, jointState.position);
        }
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        TwistMsg cmd = new TwistMsg();
        cmd.linear = new Vector3Msg(linearX, 0, 0);
        cmd.angular = new Vector3Msg(0, 0, angularZ);

        ros.Publish(cmdVelTopic, cmd);
    }
}
```

### Joint Mapping System

Create a system to map ROS joint states to Unity transforms:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class HumanoidJointMapper : MonoBehaviour
{
    [System.Serializable]
    public class JointMapping
    {
        public string rosJointName;
        public Transform unityJoint;
        public Vector3 rotationAxis = Vector3.up;
        public float multiplier = 1.0f;
        public float offset = 0.0f;
    }

    [SerializeField] private JointMapping[] jointMappings;

    public void UpdateJoints(string[] jointNames, double[] jointPositions)
    {
        for (int i = 0; i < jointNames.Length; i++)
        {
            string rosName = jointNames[i];
            double position = jointPositions[i];

            foreach (var mapping in jointMappings)
            {
                if (mapping.rosJointName == rosName && mapping.unityJoint != null)
                {
                    float angle = (float)position * mapping.multiplier + mapping.offset;

                    // Apply rotation based on axis
                    mapping.unityJoint.localRotation = Quaternion.AngleAxis(angle, mapping.rotationAxis);
                    break;
                }
            }
        }
    }

    public void SetJointPosition(string jointName, float angle)
    {
        foreach (var mapping in jointMappings)
        {
            if (mapping.rosJointName == jointName && mapping.unityJoint != null)
            {
                mapping.unityJoint.localRotation = Quaternion.AngleAxis(angle, mapping.rotationAxis);
                break;
            }
        }
    }
}
```

## Creating Interactive Elements

### Interaction Points

Define points where users can interact with the robot:

```csharp
using UnityEngine;

public class InteractionPoint : MonoBehaviour
{
    [SerializeField] private string interactionType = "button";
    [SerializeField] private Color defaultColor = Color.gray;
    [SerializeField] private Color hoverColor = Color.yellow;
    [SerializeField] private Color activeColor = Color.green;

    private Material originalMaterial;
    private Renderer objectRenderer;

    void Start()
    {
        objectRenderer = GetComponent<Renderer>();
        if (objectRenderer != null)
        {
            originalMaterial = objectRenderer.material;
            SetColor(defaultColor);
        }
    }

    public void OnHoverEnter()
    {
        SetColor(hoverColor);
    }

    public void OnHoverExit()
    {
        SetColor(defaultColor);
    }

    public void OnActivate()
    {
        SetColor(activeColor);
        // Trigger interaction behavior
        TriggerInteraction();
    }

    void SetColor(Color color)
    {
        if (objectRenderer != null)
        {
            objectRenderer.material.color = color;
        }
    }

    void TriggerInteraction()
    {
        // Implement specific interaction behavior based on interactionType
        switch (interactionType.ToLower())
        {
            case "button":
                Debug.Log("Button pressed on robot");
                break;
            case "switch":
                Debug.Log("Switch toggled on robot");
                break;
            case "knob":
                Debug.Log("Knob rotated on robot");
                break;
        }
    }
}
```

## Performance Optimization

### Level of Detail (LOD)

For complex humanoid models, implement LOD systems:

```csharp
using UnityEngine;

[RequireComponent(typeof(LODGroup))]
public class HumanoidLODController : MonoBehaviour
{
    [SerializeField] private LODGroup lodGroup;
    [SerializeField] private Transform viewer;
    [SerializeField] private float[] lodDistances = { 10f, 20f, 50f };

    void Start()
    {
        if (lodGroup == null)
            lodGroup = GetComponent<LODGroup>();

        if (viewer == null)
            viewer = Camera.main.transform;
    }

    void Update()
    {
        if (viewer != null && lodGroup != null)
        {
            float distance = Vector3.Distance(transform.position, viewer.position);

            // Determine appropriate LOD level based on distance
            int lodLevel = 0;
            for (int i = 0; i < lodDistances.Length; i++)
            {
                if (distance > lodDistances[i])
                    lodLevel = i + 1;
                else
                    break;
            }

            // Clamp to valid range
            lodLevel = Mathf.Clamp(lodLevel, 0, lodGroup.lodCount - 1);

            lodGroup.ForceLOD(lodLevel);
        }
    }
}
```

## Exercise: Complete Humanoid Integration

1. Import a humanoid model (or use the basic one from Chapter 2) and configure it as a Unity humanoid
2. Add proper physics using Articulation Bodies
3. Create an animation controller with at least 3 different states
4. Implement a basic ROS communication system to update joint positions
5. Add at least 3 interactive elements to your humanoid model
6. Test the integration by controlling the model through a simple interface

## Troubleshooting Common Issues

### Issue: Model appears distorted after import
**Solution**: Check the model's scale in the modeling software and adjust Unity's scale factor accordingly. Ensure the model is properly centered at the origin.

### Issue: Joints don't map correctly in Avatar configuration
**Solution**: Ensure your model has proper bone hierarchy and naming conventions. Use standard humanoid bone names (hips, spine, chest, neck, head, etc.).

### Issue: Physics simulation is unstable
**Solution**: Check mass distribution, ensure proper joint configurations, and consider using lower fixed timestep values in Time settings.

## Next Steps

After completing humanoid model integration, proceed to [UI Controls Implementation](./ui-controls.md) to learn how to create interfaces for controlling your digital twin humanoid robot.