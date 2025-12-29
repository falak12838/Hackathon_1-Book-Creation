# HRI Fundamentals

## Overview

Human-Robot Interaction (HRI) is a critical aspect of robotics that focuses on how humans and robots communicate and collaborate. This section covers the fundamental principles of HRI and how to implement them in Unity for digital twin environments.

## What is Human-Robot Interaction?

Human-Robot Interaction (HRI) is a multidisciplinary field that focuses on the design, development, and evaluation of robots for human use. It encompasses the study of how humans and robots communicate, collaborate, and interact in shared environments.

### Key HRI Principles

1. **Transparency**: The robot's intentions and state should be clearly communicated to the human user
2. **Predictability**: The robot should behave in ways that users can anticipate
3. **Trust**: Users should be able to rely on the robot's behavior
4. **Safety**: All interactions should prioritize human safety
5. **Usability**: The interaction should be intuitive and accessible

## Types of HRI

### 1. Physical Interaction
Direct physical contact between humans and robots, such as:
- Handshaking
- Physical assistance
- Collaborative manipulation

### 2. Social Interaction
Social behaviors and communication patterns, such as:
- Greeting behaviors
- Expressive gestures
- Verbal communication

### 3. Task-Based Interaction
Collaboration on specific tasks, such as:
- Assembly operations
- Navigation assistance
- Monitoring and alerting

## HRI in Digital Twin Environments

In digital twin environments, HRI is simulated to test and validate interaction scenarios before deployment in the real world. Unity provides excellent tools for prototyping and testing HRI concepts.

### Benefits of Digital Twin HRI
- **Safety**: Test interactions without physical risk
- **Cost-Effectiveness**: Simulate scenarios without building physical robots
- **Iterative Design**: Quickly test and refine interaction concepts
- **User Training**: Prepare users for real-world interactions

## Implementing HRI in Unity

### Visual Feedback Systems

Visual feedback is crucial for effective HRI. Implement these visual elements:

#### 1. Status Indicators
```csharp
using UnityEngine;
using UnityEngine.UI;

public class RobotStatusIndicator : MonoBehaviour
{
    [SerializeField] private Image statusLight;
    [SerializeField] private Color idleColor = Color.blue;
    [SerializeField] private Color activeColor = Color.green;
    [SerializeField] private Color warningColor = Color.yellow;
    [SerializeField] private Color errorColor = Color.red;

    private enum RobotState { Idle, Active, Warning, Error }
    private RobotState currentState = RobotState.Idle;

    void Update()
    {
        switch (currentState)
        {
            case RobotState.Idle:
                statusLight.color = idleColor;
                break;
            case RobotState.Active:
                statusLight.color = activeColor;
                break;
            case RobotState.Warning:
                statusLight.color = warningColor;
                // Add pulsing animation
                statusLight.color = Color.Lerp(warningColor, Color.white, Mathf.PingPong(Time.time, 1f));
                break;
            case RobotState.Error:
                statusLight.color = errorColor;
                // Add rapid flashing
                statusLight.color = (int)(Time.time * 10) % 2 == 0 ? errorColor : Color.black;
                break;
        }
    }

    public void SetState(RobotState newState)
    {
        currentState = newState;
    }
}
```

#### 2. Attention Indicators
Use gaze direction or highlighting to show where the robot is focusing:

```csharp
using UnityEngine;

public class AttentionIndicator : MonoBehaviour
{
    [SerializeField] private GameObject attentionGlow;
    [SerializeField] private Transform targetToFocusOn;
    [SerializeField] private float followSpeed = 2.0f;

    void Update()
    {
        if (targetToFocusOn != null)
        {
            // Make the attention indicator follow the target
            Vector3 targetPosition = targetToFocusOn.position;
            targetPosition.y = transform.position.y; // Keep at robot height
            attentionGlow.transform.position = Vector3.Lerp(attentionGlow.transform.position, targetPosition, Time.deltaTime * followSpeed);
        }
    }

    public void FocusOn(Transform target)
    {
        targetToFocusOn = target;
        attentionGlow.SetActive(true);
    }

    public void ClearFocus()
    {
        targetToFocusOn = null;
        attentionGlow.SetActive(false);
    }
}
```

### Audio Feedback Systems

Audio cues are essential for effective HRI:

```csharp
using UnityEngine;

public class AudioFeedbackSystem : MonoBehaviour
{
    [SerializeField] private AudioSource audioSource;
    [SerializeField] private AudioClip startupSound;
    [SerializeField] private AudioClip attentionSound;
    [SerializeField] private AudioClip errorSound;
    [SerializeField] private AudioClip confirmationSound;

    public void PlayStartupSound()
    {
        if (audioSource != null && startupSound != null)
            audioSource.PlayOneShot(startupSound);
    }

    public void PlayAttentionSound()
    {
        if (audioSource != null && attentionSound != null)
            audioSource.PlayOneShot(attentionSound);
    }

    public void PlayErrorSound()
    {
        if (audioSource != null && errorSound != null)
            audioSource.PlayOneShot(errorSound);
    }

    public void PlayConfirmationSound()
    {
        if (audioSource != null && confirmationSound != null)
            audioSource.PlayOneShot(confirmationSound);
    }
}
```

### Gesture and Movement Systems

Implement expressive movements to enhance interaction:

```csharp
using UnityEngine;

public class RobotGestureSystem : MonoBehaviour
{
    [SerializeField] private Transform head;
    [SerializeField] private Transform arm;
    [SerializeField] private float gestureSpeed = 2.0f;

    public void Wave()
    {
        // Simple wave animation
        StartCoroutine(WaveAnimation());
    }

    public void Nod()
    {
        // Simple nod animation
        StartCoroutine(NodAnimation());
    }

    public void ShakeHead()
    {
        // Simple head shake animation
        StartCoroutine(ShakeAnimation());
    }

    private System.Collections.IEnumerator WaveAnimation()
    {
        Vector3 initialRotation = arm.localEulerAngles;
        float duration = 1.0f;
        float progress = 0;

        while (progress < 1.0f)
        {
            progress += Time.deltaTime / duration;
            // Wave motion
            float waveRotation = Mathf.Sin(progress * Mathf.PI * 2) * 30f;
            arm.localRotation = Quaternion.Euler(initialRotation.x, initialRotation.y, initialRotation.z + waveRotation);
            yield return null;
        }

        arm.localRotation = Quaternion.Euler(initialRotation);
    }

    private System.Collections.IEnumerator NodAnimation()
    {
        Vector3 initialRotation = head.localEulerAngles;
        float duration = 0.5f;
        float progress = 0;

        while (progress < 1.0f)
        {
            progress += Time.deltaTime / duration;
            // Nod motion
            float nodRotation = Mathf.Sin(progress * Mathf.PI) * 15f;
            head.localRotation = Quaternion.Euler(initialRotation.x + nodRotation, initialRotation.y, initialRotation.z);
            yield return null;
        }

        head.localRotation = Quaternion.Euler(initialRotation);
    }

    private System.Collections.IEnumerator ShakeAnimation()
    {
        Vector3 initialRotation = head.localEulerAngles;
        float duration = 0.5f;
        float progress = 0;

        while (progress < 1.0f)
        {
            progress += Time.deltaTime / duration;
            // Shake motion
            float shakeRotation = Mathf.Sin(progress * Mathf.PI * 4) * 15f;
            head.localRotation = Quaternion.Euler(initialRotation.x, initialRotation.y + shakeRotation, initialRotation.z);
            yield return null;
        }

        head.localRotation = Quaternion.Euler(initialRotation);
    }
}
```

## Interaction Patterns

### 1. Proxemic Interaction
Respect personal space and comfort zones:
- **Intimate distance**: 0-0.5m (for close collaboration)
- **Personal distance**: 0.5-1.2m (for direct interaction)
- **Social distance**: 1.2-3.6m (for general communication)
- **Public distance**: 3.6m+ (for presentations)

### 2. Turn-Taking Protocols
Implement clear turn-taking mechanisms:
- Visual indicators for speaking/listening states
- Clear start and end of interaction signals
- Acknowledgment responses

### 3. Error Handling
Design for graceful error handling:
- Clear error messages
- Recovery suggestions
- Fallback behaviors

## HRI Design Guidelines

### 1. Visibility of System Status
- Provide appropriate feedback within reasonable time
- Show robot state clearly
- Indicate ongoing processes

### 2. Match Between System and Real World
- Use familiar concepts and icons
- Follow real-world conventions
- Make information visible and accessible

### 3. User Control and Freedom
- Provide easily accessible emergency exits
- Allow users to undo actions
- Offer multiple ways to accomplish tasks

### 4. Consistency and Standards
- Maintain consistency in design and behavior
- Follow platform conventions
- Use consistent terminology

## Exercise: Implementing Basic HRI

1. Create a simple interaction where the robot responds to the user approaching
2. Implement a greeting behavior (wave, nod, or audio greeting)
3. Add a status indicator that changes based on robot state
4. Create a simple gesture system that responds to user input
5. Design a basic turn-taking mechanism for interaction

## Best Practices

1. **Start Simple**: Begin with basic interactions and gradually add complexity
2. **Test Early and Often**: Get feedback from potential users throughout development
3. **Consider Cultural Differences**: HRI norms vary across cultures
4. **Prioritize Safety**: Ensure all interactions are safe for users
5. **Make It Predictable**: Users should be able to anticipate robot responses

## Next Steps

After understanding HRI fundamentals, proceed to [Humanoid Model Integration](./humanoid-integration.md) to learn how to integrate your digital twin with more sophisticated humanoid models and control systems.