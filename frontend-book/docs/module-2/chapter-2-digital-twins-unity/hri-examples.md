# HRI Interaction Examples

## Overview

This guide provides practical examples of Human-Robot Interaction (HRI) scenarios that can be implemented in Unity for digital twin applications. These examples demonstrate how to create meaningful interactions between humans and digital twin humanoid robots.

## Prerequisites

- Completion of UI Controls Implementation
- Understanding of HRI fundamentals
- Working humanoid robot model with physics and animation
- Basic knowledge of Unity's interaction systems

## Example 1: Guided Tour Interaction

### Scenario Overview
A humanoid robot guides a human user through a virtual environment, providing information and responding to user queries.

### Implementation

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;
using System.Collections.Generic;

public class GuidedTourInteraction : MonoBehaviour
{
    [Header("Tour Components")]
    [SerializeField] private Transform robot;
    [SerializeField] private Transform user;
    [SerializeField] private TextMeshProUGUI infoDisplay;
    [SerializeField] private Button nextPointButton;
    [SerializeField] private Button askQuestionButton;
    [SerializeField] private Button stopTourButton;

    [Header("Tour Settings")]
    [SerializeField] private List<Transform> tourPoints = new List<Transform>();
    [SerializeField] private float movementSpeed = 2.0f;
    [SerializeField] private float interactionDistance = 3.0f;
    [SerializeField] private string[] tourInformation = new string[0];

    private int currentPointIndex = 0;
    private bool isTourActive = false;
    private bool isMovingToNextPoint = false;

    void Start()
    {
        SetupTourInteraction();
    }

    void SetupTourInteraction()
    {
        nextPointButton.onClick.AddListener(MoveToNextPoint);
        askQuestionButton.onClick.AddListener(AnswerQuestion);
        stopTourButton.onClick.AddListener(StopTour);

        infoDisplay.text = "Press 'Start Tour' to begin your guided experience.";
    }

    public void StartTour()
    {
        if (tourPoints.Count == 0) return;

        isTourActive = true;
        currentPointIndex = 0;
        MoveToNextPoint();
    }

    void MoveToNextPoint()
    {
        if (!isTourActive || currentPointIndex >= tourPoints.Count) return;

        if (isMovingToNextPoint) return; // Prevent multiple movements

        StartCoroutine(MoveToTarget(tourPoints[currentPointIndex]));
        DisplayTourInformation(currentPointIndex);
    }

    IEnumerator MoveToTarget(Transform target)
    {
        isMovingToNextPoint = true;

        Vector3 targetPosition = target.position;
        targetPosition.y = robot.position.y; // Keep at robot height

        while (Vector3.Distance(robot.position, targetPosition) > 0.5f)
        {
            robot.position = Vector3.MoveTowards(robot.position, targetPosition, movementSpeed * Time.deltaTime);

            // Make robot face the target direction
            Vector3 direction = targetPosition - robot.position;
            if (direction.magnitude > 0.1f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction, Vector3.up);
                robot.rotation = Quaternion.Slerp(robot.rotation, targetRotation, Time.deltaTime * 5f);
            }

            yield return null;
        }

        isMovingToNextPoint = false;
        currentPointIndex++;

        if (currentPointIndex >= tourPoints.Count)
        {
            infoDisplay.text = "Tour completed! Thank you for your visit.";
            isTourActive = false;
        }
    }

    void DisplayTourInformation(int index)
    {
        if (index < tourInformation.Length)
        {
            infoDisplay.text = tourInformation[index];
        }
    }

    void AnswerQuestion()
    {
        // Simple question answering system
        string[] possibleAnswers = {
            "I'm here to guide you through this facility.",
            "This area contains our advanced robotics lab.",
            "The equipment here is used for digital twin simulations.",
            "Feel free to ask me any questions about our work here."
        };

        string answer = possibleAnswers[Random.Range(0, possibleAnswers.Length)];
        infoDisplay.text = "Answer: " + answer;

        // Add animation for emphasis
        StartCoroutine(HeadNodAnimation());
    }

    IEnumerator HeadNodAnimation()
    {
        Transform head = FindRobotPart("Head");
        if (head != null)
        {
            Vector3 originalRotation = head.localEulerAngles;
            float duration = 0.5f;
            float progress = 0;

            while (progress < 1.0f)
            {
                progress += Time.deltaTime / duration;
                float nodRotation = Mathf.Sin(progress * Mathf.PI) * 15f;
                head.localRotation = Quaternion.Euler(originalRotation.x + nodRotation, originalRotation.y, originalRotation.z);
                yield return null;
            }

            head.localRotation = Quaternion.Euler(originalRotation);
        }
    }

    Transform FindRobotPart(string partName)
    {
        Transform head = robot.Find(partName);
        if (head == null)
        {
            // Try finding with common variations
            head = robot.Find("Head") ?? robot.Find("head") ?? robot.Find("HEAD");
        }
        return head;
    }

    void StopTour()
    {
        isTourActive = false;
        isMovingToNextPoint = false;
        infoDisplay.text = "Tour stopped. Thank you for your visit.";
    }
}
```

## Example 2: Collaborative Assembly Task

### Scenario Overview
A humanoid robot collaborates with a human user to complete an assembly task, passing tools and components as needed.

### Implementation

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class CollaborativeAssembly : MonoBehaviour
{
    [Header("Assembly Components")]
    [SerializeField] private Transform robot;
    [SerializeField] private Transform user;
    [SerializeField] private List<Transform> assemblyPieces = new List<Transform>();
    [SerializeField] private List<Transform> toolStations = new List<Transform>();
    [SerializeField] private Transform assemblyArea;
    [SerializeField] private TextMeshProUGUI taskDisplay;
    [SerializeField] private Button passToolButton;
    [SerializeField] private Button requestToolButton;
    [SerializeField] private Button resetTaskButton;

    [Header("Assembly Settings")]
    [SerializeField] private float interactionDistance = 2.0f;
    [SerializeField] private float toolPassSpeed = 5.0f;

    private List<Transform> availableTools = new List<Transform>();
    private int completedPieces = 0;
    private bool isTaskActive = false;

    void Start()
    {
        SetupAssemblyInteraction();
    }

    void SetupAssemblyInteraction()
    {
        passToolButton.onClick.AddListener(PassToolToUser);
        requestToolButton.onClick.AddListener(RequestToolFromUser);
        resetTaskButton.onClick.AddListener(ResetTask);

        taskDisplay.text = "Assembly task ready. Select a tool to begin.";
    }

    void PassToolToUser()
    {
        if (availableTools.Count == 0)
        {
            taskDisplay.text = "No tools available to pass.";
            return;
        }

        Transform toolToPass = availableTools[0];
        StartCoroutine(PassObjectToUser(toolToPass));
    }

    void RequestToolFromUser()
    {
        // In a real implementation, this would detect if user is holding a tool
        // For this example, we'll just pretend the user has a tool
        taskDisplay.text = "Please hand me the tool when ready. I will take it when you're close enough.";

        // Check if user is close enough to the robot
        StartCoroutine(WaitForToolHandoff());
    }

    IEnumerator PassObjectToUser(Transform tool)
    {
        taskDisplay.text = "Passing tool to user...";

        Vector3 startPosition = tool.position;
        Vector3 targetPosition = user.position + new Vector3(0, 1, 0); // Position slightly above user

        float duration = Vector3.Distance(startPosition, targetPosition) / toolPassSpeed;
        float progress = 0;

        while (progress < 1.0f)
        {
            progress += Time.deltaTime / duration;
            tool.position = Vector3.Lerp(startPosition, targetPosition, progress);
            yield return null;
        }

        availableTools.Remove(tool);
        taskDisplay.text = "Tool passed to user. Ready for next step.";
    }

    IEnumerator WaitForToolHandoff()
    {
        float startTime = Time.time;
        float timeout = 10f; // 10 second timeout

        while (Time.time - startTime < timeout && isTaskActive)
        {
            float distance = Vector3.Distance(robot.position, user.position);

            if (distance <= interactionDistance)
            {
                // Simulate taking the tool
                taskDisplay.text = "Received tool from user. Thank you!";
                break;
            }

            yield return null;
        }

        if (Time.time - startTime >= timeout)
        {
            taskDisplay.text = "Timeout: Please come closer with the tool.";
        }
    }

    public void CompleteAssemblyPiece()
    {
        completedPieces++;
        taskDisplay.text = $"Assembly progress: {completedPieces}/{assemblyPieces.Count} pieces completed.";

        if (completedPieces >= assemblyPieces.Count)
        {
            taskDisplay.text = "Assembly task completed successfully! Great teamwork!";
            isTaskActive = false;
        }
    }

    void ResetTask()
    {
        completedPieces = 0;
        isTaskActive = true;
        availableTools.Clear();

        // Reset all assembly pieces to their original positions
        for (int i = 0; i < assemblyPieces.Count; i++)
        {
            // This would restore the original positions of assembly pieces
        }

        taskDisplay.text = "Assembly task reset. Ready to begin again.";
    }

    public void AddAvailableTool(Transform tool)
    {
        availableTools.Add(tool);
    }
}
```

## Example 3: Emergency Response Interaction

### Scenario Overview
A humanoid robot responds to emergency situations, providing guidance and assistance to human users.

### Implementation

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections;

public class EmergencyResponseInteraction : MonoBehaviour
{
    [Header("Emergency Components")]
    [SerializeField] private Transform robot;
    [SerializeField] private Transform user;
    [SerializeField] private TextMeshProUGUI emergencyDisplay;
    [SerializeField] private Button emergencyCallButton;
    [SerializeField] private Button evacuationButton;
    [SerializeField] private Button safetyCheckButton;
    [SerializeField] private AudioSource alertSound;
    [SerializeField] private Light emergencyLight;

    [Header("Emergency Settings")]
    [SerializeField] private float evacuationSpeed = 3.0f;
    [SerializeField] private Transform[] evacuationRoutes = new Transform[0];
    [SerializeField] private Transform[] safetyCheckPoints = new Transform[0];

    private bool isEmergencyActive = false;
    private bool isEvacuationActive = false;
    private bool isCheckingSafety = false;

    void Start()
    {
        SetupEmergencyInteraction();
    }

    void SetupEmergencyInteraction()
    {
        emergencyCallButton.onClick.AddListener(TriggerEmergency);
        evacuationButton.onClick.AddListener(StartEvacuation);
        safetyCheckButton.onClick.AddListener(PerformSafetyCheck);

        emergencyDisplay.text = "System status: Normal. Press emergency button if needed.";
    }

    void TriggerEmergency()
    {
        isEmergencyActive = true;
        emergencyDisplay.text = "EMERGENCY ACTIVATED! Please remain calm and follow my instructions.";

        // Activate emergency indicators
        if (alertSound != null) alertSound.Play();
        if (emergencyLight != null) emergencyLight.enabled = true;

        // Robot enters emergency mode
        EnterEmergencyMode();
    }

    void EnterEmergencyMode()
    {
        // Robot changes behavior for emergency
        // Change color, increase animation speed, etc.
        Renderer[] renderers = robot.GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            Material[] materials = renderer.materials;
            foreach (Material material in materials)
            {
                material.SetColor("_EmissionColor", Color.red);
            }
        }
    }

    void StartEvacuation()
    {
        if (!isEmergencyActive) return;

        isEvacuationActive = true;
        emergencyDisplay.text = "Evacuation started. Follow me to the nearest safe zone.";

        if (evacuationRoutes.Length > 0)
        {
            // Choose the closest evacuation route
            Transform closestRoute = FindClosestRoute();
            if (closestRoute != null)
            {
                StartCoroutine(GuideEvacuation(closestRoute));
            }
        }
    }

    Transform FindClosestRoute()
    {
        Transform closest = null;
        float minDistance = float.MaxValue;

        foreach (Transform route in evacuationRoutes)
        {
            float distance = Vector3.Distance(user.position, route.position);
            if (distance < minDistance)
            {
                minDistance = distance;
                closest = route;
            }
        }

        return closest;
    }

    IEnumerator GuideEvacuation(Transform destination)
    {
        Vector3 targetPosition = destination.position;
        targetPosition.y = robot.position.y; // Maintain robot height

        while (Vector3.Distance(robot.position, targetPosition) > 1.0f && isEvacuationActive)
        {
            robot.position = Vector3.MoveTowards(robot.position, targetPosition, evacuationSpeed * Time.deltaTime);

            // Face the direction of movement
            Vector3 direction = targetPosition - robot.position;
            if (direction.magnitude > 0.1f)
            {
                Quaternion targetRotation = Quaternion.LookRotation(direction, Vector3.up);
                robot.rotation = Quaternion.Slerp(robot.rotation, targetRotation, Time.deltaTime * 5f);
            }

            // Provide verbal guidance
            if (Random.Range(0, 100) < 5) // Occasionally provide updates
            {
                ProvideEvacuationGuidance();
            }

            yield return null;
        }

        if (isEvacuationActive)
        {
            emergencyDisplay.text = "Evacuation completed. You are now in a safe zone.";
            isEvacuationActive = false;
        }
    }

    void ProvideEvacuationGuidance()
    {
        string[] guidanceMessages = {
            "Stay close to me, we're almost there.",
            "Please keep moving forward, safety is near.",
            "We're taking the safest route out of the area.",
            "Almost there, just a little further."
        };

        string message = guidanceMessages[Random.Range(0, guidanceMessages.Length)];
        emergencyDisplay.text = message;
    }

    void PerformSafetyCheck()
    {
        if (isCheckingSafety) return;

        isCheckingSafety = true;
        emergencyDisplay.text = "Performing safety check...";

        StartCoroutine(SafetyCheckRoutine());
    }

    IEnumerator SafetyCheckRoutine()
    {
        // Simulate safety check process
        for (int i = 0; i < safetyCheckPoints.Length; i++)
        {
            Transform checkPoint = safetyCheckPoints[i];
            float checkTime = 2.0f; // Time to check each point
            float progress = 0;

            while (progress < 1.0f)
            {
                progress += Time.deltaTime / checkTime;
                float checkProgress = Mathf.Clamp01(progress);

                // Simulate checking behavior (e.g., looking around, scanning)
                Transform head = robot.Find("Head") ?? robot.Find("head");
                if (head != null)
                {
                    head.localRotation = Quaternion.Euler(0, Mathf.Sin(Time.time * 5f) * 30f, 0);
                }

                emergencyDisplay.text = $"Safety check: {Mathf.RoundToInt(checkProgress * 100)}% complete...";
                yield return null;
            }
        }

        isCheckingSafety = false;
        emergencyDisplay.text = "Safety check completed. Area is secure.";
    }

    public void DeactivateEmergency()
    {
        isEmergencyActive = false;
        isEvacuationActive = false;

        if (alertSound != null) alertSound.Stop();
        if (emergencyLight != null) emergencyLight.enabled = false;

        // Reset robot appearance
        Renderer[] renderers = robot.GetComponentsInChildren<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            Material[] materials = renderer.materials;
            foreach (Material material in materials)
            {
                material.SetColor("_EmissionColor", Color.black);
            }
        }

        emergencyDisplay.text = "Emergency deactivated. System status: Normal.";
    }
}
```

## Creating Interactive Environments

### Example: Interactive Control Panel

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class InteractiveControlPanel : MonoBehaviour
{
    [Header("Panel Components")]
    [SerializeField] private Button[] controlButtons = new Button[0];
    [SerializeField] private Slider[] controlSliders = new Slider[0];
    [SerializeField] private Toggle[] controlToggles = new Toggle[0];
    [SerializeField] private TextMeshProUGUI statusDisplay;

    [Header("Robot Connection")]
    [SerializeField] private HumanoidAnimationController robotController;

    private void Start()
    {
        SetupControlPanel();
    }

    void SetupControlPanel()
    {
        // Setup button listeners
        for (int i = 0; i < controlButtons.Length; i++)
        {
            int index = i; // Capture for closure
            controlButtons[i].onClick.AddListener(() => OnButtonPressed(index));
        }

        // Setup slider listeners
        for (int i = 0; i < controlSliders.Length; i++)
        {
            int index = i;
            controlSliders[i].onValueChanged.AddListener((value) => OnSliderChanged(index, value));
        }

        // Setup toggle listeners
        for (int i = 0; i < controlToggles.Length; i++)
        {
            int index = i;
            controlToggles[i].onValueChanged.AddListener((value) => OnToggleChanged(index, value));
        }

        statusDisplay.text = "Control panel ready.";
    }

    void OnButtonPressed(int buttonIndex)
    {
        string action = GetActionForButton(buttonIndex);
        statusDisplay.text = $"Button {buttonIndex + 1} pressed: {action}";

        // Execute the action
        ExecuteRobotAction(action);
    }

    void OnSliderChanged(int sliderIndex, float value)
    {
        statusDisplay.text = $"Slider {sliderIndex + 1} changed to: {value:F2}";

        // Send value to robot
        SendSliderValueToRobot(sliderIndex, value);
    }

    void OnToggleChanged(int toggleIndex, bool isOn)
    {
        string state = isOn ? "ON" : "OFF";
        statusDisplay.text = $"Toggle {toggleIndex + 1} is now: {state}";

        // Update robot based on toggle state
        UpdateRobotWithToggle(toggleIndex, isOn);
    }

    string GetActionForButton(int buttonIndex)
    {
        string[] actions = {
            "Wave", "Nod", "Shake", "Point", "Idle",
            "Move Forward", "Move Backward", "Turn Left", "Turn Right", "Stop"
        };

        if (buttonIndex < actions.Length)
            return actions[buttonIndex];

        return "Unknown Action";
    }

    void ExecuteRobotAction(string action)
    {
        if (robotController == null) return;

        switch (action.ToLower())
        {
            case "wave":
                robotController.SetGesture("wave");
                break;
            case "nod":
                robotController.SetGesture("nod");
                break;
            case "shake":
                robotController.SetGesture("shake");
                break;
            case "point":
                robotController.SetGesture("point");
                break;
            case "idle":
                robotController.SetGesture("idle");
                break;
        }
    }

    void SendSliderValueToRobot(int sliderIndex, float value)
    {
        // In a real implementation, this would send the value to the robot
        Debug.Log($"Sending slider {sliderIndex} value {value} to robot");
    }

    void UpdateRobotWithToggle(int toggleIndex, bool isOn)
    {
        // In a real implementation, this would update the robot state
        Debug.Log($"Updating robot with toggle {toggleIndex} = {isOn}");
    }
}
```

## Exercise: Implement an HRI Scenario

Choose one of the following scenarios to implement:

### Option 1: Teaching Assistant Robot
Create a scenario where the robot helps teach a concept to a human user. Implement:
- Knowledge presentation
- Question answering
- Progress tracking
- Adaptive behavior based on user responses

### Option 2: Receptionist Robot
Create a scenario where the robot serves as a receptionist. Implement:
- Visitor greeting and check-in
- Information provision
- Direction giving
- Appointment scheduling

### Option 3: Fitness Coach Robot
Create a scenario where the robot serves as a fitness coach. Implement:
- Exercise demonstration
- Form correction
- Progress tracking
- Motivational feedback

## Best Practices for HRI Implementation

1. **Context Awareness**: Design interactions that consider the current context and environment
2. **Natural Communication**: Use communication patterns that feel natural to humans
3. **Error Recovery**: Implement graceful error handling and recovery mechanisms
4. **Privacy Considerations**: Respect user privacy in interactions
5. **Cultural Sensitivity**: Consider cultural differences in interaction patterns

## Testing HRI Scenarios

### Testing Checklist
- [ ] Interactions are intuitive and easy to understand
- [ ] Robot provides appropriate feedback for all actions
- [ ] Emergency procedures are clearly defined
- [ ] Error handling is graceful
- [ ] Performance is adequate for real-time interaction
- [ ] Accessibility features are implemented

## Troubleshooting Common Issues

### Issue: Interactions feel mechanical or unnatural
**Solution**: Add more natural movements, varied responses, and context-aware behaviors.

### Issue: Robot doesn't respond appropriately to user input
**Solution**: Implement proper state management and ensure all interaction paths are handled.

### Issue: Performance degradation during complex interactions
**Solution**: Optimize animations, reduce unnecessary calculations, and use efficient data structures.

## Next Steps

After implementing HRI interaction examples, you have completed Chapter 2 of Module 2. You can now proceed to [Chapter 3: Sensor Simulation & Validation](../chapter-3-sensor-simulation/index.md) to learn about simulating sensors in your digital twin environment.