# UI Controls Implementation

## Overview

This guide covers the implementation of user interface controls for interacting with digital twin humanoid robots in Unity. You'll learn to create intuitive interfaces that allow users to control robot movements, monitor status, and interact with the robot in meaningful ways.

## Prerequisites

- Completion of Humanoid Model Integration
- Understanding of Unity UI system
- Basic knowledge of C# scripting in Unity
- Familiarity with HRI concepts

## UI Design Principles for HRI

### 1. Intuitive Controls
- Use familiar control metaphors
- Provide clear visual feedback
- Maintain consistent interaction patterns

### 2. Information Hierarchy
- Display critical information prominently
- Group related controls together
- Use visual hierarchy to guide attention

### 3. Accessibility
- Ensure controls are usable by users with different abilities
- Provide multiple interaction modalities
- Consider colorblind-friendly palettes

## Setting Up the UI Canvas

### Step 1: Creating the Main Canvas

1. Right-click in the Hierarchy window → UI → Canvas
2. Name it "RobotControlCanvas"
3. Set the Render Mode to "Screen Space - Overlay" for most control panels
4. Add a Canvas Scaler component to ensure proper scaling across different screen sizes

### Step 2: Organizing UI Elements

Create a hierarchical structure for your UI:

```
RobotControlCanvas
├── RobotStatusPanel
│   ├── BatteryIndicator
│   ├── ConnectionStatus
│   └── CurrentTaskDisplay
├── ControlPanel
│   ├── MovementControls
│   ├── GestureControls
│   └── InteractionControls
└── InfoPanel
    ├── RobotName
    ├── SystemInfo
    └── HelpText
```

## Creating Control Elements

### 1. Robot Status Panel

Create a script to manage robot status information:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class RobotStatusPanel : MonoBehaviour
{
    [Header("Status Elements")]
    [SerializeField] private Image batteryFill;
    [SerializeField] private TextMeshProUGUI batteryText;
    [SerializeField] private Image connectionIndicator;
    [SerializeField] private TextMeshProUGUI taskText;
    [SerializeField] private TextMeshProUGUI modeText;

    [Header("Status Colors")]
    [SerializeField] private Color connectedColor = Color.green;
    [SerializeField] private Color disconnectedColor = Color.red;
    [SerializeField] private Color warningColor = Color.yellow;

    private float currentBattery = 100f;
    private bool isConnected = false;
    private string currentTask = "Idle";

    void Update()
    {
        UpdateBatteryDisplay();
        UpdateConnectionIndicator();
        UpdateTaskDisplay();
    }

    void UpdateBatteryDisplay()
    {
        // Animate battery fill
        batteryFill.fillAmount = Mathf.Lerp(batteryFill.fillAmount, currentBattery / 100f, Time.deltaTime * 2f);

        // Change color based on battery level
        if (currentBattery < 20f)
            batteryFill.color = Color.red;
        else if (currentBattery < 50f)
            batteryFill.color = warningColor;
        else
            batteryFill.color = Color.green;

        batteryText.text = Mathf.RoundToInt(currentBattery) + "%";
    }

    void UpdateConnectionIndicator()
    {
        connectionIndicator.color = isConnected ? connectedColor : disconnectedColor;
    }

    void UpdateTaskDisplay()
    {
        taskText.text = currentTask;
    }

    // Public methods to update status from other scripts
    public void SetBatteryLevel(float batteryLevel)
    {
        currentBattery = Mathf.Clamp(batteryLevel, 0f, 100f);
    }

    public void SetConnectionStatus(bool connected)
    {
        isConnected = connected;
    }

    public void SetCurrentTask(string task)
    {
        currentTask = task;
    }

    public void SetMode(string mode)
    {
        modeText.text = mode;
    }
}
```

### 2. Movement Controls

Create a movement control system:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class MovementControls : MonoBehaviour
{
    [Header("Movement Components")]
    [SerializeField] private Slider linearSpeedSlider;
    [SerializeField] private Slider angularSpeedSlider;
    [SerializeField] private Button moveForwardButton;
    [SerializeField] private Button moveBackwardButton;
    [SerializeField] private Button turnLeftButton;
    [SerializeField] private Button turnRightButton;
    [SerializeField] private Button stopButton;
    [SerializeField] private TextMeshProUGUI linearSpeedText;
    [SerializeField] private TextMeshProUGUI angularSpeedText;

    [Header("Movement Parameters")]
    [SerializeField] private float maxLinearSpeed = 2.0f;
    [SerializeField] private float maxAngularSpeed = 1.0f;

    private float currentLinearSpeed = 0.5f;
    private float currentAngularSpeed = 0.3f;

    void Start()
    {
        SetupMovementControls();
    }

    void SetupMovementControls()
    {
        // Initialize sliders
        linearSpeedSlider.minValue = 0.1f;
        linearSpeedSlider.maxValue = maxLinearSpeed;
        linearSpeedSlider.value = currentLinearSpeed;
        linearSpeedText.text = currentLinearSpeed.ToString("F1") + " m/s";

        angularSpeedSlider.minValue = 0.1f;
        angularSpeedSlider.maxValue = maxAngularSpeed;
        angularSpeedSlider.value = currentAngularSpeed;
        angularSpeedText.text = currentAngularSpeed.ToString("F1") + " rad/s";

        // Add event listeners
        linearSpeedSlider.onValueChanged.AddListener(OnLinearSpeedChanged);
        angularSpeedSlider.onValueChanged.AddListener(OnAngularSpeedChanged);

        moveForwardButton.onClick.AddListener(() => SendMovementCommand(Vector3.forward, 0));
        moveBackwardButton.onClick.AddListener(() => SendMovementCommand(Vector3.back, 0));
        turnLeftButton.onClick.AddListener(() => SendMovementCommand(Vector3.zero, 1));
        turnRightButton.onClick.AddListener(() => SendMovementCommand(Vector3.zero, -1));
        stopButton.onClick.AddListener(() => SendMovementCommand(Vector3.zero, 0));
    }

    void OnLinearSpeedChanged(float value)
    {
        currentLinearSpeed = value;
        linearSpeedText.text = currentLinearSpeed.ToString("F1") + " m/s";
    }

    void OnAngularSpeedChanged(float value)
    {
        currentAngularSpeed = value;
        angularSpeedText.text = currentAngularSpeed.ToString("F1") + " rad/s";
    }

    void SendMovementCommand(Vector3 linearDirection, float angularDirection)
    {
        // This is a simplified example - in practice, you'd send this to your robot controller
        Debug.Log($"Sending movement command: Linear={linearDirection * currentLinearSpeed}, Angular={angularDirection * currentAngularSpeed}");

        // In a real implementation, you might call:
        // RobotController.Instance.Move(linearDirection * currentLinearSpeed, angularDirection * currentAngularSpeed);
    }
}
```

### 3. Gesture Controls

Create a gesture control system:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class GestureControls : MonoBehaviour
{
    [Header("Gesture Buttons")]
    [SerializeField] private Button waveButton;
    [SerializeField] private Button nodButton;
    [SerializeField] private Button shakeButton;
    [SerializeField] private Button pointButton;
    [SerializeField] private Button resetButton;

    [Header("Gesture Text Display")]
    [SerializeField] private TextMeshProUGUI gestureStatusText;

    [Header("Animation Controller")]
    [SerializeField] private HumanoidAnimationController animationController;

    void Start()
    {
        SetupGestureControls();
    }

    void SetupGestureControls()
    {
        waveButton.onClick.AddListener(() => ExecuteGesture("wave"));
        nodButton.onClick.AddListener(() => ExecuteGesture("nod"));
        shakeButton.onClick.AddListener(() => ExecuteGesture("shake"));
        pointButton.onClick.AddListener(() => ExecuteGesture("point"));
        resetButton.onClick.AddListener(() => ExecuteGesture("idle"));

        gestureStatusText.text = "Ready for gesture commands";
    }

    void ExecuteGesture(string gestureName)
    {
        gestureStatusText.text = "Executing: " + gestureName;

        if (animationController != null)
        {
            animationController.SetGesture(gestureName);
        }
        else
        {
            Debug.LogWarning("Animation controller not assigned");
        }

        // Optional: Reset status text after a delay
        Invoke("ClearGestureStatus", 2.0f);
    }

    void ClearGestureStatus()
    {
        gestureStatusText.text = "Ready for gesture commands";
    }
}
```

## Creating a Joystick Control

For more intuitive movement control, implement a virtual joystick:

```csharp
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class VirtualJoystick : MonoBehaviour, IDragHandler, IPointerUpHandler, IPointerDownHandler
{
    [Header("Joystick Components")]
    [SerializeField] private Image bgImage;
    [SerializeField] private Image joystickImage;

    [Header("Joystick Settings")]
    [SerializeField] private float joystickRange = 50f;
    [SerializeField] private bool isFixed = false;

    private Vector3 joystickCenter;
    private bool isPressed = false;

    void Start()
    {
        joystickCenter = transform.position;
    }

    public void OnDrag(PointerEventData eventData)
    {
        if (!isPressed) return;

        Vector3 position = RectTransformUtility.WorldToScreenPoint(Camera.main, transform.position);
        Vector3 direction = eventData.position - position;
        float distance = direction.magnitude;
        direction = direction.normalized;

        if (distance < joystickRange)
        {
            // Within range - move joystick normally
            transform.position = position + direction * distance;
        }
        else
        {
            // Outside range - keep at maximum distance
            transform.position = position + direction * joystickRange;
        }

        // Update robot movement based on joystick position
        UpdateRobotMovement();
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        if (!isFixed)
        {
            transform.position = eventData.position;
            joystickCenter = eventData.position;
        }
        isPressed = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        isPressed = false;
        if (!isFixed)
        {
            transform.position = joystickCenter;
        }
        else
        {
            transform.localPosition = Vector3.zero;
        }

        // Stop robot movement when joystick is released
        StopRobotMovement();
    }

    void UpdateRobotMovement()
    {
        Vector3 direction = (Vector3)(transform.position - joystickCenter).normalized;
        float magnitude = Mathf.Clamp01(Vector3.Distance(transform.position, joystickCenter) / joystickRange);

        // Send movement command based on joystick position
        // In practice, you'd send this to your robot controller
        Vector3 movement = new Vector3(direction.x, 0, direction.y) * magnitude;
        Debug.Log($"Joystick movement: {movement}");
    }

    void StopRobotMovement()
    {
        Debug.Log("Stopping robot movement");
    }
}
```

## Creating a Command Console

For advanced users, implement a command console:

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Collections.Generic;

public class CommandConsole : MonoBehaviour
{
    [Header("Console Components")]
    [SerializeField] private TMP_InputField commandInput;
    [SerializeField] private TextMeshProUGUI outputText;
    [SerializeField] private Button executeButton;
    [SerializeField] private Button clearButton;
    [SerializeField] private ScrollRect scrollRect;

    [Header("Console Settings")]
    [SerializeField] private int maxLines = 50;
    [SerializeField] private List<string> commandHistory = new List<string>();
    private int historyIndex = -1;

    void Start()
    {
        SetupCommandConsole();
    }

    void SetupCommandConsole()
    {
        executeButton.onClick.AddListener(ExecuteCommand);
        clearButton.onClick.AddListener(ClearConsole);
        commandInput.onSubmit.AddListener(ExecuteCommand);

        outputText.text = "Robot Control Console Ready\n";
    }

    void ExecuteCommand(string command = "")
    {
        if (string.IsNullOrEmpty(command))
            command = commandInput.text;

        if (string.IsNullOrEmpty(command))
            return;

        // Add to history
        commandHistory.Add(command);
        historyIndex = commandHistory.Count - 1;

        // Display command
        AddToOutput("> " + command);

        // Process command
        ProcessCommand(command);

        // Clear input
        commandInput.text = "";
    }

    void ProcessCommand(string command)
    {
        // Parse and execute the command
        string[] parts = command.Split(' ');
        string cmd = parts[0].ToLower();

        switch (cmd)
        {
            case "move":
                ProcessMoveCommand(parts);
                break;
            case "gesture":
                ProcessGestureCommand(parts);
                break;
            case "status":
                ProcessStatusCommand(parts);
                break;
            case "help":
                ShowHelp();
                break;
            default:
                AddToOutput("Unknown command: " + cmd + ". Type 'help' for available commands.");
                break;
        }
    }

    void ProcessMoveCommand(string[] parts)
    {
        if (parts.Length < 3)
        {
            AddToOutput("Usage: move <x> <z> (e.g., move 1.0 0.0)");
            return;
        }

        if (float.TryParse(parts[1], out float x) && float.TryParse(parts[2], out float z))
        {
            Vector3 direction = new Vector3(x, 0, z);
            AddToOutput($"Moving in direction: {direction}");
            // In practice, send this to robot controller
        }
        else
        {
            AddToOutput("Invalid coordinates. Use numeric values.");
        }
    }

    void ProcessGestureCommand(string[] parts)
    {
        if (parts.Length < 2)
        {
            AddToOutput("Usage: gesture <name> (e.g., gesture wave)");
            return;
        }

        string gestureName = parts[1].ToLower();
        AddToOutput($"Executing gesture: {gestureName}");
        // In practice, trigger the gesture
    }

    void ProcessStatusCommand(string[] parts)
    {
        AddToOutput("Robot Status: Operational");
        AddToOutput("Battery: 85%");
        AddToOutput("Connection: Stable");
        AddToOutput("Current Task: Idle");
    }

    void ShowHelp()
    {
        AddToOutput("Available Commands:");
        AddToOutput("  move <x> <z>     - Move robot in direction (x, z)");
        AddToOutput("  gesture <name>   - Execute named gesture (wave, nod, shake)");
        AddToOutput("  status          - Show robot status");
        AddToOutput("  help            - Show this help message");
    }

    void AddToOutput(string message)
    {
        outputText.text += message + "\n";

        // Limit lines
        string[] lines = outputText.text.Split('\n');
        if (lines.Length > maxLines)
        {
            string newText = "";
            for (int i = lines.Length - maxLines; i < lines.Length; i++)
            {
                if (i > lines.Length - maxLines) newText += "\n";
                newText += lines[i];
            }
            outputText.text = newText;
        }

        // Scroll to bottom
        Canvas.ForceUpdateCanvases();
        scrollRect.verticalNormalizedPosition = 0f;
    }

    void ClearConsole()
    {
        outputText.text = "";
        AddToOutput("Console cleared");
    }
}
```

## Responsive UI Design

### Creating Responsive Layouts

Use Unity's layout groups to create responsive UI:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class ResponsiveUILayout : MonoBehaviour
{
    [Header("Layout Elements")]
    [SerializeField] private HorizontalLayoutGroup controlRow;
    [SerializeField] private VerticalLayoutGroup mainPanel;
    [SerializeField] private ContentSizeFitter contentFitter;

    [Header("Responsive Settings")]
    [SerializeField] private float minWidth = 800f;
    [SerializeField] private float maxWidth = 1920f;
    [SerializeField] private float minControlHeight = 100f;

    void Start()
    {
        SetupResponsiveLayout();
    }

    void SetupResponsiveLayout()
    {
        // Configure layout groups with appropriate settings
        if (controlRow != null)
        {
            controlRow.childForceExpandWidth = true;
            controlRow.childForceExpandHeight = true;
        }

        if (mainPanel != null)
        {
            mainPanel.childForceExpandWidth = true;
            mainPanel.childForceExpandHeight = false;
        }

        // Add a listener for screen resolution changes
        Screen.sleepTimeout = SleepTimeout.NeverSleep;
    }

    void Update()
    {
        AdjustLayoutForScreenSize();
    }

    void AdjustLayoutForScreenSize()
    {
        float screenWidth = Screen.width;
        float ratio = screenWidth / 1920f; // Based on 1080p reference

        // Adjust UI scale based on screen size
        CanvasScaler scaler = GetComponentInParent<Canvas>().GetComponent<CanvasScaler>();
        if (scaler != null)
        {
            scaler.scaleFactor = Mathf.Clamp(ratio, 0.5f, 2f);
        }
    }
}
```

## Exercise: Complete UI Implementation

1. Create a complete robot control interface with all the components described
2. Implement at least 3 different control methods (buttons, joystick, command console)
3. Add visual feedback for all user interactions
4. Create a responsive layout that works on different screen sizes
5. Test the interface with your humanoid robot model
6. Add accessibility features (keyboard navigation, colorblind-friendly colors)

## Best Practices for HRI UI

1. **Provide Immediate Feedback**: Always respond to user input immediately
2. **Use Consistent Patterns**: Maintain consistent interaction patterns throughout the interface
3. **Prioritize Safety**: Make emergency stop controls easily accessible
4. **Consider Context**: Show relevant controls based on robot state
5. **Test with Users**: Validate your interface design with actual users

## Troubleshooting Common Issues

### Issue: UI elements not responding to input
**Solution**: Check that the Canvas has Graphic Raycaster component and that the EventSystem is present in the scene.

### Issue: Controls feel unresponsive
**Solution**: Add visual feedback for button presses and ensure appropriate sizing for touch interfaces.

### Issue: UI overlaps or doesn't scale properly
**Solution**: Use Layout Groups and Content Size Fitters properly, and test on different screen resolutions.

## Next Steps

After implementing UI controls, proceed to [HRI Interaction Examples](./hri-examples.md) to learn how to create complete interaction scenarios between humans and your digital twin humanoid robot.