# LLM Cognitive Planning Setup Guide

This guide covers setting up the Large Language Model (LLM) integration for cognitive planning in the Vision-Language-Action (VLA) system.

## Overview

The cognitive planning component uses LLMs to translate natural language commands into sequences of robot actions. This approach enables complex, multi-step task planning that adapts to dynamic environments.

## Prerequisites

Before setting up LLM cognitive planning, ensure you have:

1. An OpenAI API key with GPT access
2. Python 3.8 or higher
3. The VLA core modules installed

## Installation

### 1. Install Required Packages

```bash
pip install openai python-dotenv
```

### 2. Configure Environment Variables

Add LLM-specific variables to your `.env` file:

```env
OPENAI_API_KEY=your_openai_api_key_here
LLM_MODEL=gpt-3.5-turbo
# Alternative: LLM_MODEL=gpt-4
```

## Integration with VLA System

### Cognitive Planner Module

The VLA system includes an `IntegratedCognitivePlanner` class that combines LLM and rule-based planning:

```python
from vla.cognitive_planner import IntegratedCognitivePlanner

# Initialize the planner
planner = IntegratedCognitivePlanner()

# Plan a complex task
environment_context = {
    "objects": ["red cup", "book"],
    "locations": ["kitchen", "living room"]
}

action_sequence = planner.plan_task(
    "Go to the kitchen and bring me the red cup",
    environment_context=environment_context,
    use_llm=True
)

print(f"Planned {len(action_sequence)} actions")
```

## Configuration

### LLM Model Selection

You can configure which LLM model to use based on your requirements:

```python
# For faster, less expensive responses
LLM_MODEL = "gpt-3.5-turbo"

# For more sophisticated planning
# LLM_MODEL = "gpt-4"
```

### Planning Parameters

Configure planning behavior:

```python
# Temperature (0.0-1.0): Lower = more deterministic, Higher = more creative
PLANNING_TEMPERATURE = 0.1

# Maximum tokens for planning responses
MAX_PLANNING_TOKENS = 500
```

## Planning Strategies

### 1. LLM-Based Planning

Uses GPT for sophisticated, context-aware planning:

```python
# Best for complex, multi-step commands
plan = llm_planner.plan_complex_task("Navigate to the office, find the report on the desk, and bring it to me")
```

### 2. Rule-Based Planning

Uses predefined rules for consistent, predictable planning:

```python
# Best for simple, well-defined commands
plan = rule_planner.plan_task("Move forward 1 meter")
```

### 3. Integrated Planning

Combines both approaches for optimal results:

```python
# Uses LLM first, falls back to rules if needed
plan = integrated_planner.plan_task(command_text, use_llm=True)
```

## Best Practices

### 1. Environment Context

Provide rich environment context for better planning:

```python
context = {
    "objects": ["red cup", "blue book", "wooden table"],
    "locations": ["kitchen", "living room", "office", "bedroom"],
    "robot_state": {"x": 0.0, "y": 0.0, "battery": 0.85},
    "safety_zones": ["staircase", "construction_area"]
}
```

### 2. Plan Validation

Always validate plans before execution:

```python
is_valid = planner.validate_plan(action_sequence, environment_context)
if is_valid:
    execute_plan(action_sequence)
else:
    print("Plan validation failed")
```

### 3. Error Handling

Implement robust error handling for planning failures:

```python
try:
    plan = planner.plan_task(command_text)
    if plan:
        execute_plan(plan)
    else:
        print("Planning failed, using fallback")
except Exception as e:
    print(f"Planning error: {e}")
    use_fallback_strategy()
```

## Troubleshooting

### Common Issues

1. **API Limitations**: LLM rate limits can affect planning speed
2. **Context Length**: Very long environment descriptions may exceed token limits
3. **Cost Management**: Monitor API usage for budget control

### Performance Optimization

- Use rule-based planning for simple, predictable tasks
- Cache common plans to reduce API calls
- Implement plan validation to avoid invalid sequences

## Next Steps

After setting up LLM cognitive planning, explore the complete VLA integration in the capstone section to see how all components work together.