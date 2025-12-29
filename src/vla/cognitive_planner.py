"""
Cognitive Planner module for the Vision-Language-Action (VLA) system.
Translates high-level natural language commands into sequences of executable actions.
"""

import openai
import os
import json
from dotenv import load_dotenv
from typing import Dict, List, Any, Optional
import re


# Load environment variables
load_dotenv()


class LLMCognitivePlanner:
    """
    Uses Large Language Models to plan complex multi-step tasks.
    """

    def __init__(self):
        """
        Initialize the cognitive planner with LLM configuration.
        """
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not found in environment variables")

        openai.api_key = self.api_key

        # Define the system prompt for the planning task
        self.system_prompt = """
        You are a cognitive planning assistant for a humanoid robot.
        Your task is to break down high-level natural language commands
        into sequences of specific robot actions.

        Each action should be one of the following types:
        - navigation: Move to a specific location
        - manipulation: Grasp, place, or manipulate an object
        - perception: Identify and locate objects
        - communication: Speak to the user
        - control: Stop, pause, or resume operations

        Return the plan as a JSON array of action objects with the following structure:
        [
            {{
                "action_type": "navigation|manipulation|perception|communication|control",
                "action": "specific action name",
                "target_location|target_object|message": "specific target",
                "description": "Human-readable description"
            }}
        ]

        Be specific about locations and objects when possible.
        """

    def plan_complex_task(self, command_text: str, environment_context: Optional[Dict] = None) -> Optional[List[Dict]]:
        """
        Use LLM to plan complex multi-step tasks.

        Args:
            command_text (str): The natural language command to plan
            environment_context (Optional[Dict]): Current environment context

        Returns:
            Optional[List[Dict]]: List of actions to execute, or None if planning failed
        """
        # Prepare the environment context
        context = environment_context or {}
        environment_info = f"Environment: {json.dumps(context)}" if context else "Environment: Unknown"

        # Create the user prompt
        user_prompt = f"""
        Command: {command_text}
        {environment_info}

        Plan the sequence of actions to execute this command:
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.1,
                max_tokens=500
            )

            plan_text = response.choices[0].message['content'].strip()

            # Extract JSON from the response (in case it includes other text)
            json_match = re.search(r'\[.*\]', plan_text, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group())
                return plan
            else:
                # If no JSON found, return None
                print(f"No JSON found in LLM response: {plan_text}")
                return None

        except Exception as e:
            print(f"Error in LLM planning: {e}")
            return None


class RuleBasedCognitivePlanner:
    """
    A rule-based planner as an alternative to LLM-based planning.
    """

    def __init__(self):
        """
        Initialize the rule-based cognitive planner.
        """
        # Define rules for common command patterns
        self.rules = {
            r'.*go to.*kitchen.*and.*bring.*': self._plan_kitchen_fetch_task,
            r'.*go to.*office.*and.*bring.*': self._plan_office_fetch_task,
            r'.*go to.*bedroom.*and.*bring.*': self._plan_bedroom_fetch_task,
            r'.*pick up.*and.*bring.*': self._plan_simple_fetch_task,
            r'.*go to.*and.*get.*': self._plan_fetch_task,
            r'.*navigate to.*': self._plan_navigation_task,
        }

    def plan_task(self, command_text: str, environment_context: Optional[Dict] = None) -> Optional[List[Dict]]:
        """
        Plan a task using rule-based approach.

        Args:
            command_text (str): The natural language command to plan
            environment_context (Optional[Dict]): Current environment context

        Returns:
            Optional[List[Dict]]: List of actions to execute, or None if planning failed
        """
        command_lower = command_text.lower()

        # Try to match against rules
        for pattern, rule_func in self.rules.items():
            if re.search(pattern, command_lower):
                return rule_func(command_text, environment_context)

        # If no rule matches, return None
        return None

    def _plan_kitchen_fetch_task(self, command_text: str, env_context: Optional[Dict]) -> List[Dict]:
        """
        Plan a kitchen fetch task.
        """
        # Extract object to fetch
        object_match = re.search(r'bring.*?(\w+)', command_text.lower())
        target_object = object_match.group(1) if object_match else 'object'

        return [
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'kitchen',
                'description': 'Navigate to kitchen area'
            },
            {
                'action_type': 'perception',
                'action': 'find_object',
                'target_object': target_object,
                'description': f'Detect and locate the {target_object}'
            },
            {
                'action_type': 'manipulation',
                'action': 'grasp',
                'target_object': target_object,
                'description': f'Grasp the {target_object}'
            },
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'origin',
                'description': 'Return to original location'
            },
            {
                'action_type': 'manipulation',
                'action': 'place',
                'target_object': target_object,
                'location': 'near user',
                'description': f'Place the {target_object} near the user'
            }
        ]

    def _plan_office_fetch_task(self, command_text: str, env_context: Optional[Dict]) -> List[Dict]:
        """
        Plan an office fetch task.
        """
        # Extract object to fetch
        object_match = re.search(r'bring.*?(\w+)', command_text.lower())
        target_object = object_match.group(1) if object_match else 'object'

        return [
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'office',
                'description': 'Navigate to office area'
            },
            {
                'action_type': 'perception',
                'action': 'find_object',
                'target_object': target_object,
                'description': f'Detect and locate the {target_object}'
            },
            {
                'action_type': 'manipulation',
                'action': 'grasp',
                'target_object': target_object,
                'description': f'Grasp the {target_object}'
            },
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'origin',
                'description': 'Return to original location'
            },
            {
                'action_type': 'manipulation',
                'action': 'place',
                'target_object': target_object,
                'location': 'near user',
                'description': f'Place the {target_object} near the user'
            }
        ]

    def _plan_bedroom_fetch_task(self, command_text: str, env_context: Optional[Dict]) -> List[Dict]:
        """
        Plan a bedroom fetch task.
        """
        # Extract object to fetch
        object_match = re.search(r'bring.*?(\w+)', command_text.lower())
        target_object = object_match.group(1) if object_match else 'object'

        return [
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'bedroom',
                'description': 'Navigate to bedroom area'
            },
            {
                'action_type': 'perception',
                'action': 'find_object',
                'target_object': target_object,
                'description': f'Detect and locate the {target_object}'
            },
            {
                'action_type': 'manipulation',
                'action': 'grasp',
                'target_object': target_object,
                'description': f'Grasp the {target_object}'
            },
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'origin',
                'description': 'Return to original location'
            },
            {
                'action_type': 'manipulation',
                'action': 'place',
                'target_object': target_object,
                'location': 'near user',
                'description': f'Place the {target_object} near the user'
            }
        ]

    def _plan_simple_fetch_task(self, command_text: str, env_context: Optional[Dict]) -> List[Dict]:
        """
        Plan a simple fetch task.
        """
        # Extract object to fetch
        object_match = re.search(r'pick up.*?(\w+)', command_text.lower())
        target_object = object_match.group(1) if object_match else 'object'

        return [
            {
                'action_type': 'perception',
                'action': 'find_object',
                'target_object': target_object,
                'description': f'Detect and locate the {target_object}'
            },
            {
                'action_type': 'manipulation',
                'action': 'grasp',
                'target_object': target_object,
                'description': f'Grasp the {target_object}'
            },
            {
                'action_type': 'manipulation',
                'action': 'place',
                'target_object': target_object,
                'location': 'near user',
                'description': f'Place the {target_object} near the user'
            }
        ]

    def _plan_fetch_task(self, command_text: str, env_context: Optional[Dict]) -> List[Dict]:
        """
        Plan a general fetch task.
        """
        # Extract location and object
        location_match = re.search(r'go to.*?(\w+)', command_text.lower())
        location = location_match.group(1) if location_match else 'unknown'

        object_match = re.search(r'get.*?(\w+)', command_text.lower())
        target_object = object_match.group(1) if object_match else 'object'

        return [
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': location,
                'description': f'Navigate to {location}'
            },
            {
                'action_type': 'perception',
                'action': 'find_object',
                'target_object': target_object,
                'description': f'Detect and locate the {target_object}'
            },
            {
                'action_type': 'manipulation',
                'action': 'grasp',
                'target_object': target_object,
                'description': f'Grasp the {target_object}'
            },
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': 'origin',
                'description': 'Return to original location'
            },
            {
                'action_type': 'manipulation',
                'action': 'place',
                'target_object': target_object,
                'location': 'near user',
                'description': f'Place the {target_object} near the user'
            }
        ]

    def _plan_navigation_task(self, command_text: str, env_context: Optional[Dict]) -> List[Dict]:
        """
        Plan a navigation task.
        """
        # Extract target location
        location_match = re.search(r'navigate to.*?(\w+)', command_text.lower())
        target_location = location_match.group(1) if location_match else 'unknown'

        return [
            {
                'action_type': 'navigation',
                'action': 'navigate_to',
                'target_location': target_location,
                'description': f'Navigate to {target_location}'
            }
        ]


class IntegratedCognitivePlanner:
    """
    Combines LLM-based and rule-based planning approaches.
    """

    def __init__(self):
        """
        Initialize the integrated cognitive planner.
        """
        self.llm_planner = LLMCognitivePlanner()
        self.rule_planner = RuleBasedCognitivePlanner()

    def plan_task(self, command_text: str, environment_context: Optional[Dict] = None,
                  use_llm: bool = True) -> Optional[List[Dict]]:
        """
        Plan a task using the appropriate method.

        Args:
            command_text (str): The natural language command to plan
            environment_context (Optional[Dict]): Current environment context
            use_llm (bool): Whether to use LLM-based planning (True) or rule-based (False)

        Returns:
            Optional[List[Dict]]: List of actions to execute, or None if planning failed
        """
        if use_llm:
            # Try LLM-based planning first
            plan = self.llm_planner.plan_complex_task(command_text, environment_context)
            if plan:
                return plan
            else:
                # Fall back to rule-based planning
                print("LLM planning failed, falling back to rule-based planning")
                return self.rule_planner.plan_task(command_text, environment_context)
        else:
            # Use rule-based planning
            return self.rule_planner.plan_task(command_text, environment_context)

    def validate_plan(self, plan: List[Dict], environment_context: Optional[Dict] = None) -> bool:
        """
        Validate a planned sequence of actions.

        Args:
            plan (List[Dict]): The plan to validate
            environment_context (Optional[Dict]): Current environment context

        Returns:
            bool: True if plan is valid, False otherwise
        """
        if not plan or len(plan) == 0:
            return False

        # Check if all required resources are available
        for action in plan:
            action_type = action.get('action_type')
            if action_type == 'navigation':
                target_location = action.get('target_location')
                if not target_location:
                    return False
            elif action_type == 'manipulation':
                target_object = action.get('target_object')
                if not target_object:
                    return False

        return True