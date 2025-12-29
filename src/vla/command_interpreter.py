"""
Command Interpreter module for the Vision-Language-Action (VLA) system.
Translates natural language commands into structured robot actions.
"""

import re
from typing import Dict, List, Optional, Tuple


class CommandInterpreter:
    """
    Interprets natural language commands and converts them to structured actions.
    """

    def __init__(self):
        """
        Initialize the command interpreter with predefined command patterns.
        """
        # Define command patterns and their corresponding action types
        self.command_patterns = {
            # Navigation commands
            r'.*move forward.*': {'action_type': 'navigation', 'action': 'move_forward'},
            r'.*move backward.*': {'action_type': 'navigation', 'action': 'move_backward'},
            r'.*go forward.*': {'action_type': 'navigation', 'action': 'move_forward'},
            r'.*go back.*': {'action_type': 'navigation', 'action': 'move_backward'},
            r'.*move up.*': {'action_type': 'navigation', 'action': 'move_up'},
            r'.*move down.*': {'action_type': 'navigation', 'action': 'move_down'},
            r'.*turn left.*': {'action_type': 'navigation', 'action': 'turn_left'},
            r'.*turn right.*': {'action_type': 'navigation', 'action': 'turn_right'},
            r'.*rotate left.*': {'action_type': 'navigation', 'action': 'turn_left'},
            r'.*rotate right.*': {'action_type': 'navigation', 'action': 'turn_right'},
            r'.*go to (.+)': {'action_type': 'navigation', 'action': 'navigate_to'},
            r'.*move to (.+)': {'action_type': 'navigation', 'action': 'navigate_to'},

            # Manipulation commands
            r'.*pick up (.+)': {'action_type': 'manipulation', 'action': 'pick_up'},
            r'.*grasp (.+)': {'action_type': 'manipulation', 'action': 'grasp'},
            r'.*grab (.+)': {'action_type': 'manipulation', 'action': 'grasp'},
            r'.*lift (.+)': {'action_type': 'manipulation', 'action': 'grasp'},
            r'.*put down (.+)': {'action_type': 'manipulation', 'action': 'put_down'},
            r'.*place (.+)': {'action_type': 'manipulation', 'action': 'place'},
            r'.*release (.+)': {'action_type': 'manipulation', 'action': 'release'},
            r'.*drop (.+)': {'action_type': 'manipulation', 'action': 'release'},

            # Object interaction
            r'.*find (.+)': {'action_type': 'perception', 'action': 'find_object'},
            r'.*locate (.+)': {'action_type': 'perception', 'action': 'locate_object'},
            r'.*look for (.+)': {'action_type': 'perception', 'action': 'find_object'},
            r'.*search for (.+)': {'action_type': 'perception', 'action': 'find_object'},

            # Communication
            r'.*say (.+)': {'action_type': 'communication', 'action': 'speak'},
            r'.*speak (.+)': {'action_type': 'communication', 'action': 'speak'},

            # Control
            r'.*stop': {'action_type': 'control', 'action': 'stop'},
            r'.*halt': {'action_type': 'control', 'action': 'stop'},
            r'.*pause': {'action_type': 'control', 'action': 'pause'},
            r'.*resume': {'action_type': 'control', 'action': 'resume'},
        }

        # Define location keywords
        self.locations = {
            'kitchen', 'living room', 'bedroom', 'office', 'bathroom',
            'dining room', 'hallway', 'garage', 'garden', 'outside'
        }

        # Define common objects
        self.objects = {
            'cup', 'book', 'ball', 'bottle', 'phone', 'keys', 'box',
            'chair', 'table', 'door', 'window', 'light', 'red cup',
            'blue cup', 'green cup', 'small object', 'large object'
        }

    def interpret(self, text: str) -> Optional[Dict]:
        """
        Interpret natural language text into a structured action.

        Args:
            text (str): The natural language command to interpret

        Returns:
            Optional[Dict]: Structured action dictionary or None if command is not recognized
        """
        if not text or len(text.strip()) == 0:
            return None

        text_lower = text.lower().strip()

        # Try to match against command patterns
        for pattern, action_template in self.command_patterns.items():
            match = re.search(pattern, text_lower)
            if match:
                # Create the action with extracted parameters
                action = action_template.copy()

                # Extract parameters from the match (if any)
                if match.groups():
                    parameter = match.group(1).strip()
                    action['parameter'] = parameter

                    # Try to classify the parameter
                    if parameter in self.locations:
                        action['target_location'] = parameter
                    elif parameter in self.objects:
                        action['target_object'] = parameter
                    else:
                        # Could be an object or location - default to object
                        action['target_object'] = parameter

                # Add the original command for reference
                action['original_command'] = text
                action['confidence'] = 0.9  # Placeholder confidence

                return action

        # If no pattern matched, return None
        return None

    def extract_location(self, command_text: str) -> Optional[str]:
        """
        Extract target location from command text.

        Args:
            command_text (str): The command text to analyze

        Returns:
            Optional[str]: The extracted location or None if not found
        """
        for location in self.locations:
            if location in command_text.lower():
                return location
        return None

    def extract_object(self, command_text: str) -> Optional[str]:
        """
        Extract target object from command text.

        Args:
            command_text (str): The command text to analyze

        Returns:
            Optional[str]: The extracted object or None if not found
        """
        for obj in self.objects:
            if obj in command_text.lower():
                return obj
        return None

    def validate_command(self, command: Dict) -> Tuple[bool, str]:
        """
        Validate a structured command.

        Args:
            command (Dict): The structured command to validate

        Returns:
            Tuple[bool, str]: (is_valid, error_message)
        """
        if not command:
            return False, "Command is None"

        action_type = command.get('action_type')
        if not action_type:
            return False, "Command missing action_type"

        action = command.get('action')
        if not action:
            return False, "Command missing action"

        # Validate based on action type
        if action_type == 'navigation':
            if action == 'navigate_to':
                target = command.get('target_location') or command.get('parameter')
                if not target:
                    return False, "Navigation command missing target location"

        elif action_type == 'manipulation':
            target = command.get('target_object') or command.get('parameter')
            if not target:
                return False, "Manipulation command missing target object"

        return True, "Command is valid"

    def get_supported_commands(self) -> List[str]:
        """
        Get a list of supported command patterns.

        Returns:
            List[str]: List of supported command patterns
        """
        return list(self.command_patterns.keys())