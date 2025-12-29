"""
Action Executor module for the Vision-Language-Action (VLA) system.
Executes structured actions on the robot.
"""

import time
import json
from typing import Dict, List, Any, Optional
from enum import Enum


class ExecutionStatus(Enum):
    """
    Enum for action execution status.
    """
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ActionExecutor:
    """
    Executes structured actions on the robot.
    """

    def __init__(self):
        """
        Initialize the action executor.
        """
        self.current_action = None
        self.execution_status = ExecutionStatus.PENDING
        self.action_history = []
        self.emergency_stop = False

    def execute_action_sequence(self, action_sequence: List[Dict]) -> Dict[str, Any]:
        """
        Execute a sequence of actions.

        Args:
            action_sequence (List[Dict]): List of structured actions to execute

        Returns:
            Dict[str, Any]: Execution result with status and details
        """
        if self.emergency_stop:
            return {
                'status': 'failed',
                'error': 'Emergency stop active',
                'completed_actions': [],
                'failed_action': None
            }

        completed_actions = []
        result = {
            'status': 'completed',
            'completed_actions': completed_actions,
            'failed_action': None,
            'execution_time': 0
        }

        start_time = time.time()

        for i, action in enumerate(action_sequence):
            if self.emergency_stop:
                result['status'] = 'failed'
                result['error'] = 'Emergency stop during execution'
                result['failed_action'] = action
                break

            try:
                action_result = self.execute_single_action(action)
                completed_actions.append({
                    'action': action,
                    'result': action_result,
                    'index': i
                })

                if not action_result.get('success', False):
                    result['status'] = 'failed'
                    result['error'] = action_result.get('error', 'Unknown error')
                    result['failed_action'] = action
                    break

            except Exception as e:
                result['status'] = 'failed'
                result['error'] = f'Exception during action execution: {str(e)}'
                result['failed_action'] = action
                break

        result['execution_time'] = time.time() - start_time
        return result

    def execute_single_action(self, action: Dict) -> Dict[str, Any]:
        """
        Execute a single structured action.

        Args:
            action (Dict): The structured action to execute

        Returns:
            Dict[str, Any]: Execution result with success status and details
        """
        if self.emergency_stop:
            return {
                'success': False,
                'error': 'Emergency stop active',
                'action_type': action.get('action_type', 'unknown')
            }

        action_type = action.get('action_type', 'unknown')
        action_name = action.get('action', 'unknown')

        self.current_action = action
        self.execution_status = ExecutionStatus.EXECUTING

        try:
            if action_type == 'navigation':
                result = self.execute_navigation_action(action)
            elif action_type == 'manipulation':
                result = self.execute_manipulation_action(action)
            elif action_type == 'perception':
                result = self.execute_perception_action(action)
            elif action_type == 'communication':
                result = self.execute_communication_action(action)
            elif action_type == 'control':
                result = self.execute_control_action(action)
            else:
                result = {
                    'success': False,
                    'error': f'Unknown action type: {action_type}'
                }

            # Add action type to result for tracking
            result['action_type'] = action_type
            result['action_name'] = action_name

            return result

        except Exception as e:
            error_result = {
                'success': False,
                'error': f'Exception in {action_type} action: {str(e)}',
                'action_type': action_type,
                'action_name': action_name
            }
            return error_result

    def execute_navigation_action(self, action: Dict) -> Dict[str, Any]:
        """
        Execute navigation action.

        Args:
            action (Dict): The navigation action to execute

        Returns:
            Dict[str, Any]: Execution result
        """
        try:
            action_name = action.get('action', 'unknown')
            target_location = action.get('target_location') or action.get('parameter')

            if action_name == 'move_forward':
                # Simulate moving forward
                time.sleep(1)  # Simulate execution time
                return {'success': True, 'distance': '1 meter', 'direction': 'forward'}

            elif action_name == 'move_backward':
                # Simulate moving backward
                time.sleep(1)
                return {'success': True, 'distance': '1 meter', 'direction': 'backward'}

            elif action_name == 'turn_left':
                # Simulate turning left
                time.sleep(0.5)
                return {'success': True, 'angle': '90 degrees', 'direction': 'left'}

            elif action_name == 'turn_right':
                # Simulate turning right
                time.sleep(0.5)
                return {'success': True, 'angle': '90 degrees', 'direction': 'right'}

            elif action_name == 'navigate_to':
                if not target_location:
                    return {'success': False, 'error': 'Target location not specified'}

                # Simulate navigation to location
                time.sleep(2)  # Simulate navigation time
                return {'success': True, 'target': target_location}

            else:
                return {'success': False, 'error': f'Unknown navigation action: {action_name}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def execute_manipulation_action(self, action: Dict) -> Dict[str, Any]:
        """
        Execute manipulation action.

        Args:
            action (Dict): The manipulation action to execute

        Returns:
            Dict[str, Any]: Execution result
        """
        try:
            action_name = action.get('action', 'unknown')
            target_object = action.get('target_object') or action.get('parameter')

            if action_name == 'pick_up' or action_name == 'grasp':
                if not target_object:
                    return {'success': False, 'error': 'Target object not specified'}

                # Simulate picking up object
                time.sleep(1)
                return {'success': True, 'object': target_object, 'action': 'picked up'}

            elif action_name == 'put_down' or action_name == 'place' or action_name == 'release':
                if not target_object:
                    return {'success': False, 'error': 'Target object not specified'}

                # Simulate placing object
                time.sleep(1)
                return {'success': True, 'object': target_object, 'action': 'placed'}

            else:
                return {'success': False, 'error': f'Unknown manipulation action: {action_name}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def execute_perception_action(self, action: Dict) -> Dict[str, Any]:
        """
        Execute perception action.

        Args:
            action (Dict): The perception action to execute

        Returns:
            Dict[str, Any]: Execution result
        """
        try:
            action_name = action.get('action', 'unknown')
            target_object = action.get('target_object') or action.get('parameter')

            if action_name == 'find_object' or action_name == 'locate_object':
                if not target_object:
                    return {'success': False, 'error': 'Target object not specified'}

                # Simulate object finding/locating
                time.sleep(1)
                # Simulate finding the object (in a real system, this would use sensors)
                return {
                    'success': True,
                    'object': target_object,
                    'location': 'detected',
                    'confidence': 0.9
                }

            else:
                return {'success': False, 'error': f'Unknown perception action: {action_name}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def execute_communication_action(self, action: Dict) -> Dict[str, Any]:
        """
        Execute communication action.

        Args:
            action (Dict): The communication action to execute

        Returns:
            Dict[str, Any]: Execution result
        """
        try:
            action_name = action.get('action', 'unknown')
            message = action.get('parameter')

            if action_name == 'speak':
                if not message:
                    return {'success': False, 'error': 'Message not specified'}

                # Simulate speaking (in a real system, this would use TTS)
                print(f"Robot says: {message}")
                time.sleep(len(message.split()) * 0.3)  # Simulate speaking time
                return {'success': True, 'message': message}

            else:
                return {'success': False, 'error': f'Unknown communication action: {action_name}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def execute_control_action(self, action: Dict) -> Dict[str, Any]:
        """
        Execute control action.

        Args:
            action (Dict): The control action to execute

        Returns:
            Dict[str, Any]: Execution result
        """
        try:
            action_name = action.get('action', 'unknown')

            if action_name == 'stop':
                # Simulate stopping robot
                time.sleep(0.1)
                return {'success': True, 'action': 'stopped'}

            elif action_name == 'pause':
                # Simulate pausing robot
                time.sleep(0.1)
                return {'success': True, 'action': 'paused'}

            elif action_name == 'resume':
                # Simulate resuming robot
                time.sleep(0.1)
                return {'success': True, 'action': 'resumed'}

            else:
                return {'success': False, 'error': f'Unknown control action: {action_name}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def trigger_emergency_stop(self):
        """
        Trigger emergency stop to halt all ongoing actions.
        """
        self.emergency_stop = True
        self.execution_status = ExecutionStatus.CANCELLED
        print("Emergency stop activated - all actions halted")

    def clear_emergency_stop(self):
        """
        Clear emergency stop and reset executor.
        """
        self.emergency_stop = False
        self.execution_status = ExecutionStatus.PENDING
        print("Emergency stop cleared - ready for new actions")

    def get_execution_status(self) -> Dict[str, Any]:
        """
        Get current execution status.

        Returns:
            Dict[str, Any]: Current status information
        """
        return {
            'current_action': self.current_action,
            'execution_status': self.execution_status.value,
            'emergency_stop': self.emergency_stop,
            'action_history_count': len(self.action_history)
        }