"""
Vision-Language-Action (VLA) System Package.

This package contains modules for processing voice commands,
interpreting natural language, planning actions, and executing
robot behaviors for humanoid robots.
"""

# Import main classes for easy access
from .speech_processor import SpeechProcessor
from .command_interpreter import CommandInterpreter
from .action_executor import ActionExecutor, ExecutionStatus
from .cognitive_planner import LLMCognitivePlanner, RuleBasedCognitivePlanner, IntegratedCognitivePlanner

__all__ = [
    'SpeechProcessor',
    'CommandInterpreter',
    'ActionExecutor',
    'ExecutionStatus',
    'LLMCognitivePlanner',
    'RuleBasedCognitivePlanner',
    'IntegratedCognitivePlanner'
]

__version__ = '1.0.0'
__author__ = 'VLA Development Team'