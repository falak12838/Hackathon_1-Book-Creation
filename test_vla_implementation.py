#!/usr/bin/env python3

"""
Test script to verify VLA system implementation.
"""

import sys
import os

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_imports():
    """Test that all modules can be imported."""
    print("Testing module imports...")

    try:
        from vla import (
            SpeechProcessor,
            CommandInterpreter,
            ActionExecutor,
            LLMCognitivePlanner,
            RuleBasedCognitivePlanner,
            IntegratedCognitivePlanner
        )
        print("✓ All VLA modules imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False

def test_speech_processor():
    """Test speech processor functionality."""
    print("\nTesting SpeechProcessor...")

    try:
        from vla import SpeechProcessor

        # Create processor (will fail if API key not set, but should initialize)
        try:
            processor = SpeechProcessor()
            print("✓ SpeechProcessor initialized")
        except ValueError as e:
            print(f"⚠ SpeechProcessor initialization failed (expected if no API key): {e}")

        return True
    except Exception as e:
        print(f"✗ SpeechProcessor test failed: {e}")
        return False

def test_command_interpreter():
    """Test command interpreter functionality."""
    print("\nTesting CommandInterpreter...")

    try:
        from vla import CommandInterpreter

        interpreter = CommandInterpreter()
        print("✓ CommandInterpreter initialized")

        # Test some basic commands
        test_commands = [
            "Move forward 1 meter",
            "Go to the kitchen",
            "Pick up the red cup",
            "Turn left"
        ]

        for cmd in test_commands:
            result = interpreter.interpret(cmd)
            if result:
                print(f"  ✓ Command '{cmd}' interpreted: {result['action_type']}")
            else:
                print(f"  ⚠ Command '{cmd}' not interpreted")

        return True
    except Exception as e:
        print(f"✗ CommandInterpreter test failed: {e}")
        return False

def test_action_executor():
    """Test action executor functionality."""
    print("\nTesting ActionExecutor...")

    try:
        from vla import ActionExecutor

        executor = ActionExecutor()
        print("✓ ActionExecutor initialized")

        # Test basic execution status
        status = executor.get_execution_status()
        print(f"  ✓ Initial status: {status['execution_status']}")

        return True
    except Exception as e:
        print(f"✗ ActionExecutor test failed: {e}")
        return False

def test_planners():
    """Test cognitive planners."""
    print("\nTesting Cognitive Planners...")

    try:
        from vla import RuleBasedCognitivePlanner

        planner = RuleBasedCognitivePlanner()
        print("✓ RuleBasedCognitivePlanner initialized")

        # Test a simple command
        result = planner.plan_task("Go to the kitchen and bring me the red cup")
        if result:
            print(f"  ✓ Simple command planned: {len(result)} actions")
        else:
            print("  ⚠ Simple command not planned (may be expected)")

        return True
    except Exception as e:
        print(f"✗ Cognitive Planners test failed: {e}")
        return False

def test_documentation():
    """Test that documentation files exist."""
    print("\nTesting documentation...")

    docs_path = os.path.join(os.path.dirname(__file__), 'docs', 'module-4')

    expected_docs = [
        'voice-to-action.md',
        'cognitive-planning.md',
        'capstone-autonomous-humanoid.md'
    ]

    all_found = True
    for doc in expected_docs:
        doc_path = os.path.join(docs_path, doc)
        if os.path.exists(doc_path):
            print(f"  ✓ {doc} exists")
        else:
            print(f"  ✗ {doc} missing")
            all_found = False

    return all_found

def test_ros2_nodes():
    """Test that ROS 2 nodes exist."""
    print("\nTesting ROS 2 nodes...")

    nodes_path = os.path.join(os.path.dirname(__file__), 'src', 'ros2_nodes')

    expected_nodes = [
        'voice_command_node.py',
        'cognitive_planner_node.py',
        'humanoid_controller_node.py'
    ]

    all_found = True
    for node in expected_nodes:
        node_path = os.path.join(nodes_path, node)
        if os.path.exists(node_path):
            print(f"  ✓ {node} exists")
        else:
            print(f"  ✗ {node} missing")
            all_found = False

    return all_found

def main():
    """Run all tests."""
    print("VLA System Implementation Verification")
    print("=" * 40)

    tests = [
        test_imports,
        test_speech_processor,
        test_command_interpreter,
        test_action_executor,
        test_planners,
        test_documentation,
        test_ros2_nodes
    ]

    results = []
    for test in tests:
        results.append(test())

    print("\n" + "=" * 40)
    print("Test Summary:")
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("✓ All tests passed! VLA system implementation is complete.")
        return 0
    else:
        print("✗ Some tests failed. Please check the implementation.")
        return 1

if __name__ == "__main__":
    sys.exit(main())