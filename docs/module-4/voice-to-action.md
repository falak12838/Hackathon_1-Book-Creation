# Voice-to-Action: OpenAI Whisper for Command Interpretation

## Overview

In this module, we'll explore how to integrate OpenAI Whisper for converting voice commands into actionable instructions for humanoid robots. This forms the foundation of the Vision-Language-Action (VLA) system, enabling natural voice interaction with robotic systems.

## Learning Objectives

By the end of this chapter, you will:
- Understand how OpenAI Whisper works for speech recognition
- Implement a voice command processing pipeline
- Integrate Whisper with ROS 2 for real-time command interpretation
- Create basic command interpretation for simple robot actions

## Prerequisites

- Basic understanding of Python programming
- Familiarity with ROS 2 concepts
- OpenAI API key for Whisper service
- Microphone for voice input

## Setting Up OpenAI Whisper Integration

### Installation

First, let's install the required dependencies for Whisper integration:

```bash
pip install openai python-dotenv
```

### Basic Whisper Implementation

Here's a simple implementation of Whisper for voice command processing:

```python
import openai
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure OpenAI API
openai.api_key = os.getenv("OPENAI_API_KEY")

class SpeechProcessor:
    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not found in environment variables")

    def transcribe_audio(self, audio_file_path):
        """
        Transcribe audio file using OpenAI Whisper
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
                return transcript.text
        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return None

    def process_voice_command(self, audio_data):
        """
        Process voice command and return interpreted action
        """
        # This is a simplified example - in practice, you'd save audio_data to a file
        # and then transcribe it
        pass
```

## Integrating with ROS 2

To integrate Whisper with ROS 2, we'll create a ROS 2 node that listens for audio input and processes it:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import openai
import os
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Subscribe to audio input
        self.audio_subscription = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Publisher for interpreted commands
        self.command_publisher = self.create_publisher(
            String,
            'robot_command',
            10
        )

        # Initialize speech processor
        self.speech_processor = SpeechProcessor()

        self.get_logger().info('Voice Command Node initialized')

    def audio_callback(self, msg):
        """
        Callback for audio input
        """
        self.get_logger().info('Received audio data')
        # Process the audio and publish interpreted command
        # (Implementation details would go here)
```

## Command Interpretation

Once we have the transcribed text, we need to interpret it into robot actions. Here's a basic command interpreter:

```python
class CommandInterpreter:
    def __init__(self):
        self.commands = {
            'move forward': 'move_forward',
            'move backward': 'move_backward',
            'turn left': 'turn_left',
            'turn right': 'turn_right',
            'stop': 'stop',
            'pick up': 'pick_up',
            'put down': 'put_down'
        }

    def interpret(self, text):
        """
        Interpret transcribed text into robot command
        """
        text_lower = text.lower().strip()

        for command_phrase, command_id in self.commands.items():
            if command_phrase in text_lower:
                return command_id

        return None  # Unknown command
```

## Complete Implementation Example

Here's a complete example of the voice processing system:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import openai
import os
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from dotenv import load_dotenv
import tempfile
import wave

# Load environment variables
load_dotenv()

class SpeechProcessor:
    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not found in environment variables")

    def transcribe_audio(self, audio_file_path):
        """
        Transcribe audio file using OpenAI Whisper
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
                return transcript.text
        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return None

class CommandInterpreter:
    def __init__(self):
        self.commands = {
            'move forward': 'move_forward',
            'move backward': 'move_backward',
            'turn left': 'turn_left',
            'turn right': 'turn_right',
            'stop': 'stop',
            'pick up': 'pick_up',
            'put down': 'put_down'
        }

    def interpret(self, text):
        """
        Interpret transcribed text into robot command
        """
        text_lower = text.lower().strip()

        for command_phrase, command_id in self.commands.items():
            if command_phrase in text_lower:
                return command_id

        return None  # Unknown command

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Subscribe to audio input
        self.audio_subscription = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        # Publisher for interpreted commands
        self.command_publisher = self.create_publisher(
            String,
            'robot_command',
            10
        )

        # Initialize speech processor
        self.speech_processor = SpeechProcessor()
        self.command_interpreter = CommandInterpreter()

        self.get_logger().info('Voice Command Node initialized')

    def audio_callback(self, msg):
        """
        Callback for audio input
        """
        self.get_logger().info('Received audio data')

        # Save audio data to temporary file for processing
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_audio:
            # Write audio data to temp file
            # (This is a simplified example - actual implementation would depend on audio format)
            temp_filename = temp_audio.name

        try:
            # Transcribe the audio
            transcription = self.speech_processor.transcribe_audio(temp_filename)

            if transcription:
                self.get_logger().info(f'Transcribed: {transcription}')

                # Interpret the command
                command = self.command_interpreter.interpret(transcription)

                if command:
                    # Publish the interpreted command
                    command_msg = String()
                    command_msg.data = command
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info(f'Published command: {command}')
                else:
                    self.get_logger().warn(f'Unknown command: {transcription}')
            else:
                self.get_logger().warn('Could not transcribe audio')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
        finally:
            # Clean up temp file
            os.unlink(temp_filename)

def main(args=None):
    rclpy.init(args=args)

    voice_command_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_command_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_command_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing the Voice-to-Action System

To test the system, you can run the ROS 2 node and provide audio input:

```bash
# In one terminal
ros2 run vla voice_command_node.py

# In another terminal, publish test audio data
# (This would typically come from a microphone)
```

## Testing with Simulated Audio

For testing purposes, you can create a simple audio publisher:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestAudioPublisher(Node):
    def __init__(self):
        super().__init__('test_audio_publisher')
        self.publisher = self.create_publisher(String, 'robot_command', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'move_forward'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    test_publisher = TestAudioPublisher()

    try:
        rclpy.spin(test_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        test_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

1. **Error Handling**: Always implement robust error handling for network requests to the Whisper API
2. **Confidence Scoring**: Use Whisper's confidence scoring to filter low-quality transcriptions
3. **Audio Preprocessing**: Preprocess audio to reduce noise and improve transcription quality
4. **Caching**: Cache common commands to reduce API calls and improve response time
5. **Security**: Never log or store voice data that contains sensitive information

## Summary

In this chapter, we've covered the basics of integrating OpenAI Whisper for voice command interpretation. We've created a foundation for processing voice commands and translating them into robot actions. In the next chapter, we'll explore how to use cognitive planning to translate these interpreted commands into complex robot behaviors.

## Next Steps

- Implement the cognitive planning system (next chapter)
- Integrate with humanoid robot control systems
- Add more sophisticated command interpretation
- Implement safety features and emergency stop capabilities