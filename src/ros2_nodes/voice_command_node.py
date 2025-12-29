#!/usr/bin/env python3

"""
Voice Command Node for the Vision-Language-Action (VLA) system.
Processes voice commands and converts them to robot actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
from sensor_msgs.msg import AudioData as SensorAudioData
import os
from dotenv import load_dotenv
import tempfile
import json
from typing import Optional

# Load environment variables
load_dotenv()

# Import local modules
try:
    from vla.speech_processor import SpeechProcessor
    from vla.command_interpreter import CommandInterpreter
except ImportError:
    # If not in the same package, try alternative import
    from src.vla.speech_processor import SpeechProcessor
    from src.vla.command_interpreter import CommandInterpreter


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize speech processor and command interpreter
        try:
            self.speech_processor = SpeechProcessor()
            self.command_interpreter = CommandInterpreter()
        except ValueError as e:
            self.get_logger().error(f'Failed to initialize VLA components: {e}')
            raise

        # Publishers
        self.command_publisher = self.create_publisher(
            String,
            'robot_command',
            10
        )

        # Publishers for intermediate results
        self.transcription_publisher = self.create_publisher(
            String,
            'voice_transcription',
            10
        )

        self.interpretation_publisher = self.create_publisher(
            String,
            'command_interpretation',
            10
        )

        # Subscribers
        # Note: Using std_msgs.String for simplicity; in practice, you'd use proper audio message types
        self.audio_subscription = self.create_subscription(
            String,
            'audio_input',
            self.audio_callback,
            10
        )

        # For real audio input, you might also subscribe to sensor_msgs.AudioData
        self.real_audio_subscription = self.create_subscription(
            SensorAudioData,
            'microphone/audio',
            self.real_audio_callback,
            10
        )

        self.get_logger().info('Voice Command Node initialized and ready')

    def audio_callback(self, msg):
        """
        Callback for audio input (as String message for simulation).
        In a real implementation, this would handle actual audio data.
        """
        self.get_logger().info(f'Received audio command: {msg.data}')

        try:
            # For simulation purposes, treat the string as a command
            # In real implementation, this would process actual audio data
            command_text = msg.data

            # Publish the command for downstream processing
            self.command_publisher.publish(msg)
            self.get_logger().info(f'Published robot command: {command_text}')

        except Exception as e:
            self.get_logger().error(f'Error processing audio callback: {e}')

    def real_audio_callback(self, msg):
        """
        Callback for real audio data from microphone.
        """
        self.get_logger().info('Received real audio data')

        try:
            # Convert audio data to temporary file for processing
            # Note: This is a simplified example; real audio processing
            # would require proper audio format handling
            audio_bytes = bytes(msg.data)

            # For demonstration, we'll create a temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_audio:
                # Write audio data to temp file
                # Note: This is simplified - real implementation would need
                # proper audio format conversion
                temp_audio.write(audio_bytes)
                temp_filename = temp_audio.name

            try:
                # Transcribe the audio
                transcription = self.speech_processor.transcribe_audio(temp_filename)

                if transcription:
                    self.get_logger().info(f'Transcribed: {transcription}')

                    # Publish transcription
                    transcription_msg = String()
                    transcription_msg.data = transcription
                    self.transcription_publisher.publish(transcription_msg)

                    # Interpret the command
                    interpreted_command = self.command_interpreter.interpret(transcription)

                    if interpreted_command:
                        self.get_logger().info(f'Interpreted command: {interpreted_command}')

                        # Publish interpretation
                        interpretation_msg = String()
                        interpretation_msg.data = json.dumps(interpreted_command)
                        self.interpretation_publisher.publish(interpretation_msg)

                        # Publish the interpreted command for higher-level processing
                        robot_cmd_msg = String()
                        robot_cmd_msg.data = transcription  # Use original text for now
                        self.command_publisher.publish(robot_cmd_msg)

                        self.get_logger().info(f'Published interpreted command: {transcription}')
                    else:
                        self.get_logger().warn(f'Could not interpret command: {transcription}')
                else:
                    self.get_logger().warn('Could not transcribe audio')

            finally:
                # Clean up temp file
                os.unlink(temp_filename)

        except Exception as e:
            self.get_logger().error(f'Error processing real audio: {e}')

    def process_audio_file(self, audio_file_path: str) -> Optional[str]:
        """
        Process an audio file and return the transcribed text.

        Args:
            audio_file_path (str): Path to the audio file

        Returns:
            Optional[str]: Transcribed text or None if processing failed
        """
        try:
            transcription = self.speech_processor.transcribe_audio(audio_file_path)
            return transcription
        except Exception as e:
            self.get_logger().error(f'Error processing audio file {audio_file_path}: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)

    voice_command_node = VoiceCommandNode()

    try:
        rclpy.spin(voice_command_node)
    except KeyboardInterrupt:
        voice_command_node.get_logger().info('Voice Command Node interrupted')
    finally:
        voice_command_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()