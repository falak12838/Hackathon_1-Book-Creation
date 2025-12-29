"""
Speech Processor module for the Vision-Language-Action (VLA) system.
Handles voice command processing using OpenAI Whisper API.
"""

import openai
import os
from dotenv import load_dotenv
import tempfile
import wave
import io

# Load environment variables
load_dotenv()

class SpeechProcessor:
    def __init__(self):
        """
        Initialize the speech processor with OpenAI API configuration.
        """
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key not found in environment variables")

        openai.api_key = self.api_key

    def transcribe_audio(self, audio_file_path):
        """
        Transcribe audio file using OpenAI Whisper API.

        Args:
            audio_file_path (str): Path to the audio file to transcribe

        Returns:
            str: The transcribed text, or None if transcription failed
        """
        try:
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
                return transcript.text
        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return None

    def transcribe_audio_bytes(self, audio_bytes, file_extension=".wav"):
        """
        Transcribe audio from bytes using OpenAI Whisper API.

        Args:
            audio_bytes (bytes): Audio data as bytes
            file_extension (str): File extension for the temporary file (default: .wav)

        Returns:
            str: The transcribed text, or None if transcription failed
        """
        try:
            # Create a temporary file to store the audio bytes
            with tempfile.NamedTemporaryFile(suffix=file_extension, delete=False) as temp_audio:
                temp_audio.write(audio_bytes)
                temp_filename = temp_audio.name

            try:
                # Transcribe the temporary file
                result = self.transcribe_audio(temp_filename)
                return result
            finally:
                # Clean up the temporary file
                os.unlink(temp_filename)
        except Exception as e:
            print(f"Error transcribing audio bytes: {e}")
            return None

    def validate_transcription(self, transcription, confidence_threshold=0.7):
        """
        Validate transcription quality based on simple heuristics.

        Args:
            transcription (str): The transcribed text
            confidence_threshold (float): Minimum confidence threshold (placeholder)

        Returns:
            tuple: (is_valid: bool, confidence_score: float)
        """
        if not transcription or len(transcription.strip()) == 0:
            return False, 0.0

        # Simple validation: check for minimum length and common issues
        transcription = transcription.strip().lower()

        # Check for common transcription artifacts
        if len(transcription) < 2:
            return False, 0.1

        # Placeholder for more sophisticated validation
        # In a real implementation, you might use additional NLP techniques
        # or confidence scoring from the Whisper API if available
        return True, 0.9  # Placeholder confidence score