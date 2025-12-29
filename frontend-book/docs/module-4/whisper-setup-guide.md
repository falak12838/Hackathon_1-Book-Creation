# Whisper Setup Guide for VLA System

This guide will help you set up OpenAI Whisper for voice processing in the Vision-Language-Action (VLA) system.

## Prerequisites

Before setting up Whisper integration, ensure you have:

1. An OpenAI API key
2. Python 3.8 or higher
3. ROS 2 Humble Hawksbill installed

## Installation

### 1. Install Required Python Packages

```bash
pip install openai python-dotenv
```

### 2. Set Up Environment Variables

Create a `.env` file in your project root:

```env
OPENAI_API_KEY=your_openai_api_key_here
WHISPER_MODEL=whisper-1
```

### 3. Verify Installation

Test that you can import the Whisper API:

```python
import openai
import os
from dotenv import load_dotenv

load_dotenv()
openai.api_key = os.getenv("OPENAI_API_KEY")

# Test the API
response = openai.Model.list()
print("Whisper setup successful!")
```

## Integration with VLA System

### Speech Processor Module

The VLA system includes a `SpeechProcessor` class that handles Whisper integration:

```python
from vla.speech_processor import SpeechProcessor

# Initialize the processor
processor = SpeechProcessor()

# Transcribe an audio file
transcription = processor.transcribe_audio("path/to/audio.wav")
print(f"Transcription: {transcription}")
```

## Configuration

### Whisper Model Selection

You can configure which Whisper model to use in your environment:

```python
# In your configuration
WHISPER_MODEL = "whisper-1"  # Production model
# WHISPER_MODEL = "whisper-1-medium"  # Alternative model
```

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure your OpenAI API key is correct and has sufficient credits
2. **Audio Format Issues**: Whisper works best with WAV, MP3, or FLAC formats
3. **Network Issues**: Verify internet connectivity for API calls

### Performance Tips

- Use high-quality audio input for better transcription accuracy
- Consider audio preprocessing to reduce noise
- Implement caching for frequently used commands

## Next Steps

Once Whisper is set up, proceed to the Cognitive Planning section to learn how to process the transcribed text into robot actions.