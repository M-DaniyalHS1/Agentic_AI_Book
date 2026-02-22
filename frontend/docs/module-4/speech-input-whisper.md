---
sidebar_position: 1
---

# Speech Input with Whisper

## Overview

OpenAI's Whisper is a state-of-the-art speech recognition model that:
- Supports 99 languages
- Handles accents and background noise
- Provides timestamps and confidence scores
- Runs locally (no API required)

## Installation

```bash
pip install openai-whisper
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Basic Usage

```python
import whisper

# Load model
model = whisper.load_model("base")  # Options: tiny, base, small, medium, large

# Transcribe audio
result = model.transcribe("audio.wav")
print(result["text"])

# Transcribe with options
result = model.transcribe(
    "audio.wav",
    language="en",
    task="transcribe",  # or "translate" to English
    verbose=True
)

# Get segments with timestamps
for segment in result["segments"]:
    print(f"[{segment['start']:.2f}s - {segment['end']:.2f}s]: {segment['text']}")
```

## Real-Time Speech Recognition

```python
import whisper
import numpy as np
import sounddevice as sd

class SpeechRecognizer:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.sample_rate = 16000
        self.buffer = []
    
    def start_listening(self, duration=5):
        """Record audio for specified duration"""
        print(f"Listening for {duration} seconds...")
        audio = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1
        )
        sd.wait()
        return audio.flatten()
    
    def transcribe(self, audio):
        """Transcribe audio array"""
        # Ensure audio is float32 and normalized
        audio = audio.astype(np.float32) / 32768.0
        
        result = self.model.transcribe(audio)
        return result["text"]

# Usage
recognizer = SpeechRecognizer()
audio = recognizer.start_listening(duration=5)
text = recognizer.transcribe(audio)
print(f"You said: {text}")
```

## Integration with ROS 2

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import threading

class SpeechCommandNode(Node):
    def __init__(self):
        super().__init__('speech_command_node')
        self.publisher = self.create_publisher(String, '/speech/command', 10)
        self.model = whisper.load_model('base')
        
        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()
    
    def listen_loop(self):
        import sounddevice as sd
        while rclpy.ok():
            audio = sd.rec(16000 * 5, samplerate=16000, channels=1)
            sd.wait()
            text = self.transcribe(audio)
            if text.strip():
                self.publish_command(text)
    
    def transcribe(self, audio):
        audio = audio.astype(np.float32) / 32768.0
        result = self.model.transcribe(audio)
        return result["text"]
    
    def publish_command(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f'Command: "{text}"')

def main():
    rclpy.init()
    node = SpeechCommandNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
```

## Lab Exercise 4.1

Create a voice-controlled robot:
1. Set up Whisper for speech recognition
2. Define voice commands ("move forward", "stop", "turn left")
3. Publish commands to ROS 2 topics
4. Control a simulated robot with voice

## Next Steps

Learn about LLM-based cognitive planning.
