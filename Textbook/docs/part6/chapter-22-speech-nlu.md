---
sidebar_label: 'Chapter 22: Speech & NLU'
sidebar_position: 22
---

# Chapter 22: Speech Recognition and Natural Language Understanding

## Introduction

Voice is the most natural interface for humans. In this chapter, we give the robot ears (Speech-to-Text) and a voice (Text-to-Speech), creating a full verbal interaction loop.

## Offline Speech-to-Text with Vosk

**Vosk** is an offline speech recognition toolkit capable of running on embedded devices like the Raspberry Pi 4. It is robust to noise and doesn't require an API key.

### Code Example: Speech Publisher Node

This node listens to the microphone and publishes the text only when a sentence is complete.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer
import pyaudio
import json

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self.pub = self.create_publisher(String, '/audio/speech_text', 10)
        
        model = Model("model") # Path to unzipped Vosk model
        self.rec = KaldiRecognizer(model, 16000)
        
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16, 
            channels=1, 
            rate=16000, 
            input=True, 
            frames_per_buffer=8000)
            
        self.timer = self.create_timer(0.1, self.listen_loop)

    def listen_loop(self):
        data = self.stream.read(4000, exception_on_overflow=False)
        if self.rec.AcceptWaveform(data):
            result = json.loads(self.rec.Result())
            text = result['text']
            if text:
                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"Heard: {text}")

def main():
    # ... standard boilerplate ...
```

## Intent Classification with Grammars

NLU (Natural Language Understanding) converts raw text ("Go to the kitchen") into structured intent (`NAVIGATE`, `KITCHEN`).

Vosk supports **Grammar Constraints**. By providing a list of allowed words, you drastically increase accuracy and reduce "hallucinations" (hearing words that weren't said).

```python
# Only listen for these words
grammar = '["robot", "stop", "go", "kitchen", "living room"]'
rec = KaldiRecognizer(model, 16000, grammar)
```

Now, if someone says "Um, robot, please go to the kitchen now," Vosk will robustly extract "robot go kitchen".

## Text-to-Speech with Piper

**Piper** is a fast, local neural text-to-speech system. It sounds nearly human and runs faster than real-time even on low-end hardware.

### Integration
We create a Subscriber Node that listens to `/audio/speak`:

```python
import subprocess

def speak_callback(self, msg):
    text = msg.data
    # Pipe text to the piper executable
    cmd = f"echo '{text}' | piper --model en_US-lessac-medium.onnx --output_raw | aplay -r 22050 -f S16_LE -t raw -"
    subprocess.Popen(cmd, shell=True)
```

## Closing the Loop

1.  **Vosk Node** publishes: "Robot, tell me a joke."
2.  **Brain Node** (Chapter 21) receives text, calls LLM Action.
3.  **LLM** returns: "Why did the robot cross the road? To optimize its path."
4.  **Brain Node** publishes response to `/audio/speak`.
5.  **Piper Node** reads text and plays audio.
