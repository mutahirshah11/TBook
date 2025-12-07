# Quickstart: Part 6 - Conversational Robotics

**Prerequisites**:
- Ubuntu 22.04 / ROS 2 Humble
- Python 3.10+
- NVIDIA GPU (recommended for LLMs)

## 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y python3-pip portaudio19-dev espeak-ng
```

## 2. Install Python Libraries

We use a virtual environment or install strictly to the user site to avoid breaking system python.

```bash
pip install ollama vosk sounddevice pyaudio transformers torch
# For TTS
pip install piper-tts
```

## 3. Setup Ollama

1.  **Install**: `curl -fsSL https://ollama.com/install.sh | sh`
2.  **Pull Model**: `ollama pull llama3`
3.  **Verify**:
    ```bash
    ollama run llama3 "Hello, are you a robot?"
    ```

## 4. Setup Vosk

1.  Download the model:
    ```bash
    wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
    unzip vosk-model-small-en-us-0.15.zip
    mv vosk-model-small-en-us-0.15 model
    ```

## 5. Run the Demo

Create a test script `test_pipeline.py` to verify the stack.

```python
import ollama
from vosk import Model, KaldiRecognizer
import pyaudio

# 1. Speech
model = Model("model")
rec = KaldiRecognizer(model, 16000)
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)

print("Listening...")
# ... (Audio Loop) ...

# 2. LLM
response = ollama.chat(model='llama3', messages=[
  {'role': 'user', 'content': 'Why is the sky blue?'}
])
print(response['message']['content'])
```
