# Research: Part 6 - Conversational Robotics

**Feature**: `013-part6-conversational-robotics` | **Date**: 2025-12-07

## Decisions

### 1. LLM Integration Method
- **Decision**: Build a **Custom ROS 2 Action Server** wrapper for Ollama.
- **Rationale**:
  - LLM inference is a long-running task (seconds to minutes).
  - ROS 2 Actions (`action_msgs`) provide built-in feedback and cancellation mechanisms, which are critical for stopping a robot if it's hallucinating or taking too long.
  - Existing community nodes are either too simple (CLI-focused) or too complex (web-UI focused). A custom wrapper is educational and precise.
- **Implementation**: `OllamaActionServer` node that calls `ollama.generate()` and streams tokens via action feedback.

### 2. Speech Recognition (STT)
- **Decision**: **Vosk** with `vosk-model-small-en-us-0.15`.
- **Rationale**:
  - Truly offline and lightweight enough for embedded robotics (Raspberry Pi / Jetson).
  - "Small" model offers the best latency/accuracy trade-off for command recognition (latency < 500ms).
  - Supports grammar constraints (e.g., only listen for "robot", "stop", "go"), which drastically improves accuracy in noisy environments.
- **Alternatives**:
  - *Whisper (OpenAI)*: Too slow for real-time CPU inference without heavy optimization (Whisper.cpp).
  - *Pocketsphinx*: Outdated accuracy.

### 3. Text-to-Speech (TTS)
- **Decision**: **Piper TTS** (via `piper_ros` or direct python wrapper).
- **Rationale**:
  - State-of-the-art offline neural TTS.
  - Faster than real-time on Raspberry Pi 4.
  - "Spiritual successor" to Mimic 3, with better maintenance.
  - `pyttsx3` is too robotic; `Coqui` is too heavy.
- **Integration**: A simple subscriber node that takes `std_msgs/String` and plays audio.

### 4. Zero-Shot VQA
- **Decision**: **Hugging Face Transformers** with `CLIP` or `LlaVA` (quantized).
- **Rationale**:
  - `transformers` is the standard API.
  - A VQA node will subscribe to camera images and text queries.
  - Zero-shot allows recognizing "red cup" without training a custom YOLO model.
- **Optimization**: Use 4-bit quantization (`bitsandbytes`) to fit VQA models on consumer GPUs.

## Unknowns Resolved

- **ROS 2 Pattern**: Action Server is the correct pattern for LLMs.
- **STT Model**: Vosk "Small" model + Grammar is the winning combo.
- **TTS**: Piper is the modern choice for offline neural speech.
- **VQA**: Standard HF Transformers pipeline is sufficient.
