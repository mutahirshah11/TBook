# Implementation Plan: Part 6 - Conversational Robotics

**Branch**: `013-part6-conversational-robotics` | **Date**: 2025-12-07 | **Spec**: [specs/013-part6-conversational-robotics/spec.md](spec.md)
**Input**: Feature specification from `specs/013-part6-conversational-robotics/spec.md`

## Summary

Part 6 of the textbook explores the integration of Large Language Models (LLMs) and multi-modal interaction into robotics. It covers the setup of local LLMs (Ollama) for text generation, offline speech recognition (Vosk), and Zero-Shot Visual Question Answering (VQA) pipelines. The goal is to build a robot that can understand natural language commands ("Pick up the red cup") and ground them in visual reality.

## Technical Context

**Language/Version**: Python 3.8+ (ROS 2 Humble nodes)
**Primary Dependencies**: 
- `ollama` (Local LLM inference)
- `vosk` (Offline Speech-to-Text)
- `transformers` (Hugging Face for VQA/CLIP)
- `rclpy` (ROS 2 Python client)
- `sounddevice` / `pyaudio` (Audio capture)
**Storage**: N/A (Local model weights for Ollama and Vosk)
**Testing**: Manual verification of dialogue loops and VQA accuracy in simulation.
**Target Platform**: Linux (Ubuntu 22.04) with NVIDIA GPU (for LLM/VQA speed)
**Project Type**: Educational Content (Markdown Docusaurus) + Code Examples (ROS 2 Nodes)
**Performance Goals**: 
- LLM Latency < 2s
- STT Word Error Rate < 10%
**Constraints**: 
- Must run offline (no reliance on OpenAI API for core path).
- Must be modular (ROS 2 nodes).
**Scale/Scope**: 3 Chapters + ~3-5 ROS 2 nodes.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Accuracy**: Content must correctly describe the integration of asynchronous AI services with real-time ROS loops.
- **Clarity**: Flesch-Kincaid grade 10-12.
- **Reproducibility**: `requirements.txt` and explicit model versions (e.g., `llama3`, `vosk-model-small-en-us`) required.
- **Rigor**: Based on current state-of-the-art in Embodied AI (Zero-Shot VQA).
- **Integrity**: Original tutorial content.
- **Test-Driven Development**: "Tests" are user verification steps (e.g., "Verify the robot responds to 'Hello'").

## Project Structure

### Documentation (this feature)

```text
specs/013-part6-conversational-robotics/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
Textbook/
├── docs/
│   └── part6/
│       ├── chapter-21-llm-integration.md
│       ├── chapter-22-speech-nlu.md
│       └── chapter-23-multimodal.md
```

*Note: The ROS 2 package code examples will be embedded in the markdown files.*

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | N/A | N/A |