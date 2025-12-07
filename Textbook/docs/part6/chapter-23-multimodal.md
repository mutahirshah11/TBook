---
sidebar_label: 'Chapter 23: Multi-Modal'
sidebar_position: 23
---

# Chapter 23: Multi-Modal Interaction

## Introduction

Humans communicate with more than just words. We point, we look, and we refer to objects in our shared environment. "Pick up *that* cup" is meaningless without visual context. This chapter combines Vision, Language, and Action into a single **Multi-Modal** pipeline.

## Zero-Shot Visual Question Answering (VQA)

Traditional computer vision requires training a model on specific classes (e.g., "cup", "bottle"). **Zero-Shot** models like **CLIP** (Contrastive Language-Image Pre-training) or **LlaVA** (Large Language-and-Vision Assistant) can recognize *any* object described in text.

### The Workflow
1.  **Input**: Image + Question ("Where is the red cup?").
2.  **Model**: Encodes image and text into a shared embedding space.
3.  **Output**: Bounding box coordinates or a text answer.

### Code Example: VQA with Transformers

```python
from transformers import pipeline
from PIL import Image
import requests

# Load Zero-Shot Object Detection pipeline
detector = pipeline(model="google/owlvit-base-patch32", task="zero-shot-object-detection")

def find_object(image, description):
    predictions = detector(
        image,
        candidate_labels=[description],
    )
    # predictions = [{'box': {'xmin': 32, 'ymin': 50, ...}, 'score': 0.99, 'label': 'red cup'}]
    return predictions
```

## The Grounding Pipeline

**Grounding** is the process of linking a linguistic symbol ("red cup") to a physical entity in the world (ID: 42, Pos: [0.5, 0.2, 0.8]).

1.  **Speech**: User says "Pick up the red cup."
2.  **Intent Parser**: Extracts target object description: `target="red cup"`.
3.  **Vision**: VQA Node searches the current camera frame for "red cup".
4.  **Fusion**:
    -   If object found: Convert 2D bounding box to 3D coordinates using Depth camera.
    -   Send `Pick(x,y,z)` command to the robot arm.
    -   Respond via TTS: "Picking up the red cup."
    -   If not found: Respond: "I don't see a red cup."

## Handling Ambiguity

What if the user says "Pick up the cup," but there are two cups? A robust system detects this ambiguity.

1.  **Count**: VQA detects 2 objects matching "cup".
2.  **Dialogue Policy**:
    -   If count == 1: Proceed.
    -   If count > 1: Ask for clarification.
    -   **Robot**: "I see two cups. Do you mean the left one or the right one?"
3.  **Context**: The user replies "The left one." The system updates the target filter.

## Summary

By combining LLMs for logic, VQA for perception, and TTS/STT for communication, we create a system that feels intelligent. The robot isn't just executing code; it's collaborating with you in the physical world.
