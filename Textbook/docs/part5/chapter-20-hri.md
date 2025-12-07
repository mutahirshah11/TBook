---
sidebar_label: 'Chapter 20: HRI'
sidebar_position: 20
---

# Chapter 20: Natural Human-Robot Interaction Design

## Introduction

A humanoid robot that requires a keyboard to operate is just a machine. A humanoid that responds to a wave and a voice command feels like a partner. This chapter integrates **Keyword Spotting** and **Gesture Recognition** to build a natural, multimodal interface for the Talos robot.

## The Interaction Loop

We will build a **Finite State Machine (FSM)** to manage the interaction.
1.  **Idle**: Robot scans for a person.
2.  **Listening**: Triggered by a "Wave" gesture. Robot looks at user.
3.  **Active**: Processing voice command (e.g., "Grasp that").
4.  **Executing**: Robot performs the task.

## Keyword Spotting (KWS)

We don't need a full Large Language Model (LLM) running locally to understand basic commands. **Keyword Spotting** is efficient and low-latency. We use **Vosk**, which runs offline and fits on embedded CPUs.

### Code Example: `voice_command.py`

This script uses a grammar constraint to only listen for specific words, vastly increasing accuracy.

```python
from vosk import Model, KaldiRecognizer
import pyaudio
import json

# 1. Load Model
# Download 'vosk-model-small-en-us-0.15' for speed
model = Model("model")

# 2. Constrain Grammar
# Only listen for these specific JSON tokens
grammar = '["robot", "stop", "go", "grasp", "left", "right", "[unk]"]'
rec = KaldiRecognizer(model, 16000, grammar)

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)

print("Listening...")

while True:
    data = stream.read(4000, exception_on_overflow=False)
    if rec.AcceptWaveform(data):
        result = json.loads(rec.Result())
        text = result['text']
        
        if "robot grasp" in text:
            print("CMD: GRASP_INITIATED")
            # trigger_grasp_fsm()
        elif "robot stop" in text:
            print("CMD: E-STOP")
            # trigger_estop()
```

## Gesture Recognition with MediaPipe

**Google MediaPipe** offers real-time hand tracking. Instead of just detecting a "hand", we will compute **geometric features** to recognize gestures.

### Vector Math for Gestures
-   **"Stop" Gesture**: All 5 fingers extended, palm facing camera.
    -   Check `dist(wrist, fingertip)` is large for all fingers.
-   **"Point" Gesture**: Index finger extended, others curled.
    -   The vector `v = (index_tip - index_mcp)` defines the pointing direction in 3D space.

### Code Example: `gesture_detect.py`

```python
import mediapipe as mp
import cv2
import numpy as np

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)

def get_pointing_vector(landmarks):
    # Extract 3D coordinates
    index_tip = np.array([landmarks[8].x, landmarks[8].y, landmarks[8].z])
    index_mcp = np.array([landmarks[5].x, landmarks[5].y, landmarks[5].z])
    
    # Vector from knuckle to tip
    vector = index_tip - index_mcp
    return vector / np.linalg.norm(vector)

def main():
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        success, image = cap.read()
        results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        
        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                # Logic: Is Index extended? Are others curled?
                # ... (Geometric checks) ...
                
                vec = get_pointing_vector(hand_lms.landmark)
                print(f"User is pointing: {vec}")
                
                # Visualization
                mp.solutions.drawing_utils.draw_landmarks(image, hand_lms, mp_hands.HAND_CONNECTIONS)
                
        cv2.imshow('HRI View', image)
        if cv2.waitKey(5) & 0xFF == 27: break

    cap.release()
```

## Integration: The "Look-and-Point" System

By combining these, we solve the "Grounding Problem" (what does "that" mean?).
1.  **Voice**: User says "Look at *that*".
2.  **Gesture**: User points.
3.  **Fusion**: The robot intersects the **Pointing Ray** (from MediaPipe) with the **World Map** (Octomap/Collision map). The first object hit by the ray is the target "that".

## Summary

HRI is about establishing a shared reality. By implementing robust keyword spotting and geometric gesture parsing, we allow the user to guide the robot's high-level autonomy ("Go there") without needing to joystick it manually ("Move forward 5 meters").