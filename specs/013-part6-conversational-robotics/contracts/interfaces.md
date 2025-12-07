# ROS 2 Interface Contracts

## 1. LLM Generation Action

**Action**: `robot_interfaces/action/GenerateText`

**Goal**:
```ros
string prompt
string system_prompt "You are a helpful robot."
float32 temperature 0.7
```

**Result**:
```ros
string response
bool success
```

**Feedback**:
```ros
string partial_response
```

## 2. VQA Service

**Service**: `robot_interfaces/srv/VisualQuery`

**Request**:
```ros
sensor_msgs/Image image
string question
```

**Response**:
```ros
string answer
float32 confidence
```

## 3. Speech Topics

**Topic**: `/audio/speech_text`
- **Type**: `std_msgs/String`
- **Source**: Vosk Node
- **Semantics**: The final transcribed text after a silence pause.

**Topic**: `/audio/speak`
- **Type**: `std_msgs/String`
- **Sink**: Piper TTS Node
- **Semantics**: Text to be synthesized and played immediately.

## Grounding Flow

1.  **User**: "Pick up the red apple."
2.  **STT Node**: Publishes "Pick up the red apple" to `/audio/speech_text`.
3.  **Orchestrator Node**:
    -   Extracts intent: `action="PICK", target_desc="red apple"`.
    -   Captures latest image from `/camera/rgb`.
    -   Calls VQA Service with image + "Where is the red apple?".
4.  **VQA Node**: Returns bounding box or coordinate.
5.  **Orchestrator**: Sends goal to manipulation planner.
