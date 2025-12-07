# Data Model: Part 6 - Conversational Robotics

**Feature**: `013-part6-conversational-robotics` | **Date**: 2025-12-07

## Entities

### LLM Interaction
*Represents the state of the conversation.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **Conversation Context** | Buffer of recent messages. | `history`: list[dict] (Role: "user"\|"assistant", Content: string) |
| **Prompt Template** | System instructions for the robot. | `system_prompt`: string (e.g., "You are a robot helper...") |
| **Generation Request** | Input to the LLM. | `prompt`: string<br>`model`: string ("llama3")<br>`stream`: bool |

### ROS 2 Interfaces
*Custom messages and actions.*

| Entity | Description | Fields |
| :--- | :--- | :--- |
| **GenerateText.action** | Goal: Prompt<br>Result: Text<br>Feedback: Token stream | `string prompt`<br>`---`<br>`string response`<br>`---`<br>`string partial_text` |
| **SpeechAudio** | Raw audio stream. | `int16[] data` (chunked) |
| **VQAQuery** | Visual question. | `sensor_msgs/Image image`<br>`string question` |

### Multi-Modal State
*Fused understanding of the world.*

| Entity | Description | Attributes |
| :--- | :--- | :--- |
| **DetectedObject** | An object found by VQA/Vision. | `label`: string<br>`bbox`: [x, y, w, h]<br>`confidence`: float |
| **UserIntent** | Structured command. | `action`: enum (PICK, GOTO)<br>`target_id`: string (Grounding ID) |

## Validation Rules

1. **Context Window**: The `Conversation Context` must be truncated if it exceeds the model's context length (e.g., 4096 tokens).
2. **Action Timeout**: The LLM Action Server must abort if generation takes longer than 30 seconds to prevent blocking behaviors.
3. **Grammar Enforcement**: The STT engine should only accept words defined in the active grammar set to reduce hallucinations.

## State Transitions

### Dialogue Loop
1.  **Listening**: Audio buffer fills -> Voice Activity Detection (VAD) triggers -> STT.
2.  **Thinking**: STT Text -> Prompt Construction -> LLM Action Call.
3.  **Speaking**: LLM Result -> TTS Service -> Audio Playback.
4.  **Acting**: Intent Parser -> Behavior Tree execution.
