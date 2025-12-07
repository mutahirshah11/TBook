---
sidebar_label: 'Chapter 21: LLM Integration'
sidebar_position: 21
---

# Chapter 21: Integrating GPT Models for Conversational AI

## Introduction

Large Language Models (LLMs) like GPT-4 and Llama 3 have revolutionized natural language understanding. In robotics, they act as the "cognitive engine," allowing robots to reason about tasks, plan actions, and converse with humans. This chapter covers how to integrate these models into a ROS 2 system.

## Cloud vs. Local LLMs

### Cloud APIs (e.g., OpenAI GPT-4)
- **Pros**: Highest intelligence, easy to use API, no local hardware required.
- **Cons**: Latency (network + inference), privacy concerns, cost.

### Local LLMs (e.g., Ollama / Llama 3)
- **Pros**: Privacy (data never leaves the robot), consistent latency, free.
- **Cons**: Requires powerful hardware (GPU/RAM), slightly lower reasoning capability compared to massive cloud models.

For autonomous robots, **Local LLMs** are often preferred to ensure operation without internet access. We will use **Ollama**, a lightweight framework for running Llama 3, Mistral, and other open models.

## The ROS 2 Action Server Pattern

LLM generation takes time—from milliseconds to seconds. Using a standard ROS 2 Service (`srv`) blocks the client until generation is complete, which can freeze your robot's behavior loop.

Instead, we use a **ROS 2 Action**.
- **Goal**: The prompt ("Plan a path to the kitchen").
- **Feedback**: Stream of tokens as they are generated ("To...", "get...", "to...", "the...").
- **Result**: The final complete response.

This allows the robot to "think out loud" or cancel the thought process if the situation changes.

## Tutorial: Defining the GenerateText Action

Create a new file `action/GenerateText.action` in your package:

```text
# Goal
string prompt
string system_prompt "You are a helpful robot assistant."
float32 temperature 0.7
---
# Result
string response
bool success
---
# Feedback
string partial_response
```

## Code Example: Ollama Action Server

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from robot_interfaces.action import GenerateText
import ollama

class OllamaActionServer(Node):
    def __init__(self):
        super().__init__('ollama_server')
        self._action_server = ActionServer(
            self,
            GenerateText,
            'generate_text',
            self.execute_callback)
        self.get_logger().info("Ollama Action Server Ready")

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Generating for: {goal_handle.request.prompt}")
        feedback_msg = GenerateText.Feedback()
        
        # Stream response from Ollama
        full_response = ""
        stream = ollama.chat(
            model='llama3',
            messages=[
                {'role': 'system', 'content': goal_handle.request.system_prompt},
                {'role': 'user', 'content': goal_handle.request.prompt}
            ],
            stream=True,
        )

        for chunk in stream:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return GenerateText.Result(success=False, response=full_response)
            
            token = chunk['message']['content']
            full_response += token
            feedback_msg.partial_response = full_response
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        return GenerateText.Result(success=True, response=full_response)

def main(args=None):
    rclpy.init(args=args)
    node = OllamaActionServer()
    rclpy.spin(node)
```

## Prompt Engineering for Robots

Robots need structured outputs (e.g., JSON coordinates), not poetry. Use the **System Prompt** to enforce constraints:

> "You are a navigation agent. Output ONLY a JSON object with keys 'target' and 'action'. Do not include markdown formatting."

By constraining the output, you can parse the LLM's response directly into robot commands.
