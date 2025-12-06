# Conceptual API Contracts: Humanoid Robotics Book

**Feature Branch**: `1-readme-spec-creation` | **Date**: 2025-12-04 | **Spec**: [specs/1-readme-spec-creation/spec.md](specs/1-readme-spec-creation/spec.md)

## Summary

This document outlines the conceptual API contracts and interaction protocols relevant to the "Humanoid Robotics" book. While not traditional software API specifications, these contracts define how different components within robotics frameworks (like ROS 2) and integrated AI systems (like LLMs and VLA) communicate, exchange data, and handle errors. This ensures a clear understanding of system-level interactions for educational purposes.

## 1. ROS 2 Communication Contracts

### a. Topics (Publish/Subscribe)
- **Purpose**: Real-time, asynchronous data streaming (e.g., sensor data, joint states, robot commands).
- **Inputs (Publisher)**: Data message based on a defined ROS 2 message type (e.g., `sensor_msgs/Image`, `geometry_msgs/Twist`).
- **Outputs (Subscriber)**: Consumes data messages of the specified type.
- **Error Handling**: Lost messages are generally not retransmitted; subscribers might receive stale data or no data if publisher fails. QoS settings can mitigate some issues.

### b. Services (Request/Response)
- **Purpose**: Synchronous, RPC-like calls for specific computations or actions (e.g., requesting a robot's current pose, triggering a single action).
- **Inputs (Client)**: Service request message based on a defined ROS 2 service type.
- **Outputs (Server)**: Service response message based on the defined ROS 2 service type.
- **Error Handling**: Server can return an error status within the response. Client typically blocks until response or timeout. Network errors can lead to call failures.

### c. Actions (Goal/Feedback/Result)
- **Purpose**: Long-running, asynchronous tasks with pre-emptability and continuous feedback (e.g., complex navigation tasks, sequence of manipulations).
- **Inputs (Client)**: Action goal message.
- **Outputs (Server)**: Action result message, with periodic feedback messages.
- **Error Handling**: Server reports status (e.g., `SUCCEEDED`, `ABORTED`, `REJECTED`) in the result. Client can cancel goals. Timeout mechanisms apply.

## 2. LLM-Robot Interface Contracts (VLA System)

### a. Voice-to-Action Pipeline (Conceptual)
- **Purpose**: Translate natural language voice commands into executable robot actions.
- **Components Involved**:
  - **Speech-to-Text (STT)** (e.g., OpenAI Whisper):
    - **Input**: Audio stream of human speech.
    - **Output**: Text transcription of the speech.
    - **Error Handling**: Transcription errors due to noise, accents, or ambiguous speech; handled by confidence scores or re-prompting.
  - **Natural Language Understanding (NLU) / LLM Integration**:
    - **Input**: Text transcription from STT, current robot state/context.
    - **Output**: Structured robot command (e.g., JSON specifying ROS 2 action, target object, location).
    - **Error Handling**: Misinterpretation of commands (hallucinations), ambiguous instructions, commands outside robot capabilities; handled by LLM's confidence, clarification dialogues, or fallback to default behaviors.

### b. Perception-to-Planning Interface
- **Purpose**: Provide processed sensor data to navigation and planning modules.
- **Inputs (Perception Module)**: Raw sensor streams (e.g., LiDAR point clouds, camera images, IMU data).
- **Outputs (Planning Module)**: Semantic maps, object detections, estimated robot pose, obstacle maps.
- **Error Handling**: Inaccurate or incomplete perception due to sensor noise, occlusion; handled by sensor fusion, filtering, and robust planning algorithms.

### c. Action Execution Feedback
- **Purpose**: Report the status and outcome of executed robot actions back to higher-level AI components (e.g., LLM, VLA system).
- **Inputs (Robot Controller)**: Current joint states, task completion status, error codes from actuators.
- **Outputs (AI System)**: Structured feedback on action success/failure, current robot state, environmental changes.
- **Error Handling**: Action failures (e.g., dropped object, collision); handled by retry mechanisms, re-planning, or reporting back for human intervention/re-evaluation by LLM.
