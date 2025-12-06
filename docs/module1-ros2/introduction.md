# ROS 2 for Humanoid Control - Introduction

## Summary

This chapter introduces the Robotic Operating System 2 (ROS 2) as the fundamental "nervous system" for controlling humanoid robots. It covers the core architecture of ROS 2, explaining key concepts such as nodes, topics, services, and actions, which enable distributed communication and control within a robotic system.

## Learning Objectives

Upon completing this chapter, you will be able to:
- Understand the high-level architecture of ROS 2.
- Identify and describe the function of ROS 2 nodes, topics, services, and actions.
- Explain how these communication mechanisms facilitate inter-component communication in a robot control system.
- Recognize the importance of ROS 2 as a middleware for complex robotic applications, particularly for humanoids.

## 1.1 What is ROS 2?

ROS 2 is an open-source, flexible framework for writing robot software. It is not an operating system in the traditional sense, but rather a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior. Designed for modern robotics, ROS 2 addresses the needs of real-time control, multi-robot systems, and embedded platforms, providing a robust foundation for humanoid robotics.

## 1.2 ROS 2 Core Architecture

At its heart, ROS 2 is built around a distributed communication graph where various independent processes (nodes) exchange information. This architecture promotes modularity and reusability, allowing developers to build complex robot behaviors by combining smaller, specialized components.

### 1.2.1 Nodes

A **Node** is an executable process that performs computations. In a robotic system, individual functionalities—such as controlling a motor, reading sensor data, or performing a navigation algorithm—are encapsulated within nodes. Each node is designed to be self-contained and communicates with other nodes to achieve larger tasks.

### 1.2.2 Topics

**Topics** are the primary mechanism for asynchronous, one-to-many communication in ROS 2. Nodes publish data to topics, and other nodes can subscribe to these topics to receive that data. This publish-subscribe model is ideal for streaming continuous data, like sensor readings (e.g., camera images, LiDAR scans) or joint states of a humanoid robot.

- **Publisher**: A node that sends messages to a topic.
- **Subscriber**: A node that receives messages from a topic.
- **Message**: The data structure exchanged over a topic. ROS 2 provides a rich set of standard message types, and users can define custom ones.

### 1.2.3 Services

**Services** provide a synchronous, request-response communication pattern. Unlike topics, which are one-way data streams, services are used when a node needs to request a specific action or piece of information from another node and wait for a response. This is analogous to a function call in a distributed system.

- **Service Server**: A node that offers a service and processes requests.
- **Service Client**: A node that sends a request to a service server and waits for a response.

### 1.2.4 Actions

**Actions** are designed for long-running tasks that require continuous feedback and can be preempted. They combine aspects of both topics and services, allowing a client to send a goal, receive periodic feedback on the progress, and ultimately get a result. This is particularly useful for complex humanoid movements like walking to a target or performing a manipulation sequence, where intermediate progress updates are crucial.

- **Action Client**: Sends a goal, receives feedback, and waits for a result.
- **Action Server**: Receives a goal, provides feedback, and computes a final result.

## Conclusion

ROS 2 provides a powerful and flexible framework for building sophisticated robotic applications. By understanding its core communication mechanisms—nodes, topics, services, and actions—you gain the foundational knowledge to design, implement, and control complex humanoid robot systems. The modularity and distributed nature of ROS 2 are critical for managing the complexity inherent in advanced robotics.
