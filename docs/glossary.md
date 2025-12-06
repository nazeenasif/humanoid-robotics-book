# Glossary of Terms

This glossary provides definitions for key terms used throughout the Humanoid Robotics Book. Terms are organized alphabetically for easy reference.

## A

### Action (in VLA Systems)
In Vision-Language-Action (VLA) systems, the "Action" component refers to the physical or behavioral response executed by the robot based on visual perception and language understanding.

### Action Server
A ROS 2 component that handles long-running goals with feedback, allowing clients to send goals, receive feedback during execution, and get results when the goal is completed.

### Adaptive Control
A control system that adjusts its parameters in real-time based on changing conditions or system dynamics to maintain optimal performance.

### Affordance
The potential actions that an object or environment offers to an agent, particularly relevant in robotics for understanding how objects can be manipulated.

### AI Robot Brain
A comprehensive system integrating perception, decision-making, and action execution capabilities in humanoid robots using artificial intelligence techniques.

### Anthropomorphic
Having human-like characteristics, particularly referring to robots designed with human-like form and movement patterns.

## B

### Behavior Tree
A hierarchical structure used in robotics and AI for organizing and executing complex behaviors, providing a flexible alternative to finite state machines.

### Bidirectional Encoder Representations from Transformers (BERT)
A transformer-based model for natural language processing that pre-trains on large text corpora to understand context in both directions.

### Bipedal Locomotion
The act of walking on two legs, a key capability for humanoid robots operating in human environments.

### Bridge
In robotics, a component that connects different communication frameworks, such as connecting ROS 2 with external systems like Unity or Gazebo.

## C

### Cartesian Space
A coordinate system defining positions and orientations in 3D space, often used for specifying end-effector positions in robotics.

### Computer Vision
A field of artificial intelligence that enables computers to interpret and understand visual information from the world.

### Control Theory
The mathematical study of dynamical systems with inputs, and how their behavior is modified by feedback.

### Convolutional Neural Network (CNN)
A class of deep neural networks commonly used for analyzing visual imagery, characterized by convolutional layers that can identify spatial features.

### Cross-Modal Attention
A mechanism in multimodal AI systems that allows information from one modality (e.g., vision) to influence processing in another modality (e.g., language).

## D

### Deep Learning
A subset of machine learning that uses artificial neural networks with multiple layers to model and understand complex patterns.

### Deep Reinforcement Learning (DRL)
A combination of deep learning and reinforcement learning where agents learn optimal behaviors through interaction with an environment using deep neural networks.

### Degrees of Freedom (DOF)
The number of independent movements a mechanical system has, particularly relevant for robotic joints and manipulators.

### Diffeomorphic
A mathematical property describing smooth, invertible transformations, often used in robotics for motion planning.

### Digital Twin
A virtual representation of a physical system that serves as the real-time digital counterpart of a physical object or system.

### Domain Randomization
A technique in machine learning where the training environment is varied randomly to improve the robustness of learned policies when transferring to real-world scenarios.

## E

### Embodied AI
Artificial intelligence that is integrated into physical agents (robots) that interact with the real world, as opposed to purely software-based AI systems.

### End-Effector
The device at the end of a robotic arm designed to interact with the environment, such as a gripper or tool.

### Episodic Memory
In robotics and AI, the ability to store and recall specific experiences or sequences of events, important for learning and adaptation.

### Event Camera
A bio-inspired sensor that captures changes in brightness asynchronously, providing high temporal resolution for dynamic scenes.

## F

### Forward Kinematics
The process of determining the position and orientation of a robot's end-effector based on the joint angles of its manipulator.

### Force Control
A control strategy in robotics that directly controls the forces applied by the robot, often used in manipulation tasks requiring precise interaction with objects.

### Fused Multi-Scale Dense Feature Extraction
A computer vision technique that combines features from multiple scales to improve object recognition and scene understanding.

## G

### Gazebo
An open-source 3D robotics simulator that provides accurate physics simulation, high-quality graphics, and convenient programmatic interfaces.

### Generative Adversarial Network (GAN)
A class of machine learning frameworks where two neural networks contest with each other to generate new data instances.

### Geometric Reasoning
The ability to understand and manipulate spatial relationships and properties, crucial for robot navigation and manipulation.

### Global Path Planning
The process of computing a high-level path from a start to goal location considering the overall environment structure.

### GPU Acceleration
The use of graphics processing units to accelerate computation, particularly important for deep learning and real-time robotics applications.

## H

### Human-Robot Interaction (HRI)
The study of interactions between humans and robots, focusing on design, development, and evaluation of robotic systems for human use.

### Humanoid Robot
A robot with a body structure similar to that of a human, typically featuring a head, torso, two arms, and two legs.

### Hugging Face Transformers
A library providing state-of-the-art natural language processing models and tools for transfer learning.

## I

### Inverse Kinematics
The process of determining the joint angles required to position a robot's end-effector at a desired location and orientation.

### Isaac Sim
NVIDIA's robotics simulation platform that provides photorealistic rendering, accurate physics simulation, and AI training environments.

### Iterative Closest Point (ICP)
An algorithm used to minimize the difference between two point clouds, commonly used in 3D registration and SLAM.

## J

### Joint Space
The space defined by the joint angles of a robot, as opposed to Cartesian space which is defined by position and orientation.

### Jacobian Matrix
In robotics, a matrix that relates the joint velocities to the end-effector velocities, important for motion control and force analysis.

## K

### Kinematics
The study of motion without considering the forces that cause it, crucial for robot motion planning and control.

### Kalman Filter
A mathematical method that uses a series of measurements to estimate the state of a system, widely used in robotics for sensor fusion.

## L

### Language Model
A type of artificial intelligence model designed to understand and generate human language, such as GPT models.

### Latent Space
A lower-dimensional space that represents the essential features of high-dimensional data, often used in generative models.

### Learning from Demonstration (LfD)
A method where robots learn tasks by observing and imitating human demonstrations.

### LiDAR (Light Detection and Ranging)
A sensing technology that uses laser light to measure distances, commonly used in robotics for mapping and navigation.

### Long Short-Term Memory (LSTM)
A type of recurrent neural network architecture designed to learn from experience over long periods, useful for sequential data.

## M

### Manipulation
The ability of a robot to physically interact with objects in its environment, typically through grasping, moving, or repositioning.

### Multimodal AI
Artificial intelligence systems that can process and integrate information from multiple sensory modalities, such as vision, language, and audio.

### Machine Learning
A subset of artificial intelligence that enables systems to learn and improve from experience without being explicitly programmed.

### Model Predictive Control (MPC)
An advanced control method that uses a model of the system to predict future behavior and optimize control actions.

## N

### Navigation2 (Nav2)
The second-generation navigation system for ROS 2, providing a comprehensive framework for robot navigation.

### Natural Language Processing (NLP)
A field of artificial intelligence focused on enabling computers to understand, interpret, and generate human language.

### Neural Radiance Fields (NeRF)
A technique for synthesizing novel views of complex 3D scenes based on a sparse set of 2D images.

### Node (in ROS)
A process that performs computation in the Robot Operating System, which can publish or subscribe to topics and provide services.

## O

### OpenAI GPT
A series of large language models developed by OpenAI, capable of understanding and generating human-like text.

### OpenAI Whisper
An automatic speech recognition (ASR) system developed by OpenAI that converts speech to text.

### Operational Space Control
A control framework that allows direct control of a robot's end-effector in Cartesian space while maintaining stability in the null space.

### Ontology
A formal representation of knowledge as a set of concepts within a domain and the relationships between those concepts.

## P

### Path Planning
The computational problem of finding a valid path from a start to a goal position, considering obstacles and constraints.

### Point Cloud
A set of data points in space, typically representing the external surface of an object, used in 3D mapping and perception.

### Proximal Policy Optimization (PPO)
A policy gradient method in reinforcement learning that uses a clipped objective function to prevent large policy updates.

### Physics Simulation
The computational modeling of physical phenomena, essential for robotics simulation and training.

### Prismatic Joint
A joint that provides linear sliding movement between two links of a robot.

### Proxemics
The study of personal space and how humans use space in social interactions, relevant for social robotics.

## Q

### Quality of Service (QoS)
In ROS 2, a set of policies that define how messages are delivered between publishers and subscribers, including reliability and durability.

### Quaternion
A mathematical representation of rotations and orientations in 3D space, commonly used in robotics to avoid gimbal lock.

## R

### Reinforcement Learning (RL)
A type of machine learning where agents learn to make decisions by performing actions and receiving rewards or penalties.

### Robot Operating System (ROS)
A flexible framework for writing robot software that provides services designed for a heterogeneous computer cluster.

### ROS 2
The second generation of the Robot Operating System with improved security, real-time support, and better architecture.

### Riemannian Motion Policies (RMPs)
A mathematical framework for robot motion generation that combines geometric structure with reactive behaviors.

### ROS Bridge
A component that enables communication between ROS/ROS 2 and external systems, such as web browsers or simulation environments.

## S

### Simultaneous Localization and Mapping (SLAM)
The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location.

### Service
In ROS, a synchronous communication pattern where a client sends a request and waits for a response from a server.

### Socially-Aware Navigation
Navigation that considers the presence and behavior of humans, respecting social conventions and personal space.

### State Estimation
The process of estimating the internal state of a system from noisy and incomplete measurements.

### Sensor Fusion
The process of combining data from multiple sensors to improve the accuracy and reliability of perception.

### Speech-to-Text (STT)
The conversion of spoken language into written text, often using neural network models.

### Static Stability
A stability condition where a robot remains stable without active control, important for bipedal walking.

## T

### Topic
In ROS, an asynchronous communication mechanism where publishers send messages to subscribers without direct connection.

### Transformer Architecture
A deep learning architecture based on attention mechanisms, widely used in natural language processing and computer vision.

### Trajectory Planning
The process of determining the path and timing of motion for a robot to follow, considering dynamics and constraints.

### TF (Transforms)
In ROS, the package that keeps track of coordinate frames and their relationships over time.

### Trust Region Policy Optimization (TRPO)
A reinforcement learning algorithm that ensures stable policy updates by constraining the size of policy changes.

## U

### Unified Robot Description Format (URDF)
An XML format for representing robot models, including kinematic and dynamic properties.

### Unity
A cross-platform game engine that can be used for robotics simulation and visualization.

### Universal Approximation
The property of neural networks to approximate any continuous function given sufficient capacity.

## V

### Vision-Language-Action (VLA) Systems
Integrated systems that combine visual perception, language understanding, and action execution in robotics.

### Visual-Inertial Odometry (VIO)
The process of estimating motion by combining visual and inertial sensor data for robust localization.

### Visual Servoing
A control strategy that uses visual feedback to control the motion of a robot.

### VSLAM (Visual SLAM)
SLAM that uses visual sensors as the primary source of information for mapping and localization.

### Vector Database
A type of database that stores and retrieves data based on similarity in vector space, important for retrieval-augmented generation.

## W

### Whole-Body Control
A control approach that considers the entire robot body simultaneously, optimizing for multiple tasks and constraints.

### Whisper
OpenAI's automatic speech recognition system for converting speech to text.

### Waypoint Navigation
Navigation that moves a robot through a sequence of predefined positions or orientations.

## X

## Y

## Z

### Zero Moment Point (ZMP)
A criterion for static and dynamic stability of legged robots, representing the point where the sum of moments of the active forces is zero.

---

*This glossary will be updated as new terms are introduced throughout the book.*