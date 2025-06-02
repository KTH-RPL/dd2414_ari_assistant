# RPL ARI Resources
## Introduction
This project focuses on developing an intelligent behavior system for **ARI**, a humanoid service robot developed by PAL Robotics. ARI is equipped with a variaty of sensors—including RGB-D cameras, microphones, and a mobile base—making it ideal for social robotics and human-robot interaction (HRI) tasks in indoor environments.

### Goal

The primary goal of the project is to turn ARI into a **smart office assistant** capable of understanding and responding to human commands in several languages, navigating autonomously, and interacting naturally with people. This includes tasks such as recognizing individuals, following them, speaking, and reacting to voice commands—all within a behavior-based framework.

We envision ARI being used in an office environment to:

- Greet visitors,
- Escort people to locations,
- Find other people based on last seen location or their office,
- Answer spoken questions,
- Assist with simple tasks via natural interaction.

### How It Works

We implemented a **behavior tree system** using ROS and Python to control ARI’s high-level behavior. The system coordinates a set of modular, reusable ROS nodes that each handle specific capabilities such as:

- **Face recognition**
- **Speech-to-text (STT)** for command recognition in multiple languages
- **Text-to-speech (TTS)** for robot responses in multiple languages
- **Person following**
- **Navigation to Points of Interest**
- **Stop behavior**
- **Orientation control** to ensure that ARI faces the speaker.
- **Translate conversation** act as translator between two persons

This project uses an **LLM (Large Language Model)** to interpret voice commands. Instead of requiring specific trigger phrases, the user can speak to **ARI** in conversational language. The LLM interprets the intent behind the spoken request and determines which functional module to activate—such as navigation, or person following—based on context. This approach significantly enhances the robot's usability in real-world environments like an office, where commands may vary in phrasing and complexity.

## Documentation
- [Setup & Installation](docs/setup_and_installation.md)
- [Brain Interfaces and Behaviour Tree](docs/brain_interfaces_and_behaviour_tree.md)
- [Usage & Tutorials](docs/usage_and_tutorials.md)
- [Testing & Troubleshooting](docs/testing_and_troubleshooting.md)
- [Results](docs/results.md)
- [Existing Infrastructure from PAL Robotics](docs/existing_infrastructure_from_pal_robotics.md)
