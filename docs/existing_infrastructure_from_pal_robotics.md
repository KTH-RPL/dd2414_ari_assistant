# Existing Infrastructure from PAL

## Hardware

- **Sensors & Perception:**
-   Head Camera: RGB-D
-   Torso Front Camera: RGB-D
-   Torso Back Camera: Stereo-fisheye
-   LIDAR
- **Interaction & Display:**
-   Touchscreen
-   Eyes: 2 LCD screens with custom animations
-   Ears: 2x 16 RGB LEDs
-   Back: 40 RGB LED ring
- **Audio:**
-   Speakers: 2 x 30W Hi-Fi full-range
-   Microphones: 4x high-performance digital microphone array

## Software

- **ROS4HRI Framework:** Implements REP-155 for human representation using unique identifiers: face, body, voice, and person.
- **Face Detection and Recognition:** Detects and recognizes faces, extracting facial landmarks for identification.
- **Skeleton Tracking:** Supports both 2D and 3D skeleton tracking to interpret human poses and movements. Only for single body.
- **Speech Recognition:** Utilizes the Vosk recognizer for offline speech-to-text conversion.
- **Dialogue Management:** Integrates with the RASA chatbot engine to manage conversations.
- **Speech Synthesis:** Uses the Acapela engine for text-to-speech conversion, only for English.
- **Knowledge Base:** Supports RDF/OWL standards for symbolic reasoning and complex queries using SPARQL.
- **Gesture Recording and Playback:** Allows for the creation and execution of predefined gestures.
- `expressive_eyes` Node: Controls the robot's eye movements and expressions.
- `gaze_manager` Node: Coordinates head and eye movements for natural gaze behavior.
- **Web-Based User Interface:** Allows intuitive control and customization of ARI's features.
- **Simulation Tools:** Provides a ROS-based simulator for testing and development without the physical robot.
