# Usage & Tutorials

Below you will find an explanation of each package and the nodes that you can find as well as tutorials on how to use them.

# Follow user

**File**: `dd2414_follow_user/follow_user.py`

**Run**: inside ARI (launch file)

This module enables ARI to detect and follow a nearby person using body tracking and navigation. It integrates ARI’s onboard sensors with the `pyhri` human-robot interaction library and `move_base` for autonomous motion, allowing the robot to maintain a respectful distance (1 meter) to the user while keeping them in view. This will only be activated by request of the user by voice command, and will be deactivated by the command “Stop”. ARI will only approach the user if the user steps out of a predifined distance, while the user is within this distance, ARI will just follow with her head. If the user were to go somewhere out of sight to ARI, it will turn with its whole body towards the direction it saw the user.

# Human Detection

## Multibody

**File**: `dd2414_human_detection/multiple_bodies_detected.py`

**Run**: locally (needs YOLO)

This section explains how to configure the multibody detection system for ARI using the `/hri_fullbody` library. This package enables skeleton estimation from camera input but is limited to single-body detection when used alone in ROS1 Noetic. To enable multibody detection, an external application must be introduced to identify and separate multiple individuals in the frame.

### System Design

- **Core Library:** `/hri_fullbody` (ROS1 Noetic)
- **Limitation:** By default, only supports single-body tracking.
- **Solution:** Integrate a separate detection pipeline that:
1.   Detects multiple individuals
2.   Generates bounding boxes
3.   Crops images per individual
4.   Assigns unique IDs
5.   Publishes preprocessed data to `/hri_fullbody`

### Human Detection Integration

- **Detection Model:** YOLOv8n (recommended for speed)
-   *Note:* Other YOLO versions can be used, but the nano version (YOLOvXn) is preferred due to its faster inference, which improves real-time tracking performance.
- **Detection Node:** `human_detection_node`
-   **Input:** Raw camera images from the topic provided by PAL Robotics.
-   **Output Topics:**
  
  -   `/humans/bodies/tracked`: Publishes a list of currently detected bodies.
  
  -   `/humans/bodies/<body_id>/cropped`: Cropped images per individual, used for 3D pose estimation.
  
  -   `/humans/bodies/<body_id>/roi`: Region of interest metadata for each detected person.

### Configuration Changes

- **Launch File Parameter:**
-   In the `hri_fullbody` launch file (Set launch files tutorial in Setup & Installation), set:
  ```
  roslaunch: "hri_fullbody hri_fullbody.launch rgb_camera:=/head_front_camera/color depth_camera:=/head_front_camera/depth single_body:=false"
  dependencies: []
  ```

This change switches from single-body mode to multi-body processing.

### Synchronization Requirement

To ensure that the `hri_fullbody` node processes the incoming topics correctly, each message must include a properly formatted **ROS header** with an accurate **current timestamp** (i.e., the time the message is published).

- Use `rospy.Time.now()` to assign the timestamp.
- This is critical for message synchronization within the `hri_fullbody` node, which relies on internal synchronization logic to align tracked body data with image crops and ROIs.

If these timestamps are missing, outdated, or inconsistent, the topics will be **ignored** by the node and multibody tracking will fail.

## Face Recognition

**File**: `dd2414_human_detection/face_recognition_node.py`

**Run**: inside ARI (launch file)

The face recognition module enables ARI to detect and identify individuals by comparing facial encodings. It uses a Python-based recognition system to store and match faces in real time.

### How It Works

- A script called `face_recognition_node` is used to detect and recognize faces.
- When a face is detected, its encoding is generated and compared against a database of previously known faces stored in a **JSON file**.
- If a match is found, the face is identified.
- If no match is found, the system creates a new entry with a unique ID and stores the new encoding.
- Additionally, if **name and location information** are available at the time of detection, the system can associate these details with the corresponding face entry.
- For location, it will save both the coordinates in ARI’s map and the room the person is in.
- If the name is later received (if the person presents itself), it will be saved accordingly.
- If someone greets ARI, the brain will activate this node to search the person’s name (the person ARI is currently detecting).

### Dependencies

This module uses the `face_recognition` Python library.

- To install:
```
pip install face_recognition
```

Ensure this library is installed in the correct Python environment used by the ROS node. It is already installed on ARI.

# Joystick

**File**: `dd2414_joystick/launch/joy_conversion.launch`

**Run**: inside ARI (launch file)

With the Logitech controller in Room 349 and its paired USB receiver () plugged in the left-most USB port on its backside, when running the launch file it allows the user to drive ARI manually with the right stick.

# Navigation

**File**: `dd2414_navigation/move_to_poi.py`

**Run**: inside ARI (launch file)

In this package you will find a node called `move_to_poi.py` where the navigation from PAL Robotics is used to navigate to certain Points of Interest previously defined in the map. It will also access the list of faces saved in the document created by `face_recognition_node.py` in case the brain has sent not a name of a room but a name of a person we have to find. In this case, the node will check all the people saved for a match and get the Zone Of Interest from the JSON file and go towards it if it exists.

(Maybe photo of the map as an example of the ZOIS created?)

# Parameter Server

**File**: `dd2414_parameter_server/parameter_server.py`

**Run**: inside ARI (launch file)

This is the node that makes sure the inflation radius for obstacles is lowered when going through a “door zone” (this zones have been previously defined as “door\_XX” in the map). The inflation radius will be returned to it’s original value after the door has passed. This allows ARI to center herself at the door, go through straight without troubles and continue a safe navigation later.

# Speech

## Find speaker

**File**: `dd2414_speech/find_speaker.launch`

**Run**: inside ARI (launch file)

This ROS launch file is composed of two nodes, the first one's (`dd2414_speech/find_speaker.py`) primary goal is to allow a robot to locate and orient itself toward a person who is speaking. As for the second one (`dd2414_human_detection/detect_people_looking_at_ari.py`), it simply publishes the topic `person_looking_at_robot` when a person is detected to have it’s body orientated towards ARI.

Below is the detailed workflow of `find_speaker.py`:

### 1\. **Sound Detection**:

The robot begins by detecting when someone starts speaking. It subscribes to a topic that indicates whether speech has started or stopped. When speech is detected, the robot starts recording the directions from which the sound is coming. This information is provided through a topic that gives the sound source's direction, and it is handled by the `direction_cb` method. During the speech, the robot stores the directions for later processing.

### 2\. **Rotating Toward the Sound**:

Once the robot has recorded the sound directions, the next step is to calculate the average direction of all the captured directions. If enough directions are stored, the robot computes this average direction. Then, it uses the `rotate_to` function to rotate toward that direction. This process helps the robot get closer to the general area where the speaking person is located. The direction returned is not very reliable due to the position of the microphones and the echo that can happen in close spaces, this is why ARI is programmed to search rotating in that direction and not to turn uniquely to that position.

### 3\. **Person Detection**:

Once the robot has oriented itself toward the general direction of the sound, the next step is to check if the person who is speaking is looking toward the robot. This is detected through the `person_found_cb` method, which listens for updates on the person looking at the robot. If it is detected that the person is looking at the robot, the search for the sound source is stopped, and the robot stops turning. The robot calculates the person's position in space and sends a new movement goal to approach them.

This workflow enables the robot not only to locate the sound source but also to adjust its orientation and approach the person who is speaking, facilitating human-robot interaction in a dynamic environment.

## Look at Person

**File**: `dd2414_speech/look_at_person.launch`

**Run**: inside ARI (launch file)

This ROS node enables a robot to detect human faces and direct its gaze toward them using the `pyhri` Human-Robot Interaction library. It listens for detected faces, computes their position, and publishes gaze targets so the robot can maintain eye contact. If no face is detected for a while, the robot returns to looking forward.

# Speech to Text

## Speech-to-Text (STT)

**File**: `dd2414_stt/stt.py`

**Run**: locally

While ARI provides built-in Text-to-Speech (TTS) and Speech-to-Text (STT) language packages, these do not include other languages, which requires a paid license. For Speech-to-Text functionality, this project uses **Whisper**, an open-source speech recognition system developed by OpenAI. It provides high-quality transcription across multiple languages and accents.

Unlike other STT solutions, Whisper runs locally and does not require an internet connection, making it suitable for secure or offline environments.

### Input Source

The transcription system subscribes to the following ROS topic: `\audio_speech` . This topic provides an array of bytes containing the raw audio data captured by ARI’s microphones.

### Whisper Integration

You must first install the Whisper library and its dependencies on the local machine:

```
pip install -U openai-whisper

#Requires ffmpeg
# on Ubuntu or Debian
sudo apt update && sudo apt install ffmpeg
# on Arch Linux
sudo pacman -S ffmpeg

pip install setuptools-rust
```

### Example: Generate STT in a Python Script

```
import wave
import json
import rospy
import whisper

from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

class STT:
    def __init__(self):
        self.stt_model = whisper.load_model("small")
        self.msg_data  = None
        
        # Audio parameters – adjust according to your setup
        self.stt_filename = f"/tmp/ARI_stt.wav"
        self.SAMPLE_RATE  = 16000  # Hz
        self.SAMPLE_WIDTH = 2      # 2 bytes = 16-bit audio
        self.CHANNELS     = 1      # Mono
        
        rospy.Subscriber("/audio/speech", AudioData, self.stt_audio, queue_size=1)

    def stt_audio(self,msg):
        if not msg.data:
           self.msg_data = None
           return    
        
        self.msg_data = msg.data
        process_audio(self.msg_data)

    def process_audio(self,msg_data):
        transcript = {}
        with wave.open(self.stt_filename, 'w') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.SAMPLE_WIDTH)
            wf.setframerate(self.SAMPLE_RATE)
            wf.writeframes(msg_data)  # msg.data is a bytes object
            wf.close
    
        result = self.stt_model.transcribe(self.stt_filename)
```

## Speak

**File**: `dd2414_ollama_response/ollama_response.py`

**Run**: locally

The project uses an open-source alternative: **Edge Text-to-Speech (edge\_tts)**. When the language the speaker is using is recognized as English by the STT, it will use ARI’s incorporated TTS; but when the language is another (in this project we have enabled Swedish, German, Spanish, Japanese and French), then it will use Edge’s TTS.

This node makes requests to ollama server to generate personalized responses to the user in different languages, generates the corresponding audio file and send it to ARI in the /tmp folder. This audio file will then be played by the /text\_multilanguage\_speech node directly in ARI.

# Status Update

# Talking Orientation

**File**: `dd2414_talking_orientation/body_orientation_listener.py`

**Run**: inside ARI (launch file)

This node enables ARI to recognize a person's orientation and position itself to their left or right (choosing the side that is unoccupied) while aligning its gaze in the same direction the person is looking. This behavior is particularly useful in scenarios such as acting as a translator, where maintaining a shared line of sight is important.

# Text to Speech (TTS)

**File**: `dd2414_text_speech/text_multilanguage_speech.py`

**Run**: locally

This node receives the detected language, as well as the phrase or location of the audio file to be played. This provides the ability to respond in different languages.

### Installing Edge TTS (edge\_tts)

To use Edge TTS in your Python scripts, install it via pip:

```
pip install edge_tts
```

This is **NOT** installed in ARI since **ARI does not have permit internet access (missing proxys or network configuration),** which doesn’t allow TTS node from functioning as intended when deployed. To address this, the TTS is handled on a **separate local computer** connected to the same internal network. The audio is generated locally using `edge_tts`, saved as a file, and then transferred to ARI for playback.

### Example: Generate TTS in a Python Script

Here’s how to generate speech from a longer Swedish text:

```
import os
import edge_tts

# Longer Swedish sentence
text = "Sverige är ett vackert land med många intressanta platser att besöka, från de majestätiska fjällen i norr till de pittoreska städerna vid kusten."

# Generate the TTS audio
tts = edge_tts.Communicate(text,voice="sv-SE-SofieNeural" lang='sv')

# Save the audio to your catkin workspace
save_path = os.path.expanduser('~/catkin_ws/src/long_swedish_audio.mp3')
print("Saving long Swedish audio file!")

tts.save_sync(save_path)
```

### Playing audios in ARI

To play an audio in ARI there are of course several ways, but the following one is the one we found to work best. Once the audio file is created, you can play it back on ARI using the `simpleaudio` library. Increase the volume by 6db as audio files have lower volume than the local TTS.

#### Install the library:

```
pip install simpleaudio
```

#### Playback Example:

```
import simpleaudio as sa

audio = AudioSegment.from_wav("path/to/file.wav")
louder_audio = audio + 6
louder_audio.export("path/to/file.wav", format="wav")
wave_obj = sa.WaveObject.from_wave_file("path/to/file.wav")
play_obj = wave_obj.play()
play_obj.wait_done()
```

# Translate Conversation

**File**: `dd2414_translate_conversation/translate_conversation.py`

**Run**: locally

This node allows for bidirectional translation (perfect for a conversation). The user indicates the target language they want to translate into, and the language in which the robot received the instruction is used as the source language. This node sends the phrases and the corresponding language to ollama\_response to generate the corresponding audio files.

# Expressive Eyes Divisor
This package publishes the image to the eyes so they gain movement.

# Chatbox (LLM)

**Run**: locally

The **LLM** node listens to stt node to obtain what the user said. Once the command is recognized and finalized, the node processes the speech to extract the intent behind the user's words. It uses a the**Large Language Model (LLM) Mistral** via **Ollama server,** which runs locally in the computer. The LLM analyze the text and categorize it into predefined intents, such as "greet," "remember user," "find object," "go to," “follow user“, “explore”, or "provide information."

After the intent is identified, the **LLM** node determines the appropriate response or action and send it to the brain, according to the hierarchy, the behaviour tree will decide to extecute or not the action. If the intent requires a verbal response, the brain passes the result to **Ollama\_response** node which also calls **Text-to-Speech (TTS)** system, enabling ARI to speak directly to the user. If the intent involves actions like moving the robot or interacting with other systems, the LLM node prepares the necessary data and sends it to the brain.

The brain, as the central node, receives these intents and makes decisions about how to respond.
