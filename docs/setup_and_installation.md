# Setup & Installation

## How to setup the Network and Docker environment

Please refer to the ARI Guide by RPL.

## How to deploy packages to ARI

Inside the CMakeLists.txt:

```
 #For any python node we want to install into ARI we add it here
 
 catkin_install_python(PROGRAMS
   src/joy_conversion_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
 #For any other type of file we want to install we add it here
 
 install(FILES
  launch/joy_conversion.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
```

We run this from inside the container

```
rosrun pal_deploy deploy.py ari --package my_package_name
```

We now are able to SSH into ARI and run the script. If there are issues ,verify that the required files have been installed into ARI by checking in ARI `~/deployed_ws/lib/my_package_name` and in `~/deployed_ws/share/my_package_name`

## How to change launch parameters in startup

Inside the robot (ssh pal@ari) go to:

```
cd .pal/pal_startup/apps
```

Open or create a new .yaml with the name of the node you want to change the startup configuration of.

This is an example of how the document should look like:

```
roslaunch: "hri_fullbody hri_fullbody.launch rgb_camera:=/head_front_camera/color depth_camera:=/head_front_camera/depth use_depth:=true single_body:=false"
dependencies: []
```

Here we can change parameters and this will override the original launch file.

## How to install Ollama

For the LLM node is necessary to download ollama, you can do it drectly from the web page [https://ollama.com/](https://ollama.com/).

After installing ollama, using your cmd download **mistral** model.

```
ollama pull mistral:7b
```

To be able to communicate with ollama server, it is necessary to install the python client.

```
pip install ollama
```

To know more about ollama with python, review the documentation in [https://github.com/ollama/ollama-python](https://github.com/ollama/ollama-python). For this project, we used **Custom Client.**

```
from ollama import Client
client = Client(
  host='http://localhost:11434',
  headers={'x-some-header': 'some-value'}
)
response = client.chat(model='llama3.2', messages=[
  {
    'role': 'user',
    'content': 'Why is the sky blue?',
  },
])
```

## How to Translate

In order to increase speed, we are using a unofficial translator from Google, which works perfect for simple tasks and not complex text.

```
pip install googletrans==4.0.0-rc1
```

```
from googletrans import Translator

translator = Translator()
result = translator.translate("How are you?", src='en', dest='fr')

print(result.text)
```

**NOTE: Do not install** this translator and ollama python client in the same computer or container, as they use packages that conflict with each other. Do it in separate containers or use another translator if you need both programs in the same computer.

## How to run the whole demo

After following the setup of both the LLM and translate, in the package dd2414\_brain is the launch file ari\_startup.launch which will need to be run inside ARI (SSH to ARI) to run all the nodes. Behaviour tree (`behaviour_tree.py`) needs to be running locally **AFTER** the nodes have been ran.

The following nodes run individually in two different computers:

- Computer 1:
-   chatbox
-   ollama\_response
- Computer 2:
-   stt
-   translate\_conversation

To know how to use stt, review the section **Speech-to-Text** in [Usage & Tutorials](https://github.com/KTH-RPL/dd2414_ari_assistant/blob/development/docs/usage_and_tutorials.md)

### Checklist before executing

- ollama\_response and chatbox running
- speech-to-text and translate running, launch file on ARI running
- behaviour\_tree running
- make sure rviz is running
- check camera’s and expressive\_eyes are running
- make sure battery above 60%
