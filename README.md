# ContestCognitiveRobotics
## Speaker Identification in real-time
This is the official repository of Cognitive Robotics exam of University of Salerno
# Group 8
* [Picariello Emilio](https://github.com/Emilio-Picariello)
* [Valitutto Andrea](https://github.com/andrewvali/)
# Requirements
* librosa
* python_speech_features
* scipy
* redis-server
* redis
* tensorflow
* numpy
* sounddevice
* pyaudio
* speechrecognition
## Problem Description 
Implementation of a [ROS](https://www.ros.org/) architecture that identifies the person who is speaking in real time through voice analysis.
In particular:
* [Node 1](https://github.com/andrewvali/ContestCognitiveRobotics/blob/main/src/voice_detection/src/voice_node.py): The application must capture the audio stream using the built-in microphone in your computer
* [Node 2](https://github.com/andrewvali/ContestCognitiveRobotics/blob/main/src/sound_event_detection/src/voice_activity_detection_node.py): The VAD module must be able to recognize the voice and distinguish it from other sound events (unlike Speech Recognition which works with energy thresholding)
* [Node 3](https://github.com/andrewvali/ContestCognitiveRobotics/blob/main/src/speaker_identification2/src/speaker_identification_node.py): The Speaker Re-Identification module must allow the recognition of the person who is speaking among those already entered in the database and must allow the display of the name relating to the terminal
* [Node 4](https://github.com/andrewvali/ContestCognitiveRobotics/blob/main/src/dynamic_db_pkg/src/manage_new_identity_node.py): When the system does not recognize the person who is speaking, it must allow him / her to be entered in the database of people known through a terminal.
# How to launch a Demo
## Clone the Repository and Workspace Setup
```bash
cd ~
git clone https://github.com/andrewvali/ContestCognitiveRobotics.git
cd ContestCognitiveRobotics
catkin build
source devel/setup.bash
```
## Run redis server
To install redis server run:<br>
```bash
sudo apt update
sudo apt install redis-server
```
To install python redis library run:<br>
```bash
pip install redis
```

To launch redis:<br>
```bash
redis-server
```

## Run redis client
To launch stand-alone redis cli interface:<br>
```bash
redis-cli
```

To launch redis-client, open another terminal and run this command:
```bash
roslaunch redis_cli_ros redis_cli_launch.launch host:=*host_service* port:=*service_port*
```
### Note
The default:
* host = localhost
* port = 6379
## Run others nodes
To launch:
* voice_node
* voice_activity_detection_node
* speaker_identification_node
* manage_new_identity_node<br>
Open another terminal and run this command:
```bash
roslaunch speaker_identification2 identificator_launch.launch
```
#### Group 8
