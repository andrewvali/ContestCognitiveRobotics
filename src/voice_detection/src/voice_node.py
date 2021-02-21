#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

import time
import speech_recognition as sr
from sound_event_detection.srv import SpeechRecognition

rospy.init_node('voice_node', anonymous=True)
pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
#client = rospy.ServiceProxy('sound_event_detection',SpeechRecognition)

# this is called from the background thread
def callback(recognizer, audio):
    print("ti ho sentito...")
    data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
    data_to_send = Int16MultiArray()
    data_to_send.data = data
    #success = client(data_to_send)
    #if success:
    pub.publish(data_to_send)

def listener():
    # Initialize a Recognizer
    r = sr.Recognizer()

    # Audio source
    m = sr.Microphone(sample_rate=16000)

    # Calibration within the environment
    # we only need to calibrate once, before we start listening
    print("Calibrating...")
    with m as source:
        r.adjust_for_ambient_noise(source)  
    print("Calibration finished")

    # start listening in the background
    # `stop_listening` is now a function that, when called, stops background listening
    print("Recording...")
    stop_listening = r.listen_in_background(m, callback)
    print("cIAO...")
    rospy.spin()

if __name__ == '__main__':
    listener()