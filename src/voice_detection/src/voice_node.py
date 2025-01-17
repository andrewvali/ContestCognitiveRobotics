#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np

import time
import speech_recognition as sr

###### CONSTANT #######
SAMPLE_RATE = 16000

class Microphone():
    __slots__ = "pub","r","mic"

    def __init__(self,topic_mic_data):
        """
            Initialization of microphone capture, in detail:
            - Initialization of publisher connection to a given topic
            - Begins listening 
        """
        print("Publisher to 'mic_data' topic...")
        self.pub = rospy.Publisher(topic_mic_data,Int16MultiArray,queue_size=0)
        self.listener()
    
    # this is called from the background thread
    def callback(self,recognizer, audio):
        """Called when audio is captured from microphone, it is converted into Int16MultiArray
            and published on given topic
        """
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        
        self.pub.publish(data_to_send)
        
    def listener(self):
        """
            It connects to primary microphone, it calibrates on ambient noise and
            it starts recording task        
        """
        # Initialize a Recognizer
        self.r = sr.Recognizer()
        
        # Audio source
        self.mic = sr.Microphone(sample_rate=SAMPLE_RATE)

        # Calibration within the environment
        # we only need to calibrate once, before we start listening
        print("Calibrating...")
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)  
        print("Calibration finished")

        # start listening in the background
        # `stop_listening` is now a function that, when called, stops background listening
        print("Recording...")
        self.r.listen_in_background(self.mic, self.callback)
        
###### MAIN #######

if __name__ == '__main__':
    time.sleep(3)
    rospy.init_node('voice_node', anonymous=True)
    microphone = Microphone("mic_data")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")