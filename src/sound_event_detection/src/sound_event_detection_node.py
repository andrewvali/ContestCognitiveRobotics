#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String, Bool

from sound_event_detection.srv import SpeechRecognition, SpeechRecognitionResponse
import os

import numpy as np
from keras_yamnet import params
from keras_yamnet.yamnet import YAMNet, class_names, class_id
from keras_yamnet.preprocessing import preprocess_input
from std_msgs.msg import Int16MultiArray

###### CONSTANTS #####

RATE = params.SAMPLE_RATE
WIN_SIZE_SEC = 0.975
CHUNK = int(WIN_SIZE_SEC * RATE)

PATH_TO_MODEL = os.path.join(os.path.dirname(__file__),'keras_yamnet/yamnet.h5')
PATH_TO_YAMNET_CLASS = os.path.join(os.path.dirname(__file__),'keras_yamnet/yamnet_class_map.csv')
yamnet_classes = class_names(PATH_TO_YAMNET_CLASS)
yamnet_classes_id = class_id(PATH_TO_YAMNET_CLASS)

for i in range(0,len(yamnet_classes_id)):
    yamnet_classes_id[i] = int(yamnet_classes_id[i])

###### DEFINING CLASS ######

class SoundEventDetection():

    __slots__ = 'mic_sub', 'model', 'reidentification_pub'

    def __init__(self,topic_mic,topic_reidentification):
        self.mic_sub = rospy.Subscriber(topic_mic, Int16MultiArray, self.speech_recognition)
        self.model = YAMNet(weights=PATH_TO_MODEL)
        self.reidentification_pub=rospy.Publisher(topic_reidentification,Int16MultiArray,queue_size=10)        
    
    def speech_recognition(self, audio):
        print("Audio Received!")
        #audio_data = np.frombuffer(audio.get_raw_data(),dtype=np.int16)
        ret = np.array(audio.data).astype(np.float32)
        ret /= 32768
        ret[ret > 1] = 1.0
        ret[ret < -1] = -1.

        stream_in_chunk = int(len(ret)/len(ret[:CHUNK]))
        
        speech=0
        stream_augumented = np.array([])

        for i in range(0,stream_in_chunk):
            data = preprocess_input(np.fromstring(
                ret[i*CHUNK:(i*CHUNK)+CHUNK], dtype=np.float32), RATE)

            prediction = self.model.predict(np.expand_dims(data,0))[0]
            max_probability = max(np.expand_dims(prediction[yamnet_classes_id],-1))
            if max_probability == prediction[yamnet_classes_id[0]]:
                stream_augumented = np.append(stream_augumented,audio.data[i*CHUNK:(i*CHUNK)+CHUNK])
                speech = speech+1
        
        print(type(stream_augumented))
        data_to_send = Int16MultiArray()
        data_to_send.data = np.array(stream_augumented).astype(np.int16)

        if speech >= stream_in_chunk/2:
            print("Speech")
            print(type(data_to_send))
            
            self.reidentification_pub.publish(data_to_send)
        else:
            print("No Speech")
            
        

################ MAIN ################

if __name__ == '__main__':
    rospy.init_node("sound_event_detection")
    im = SoundEventDetection('mic_data','stream_audio_topic')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")