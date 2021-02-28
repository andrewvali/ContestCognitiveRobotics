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
import tensorflow as tf

######### CONSTANTS #########

RATE = params.SAMPLE_RATE
WIN_SIZE_SEC = 0.975
CHUNK = int(WIN_SIZE_SEC * RATE)

PATH_TO_MODEL = os.path.join(os.path.dirname(__file__),'keras_yamnet/yamnet.h5')
PATH_TO_YAMNET_CLASS = os.path.join(os.path.dirname(__file__),'keras_yamnet/yamnet_class_map.csv')

######### Yamnet classes #########
yamnet_classes = class_names(PATH_TO_YAMNET_CLASS)
yamnet_classes_id = class_id(PATH_TO_YAMNET_CLASS)

######### Convert string id classes to int ######### 
for i in range(0,len(yamnet_classes_id)):
    yamnet_classes_id[i] = int(yamnet_classes_id[i])

###### DEFINING CLASS ######

class SoundEventDetection():

    __slots__ = 'mic_sub', 'model', 'reidentification_pub'

    def __init__(self,topic_mic,topic_reidentification):
        """It subsribes to a given topic to retrieve audio, it initializes YAMNet model with
            its weights, it conncets as a publisher of reidentification topic         
        """
        print("Subscribing to %s topic",topic_mic)
        self.mic_sub = rospy.Subscriber(topic_mic, Int16MultiArray, self.speech_recognition)
        self.model = YAMNet(weights=PATH_TO_MODEL)
        print("Publisher to %s topic",topic_reidentification)
        self.reidentification_pub=rospy.Publisher(topic_reidentification,Int16MultiArray,queue_size=10)        
    
    def speech_recognition(self, audio):
        """This function performs sound event detection on the received audio and it decides
            if it is a human activity or not. 
           It converts the audio into float32 and split the file audio in CHUNKS to predict the 
           event of the audio. 
           Then a new audio is created containing only the speech of the original audio

        Args:
            audio ([std_msgs/Int16MultiArray]): File audio received from voice node
        """
        ret = np.array(audio.data).astype(np.float32)
        ret /= 32768
        ret[ret > 1] = 1.0
        ret[ret < -1] = -1.

        # Split audio in chunks
        if len(ret)>=CHUNK:
            stream_in_chunk = int(len(ret)/len(ret[:CHUNK]))
                
            stream_augumented = np.array([])
            if stream_in_chunk!=0:
                for i in range(0,stream_in_chunk):
                    
                    data = preprocess_input(np.fromstring(
                        ret[i*CHUNK:(i*CHUNK)+CHUNK], dtype=np.float32), RATE)
                    prediction = self.model.predict(np.expand_dims(data,0))[0]
                    max_probability = max(np.expand_dims(prediction[yamnet_classes_id],-1))
                    for i in range(0,31):
                        if max_probability == prediction[yamnet_classes_id[i]]:
                            stream_augumented = np.append(stream_augumented,audio.data[i*CHUNK:(i*CHUNK)+CHUNK])
                            break
                    
            # stream_augumented is audio with only human speeching
            if len(stream_augumented) !=0:
                print("Recognized human speech or vocal activity")
                data_to_send = Int16MultiArray()
                data_to_send.data = np.array(stream_augumented).astype(np.int16)
            
                self.reidentification_pub.publish(data_to_send)
                
        

################ MAIN ################

if __name__ == '__main__':
    rospy.init_node("sound_event_detection")
    sound_event_detection = SoundEventDetection('mic_data','voice_data')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
