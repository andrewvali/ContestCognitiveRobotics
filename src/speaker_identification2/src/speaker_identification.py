#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import time
import speech_recognition as sr
import pickle
import os 
from identification.deep_speaker.audio import get_mel
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id
from scipy.io import wavfile

SPEAKER_PATH=os.path.join(os.path.dirname(__file__),'deep_speaker.h5')
EMBED_PATH=os.path.join(os.path.dirname(__file__),'voice_identities')
RATE = 16000

class SpeakerReidentification():
    
    def __init__(self,topic_microphone,topic_result):
        rospy.loginfo("Subscribing to topic %s", topic_microphone)
        self.microphone_sub = rospy.Subscriber(topic_microphone, Int16MultiArray, self.callback)
        self.model = get_deep_speaker(SPEAKER_PATH)
        self.result_pub = rospy.Publisher(topic_result,String,queue_size=0)
    
    def callback(self,audio_data):
        # Conversion to float32
        
        ret = np.array(audio_data.data).astype(np.float32)
        ret /= 32768
        ret[ret > 1] = 1.0
        ret[ret < -1] = -1.

        samplerate = 16000 #costante
        OUT_PATH=os.path.join(os.path.dirname(__file__),'../test.wav')
        wavfile.write(OUT_PATH,samplerate,ret)

        #audio_data = audio_data.data.astype(np.float32, order='C') / 32768.0

        # Processing
        ukn = get_mel(ret, RATE)

        # Prediction
        ukn = self.model.predict(np.expand_dims(ukn, 0))

        # Load embeddings, labels and thresholds
        with open(EMBED_PATH+'/embed.pk', 'rb') as file:
            emb_dict = pickle.load(file)

        n_embs = len(emb_dict['y'])
        X = emb_dict['embeddings']
        y = emb_dict['y']
        ths = emb_dict['ths']

        # Distance between the sample and the support set
        emb_voice = np.repeat(ukn, n_embs, 0)
        cos_dist = batch_cosine_similarity(X, emb_voice)
        
        # Matching
        id_label = dist2id(cos_dist, y, ths, mode='avg')

        
        # Rejection
        if id_label is None:
            id_label = "?"
        
        self.result_pub.publish(id_label)
        
        
        

if __name__ == '__main__':
    rospy.init_node('speaker_reidentification', anonymous=True)
    sp = SpeakerReidentification('stream_audio_topic','result_topic')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    