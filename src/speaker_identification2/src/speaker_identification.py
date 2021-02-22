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
from identification.utils import *
from scipy.io import wavfile
from dynamic_db_pkg.srv import *
from std_msgs.msg import String
from redis_cli_ros.srv import *

######### PATH OF THE MODEL #########
SPEAKER_PATH=os.path.join(os.path.dirname(__file__),'deep_speaker.h5')
######### PATH OF THE EMBEDDING FILE #########
EMBED_PATH=os.path.join(os.path.dirname(__file__),'voice_identities')
######### CONSTANT #########
RATE = 16000

class SpeakerReidentification():
    
    def __init__(self,stream_audio_topic,topic_result):
        rospy.loginfo("Subscribing to topic %s", stream_audio_topic)
        self.microphone_sub = rospy.Subscriber(stream_audio_topic, Int16MultiArray, self.callback)
        self.model = get_deep_speaker(SPEAKER_PATH)
        self.result_pub = rospy.Publisher(topic_result,String,queue_size=0)
        self.client = rospy.ServiceProxy('manage_audio_identity_error',ManageAudioIndentityError)
    
    def callback(self,audio_data):
        """Tis callback is called when a message on 'stream_audio_topic' is received. 
           It's perfrmed the speaker identification of the audio received and the result is 
           published on 'topic_result' topic.

        Args:
            audio_data ([type]): [description]
        """
        # Conversion to float32
        
        ret = np.array(audio_data.data).astype(np.float32)
        ret /= 32768
        ret[ret > 1] = 1.0
        ret[ret < -1] = -1.

        #samplerate = 16000 #costante
        #OUT_PATH=os.path.join(os.path.dirname(__file__),'../test.wav')
        #wavfile.write(OUT_PATH,samplerate,ret)

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
        id_label,max_score = dist2id(cos_dist, y, ths, mode='avg')

        self.result_pub.publish(id_label)
        
        # Rejection
        if id_label is None:
            print("Person not reognized!")
            id_label = "?"
            rospy.wait_for_service('manage_audio_identity_error')
            self.microphone_sub.unregister()
            try:
                resp = self.client.call(String(id_label),audio_data)
                if not resp.success:
                    print("Warning: Can not handle identity error.")
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            finally:
                self.microphone_sub = rospy.Subscriber('stream_audio_topic', Int16MultiArray, self.callback)
        else:
            print("Person recognized " + id_label)
            print("Score: " + str(max_score))
            if max_score >= 0.73:
                print("Score >= 0.73, this audio is to be stored in db.")
                audio = serialize_audio(np.array(audio_data.data).astype(np.int16))
                rospy.wait_for_service('store_data_append')
                try:
                    audio_save = rospy.ServiceProxy('store_data_append', StoreData)                        
                    resp = audio_save(id_label,audio)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                    


if __name__ == '__main__':
    rospy.init_node('speaker_reidentification', anonymous=True)
    sp = SpeakerReidentification('stream_audio_topic','result_topic')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    