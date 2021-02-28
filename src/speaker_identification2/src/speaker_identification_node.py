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
from identification.identities_mng import *
from prep_voice_embeddings import CreateEmbedding

######### PATH OF THE MODEL #########
SPEAKER_PATH=os.path.join(os.path.dirname(__file__),'deep_speaker.h5')
######### PATH OF THE EMBEDDING FILE #########
EMBED_PATH=os.path.join(os.path.dirname(__file__),'voice_identities')
######### CONSTANT #########
RATE = 16000
######### AUDIO EMBEDDING CREATOR SERVICE #########
EMBEDDING_CREATOR = CreateEmbedding()

class SpeakerReidentification():
    
    def __init__(self,stream_audio_topic,topic_result):
        """
            Creates a new embedding of already saved data, 
            it subscribes to a given topic to retrieve data, it initiliazes model with 
            its weights, it conncets as a publisher to an output topic, it initializes 
            embedding creation service.

        """
        EMBEDDING_CREATOR.create_new_embedding()
        rospy.loginfo("Subscribing to topic %s", stream_audio_topic)
        self.microphone_sub = rospy.Subscriber(stream_audio_topic, Int16MultiArray, self.callback)
        self.model = get_deep_speaker(SPEAKER_PATH)
        rospy.loginfo("Publisher to topic %s", topic_result)
        self.result_pub = rospy.Publisher(topic_result,String,queue_size=0)
        rospy.loginfo("Service client to service manage_audio_identity")
        self.client = rospy.ServiceProxy('manage_audio_identity',ManageAudioIndentityError)

    
    def callback(self,audio_data):
        """This callback is called when a message on a given topic is received.  
           It's performed the speaker identification of the audio received and the result is 
           published on 'topic_result' topic. It decides if a new identity and/or audio sample
           may be saved, using a support service

        Args:
            audio_data (std_msgs/Int16MultiArray): audio received from sound event detection
        """
        # Conversion to float32
        
        ret = np.array(audio_data.data).astype(np.float32)
        ret /= 32768
        ret[ret > 1] = 1.0
        ret[ret < -1] = -1.

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
        id_label,max_score,th_max = dist2id(cos_dist, y, ths, mode='avg')

        self.result_pub.publish(id_label)
        
        
        # Rejection
        if id_label is None:
            print("Person not recognized!")
            id_label = "?"
            # Support service is called to decide if new identity has to be added
            rospy.wait_for_service('manage_audio_identity')
            self.microphone_sub.unregister()
            try:
                resp = self.client.call(String(id_label),audio_data)
                if not resp.success:
                    print(resp.error)
                else:
                    # If a new identity has been added, a new embedding is created
                    EMBEDDING_CREATOR.create_new_embedding()
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            finally:
                self.microphone_sub = rospy.Subscriber('voice_data', Int16MultiArray, self.callback)

        else:
            # Print the results
            date,_,name = get_first_date(id_label)
            print("Person recognized {}. We first met time {} at {}".format(name.upper(),date["date"],date["time"]))
            print("Score: {}\n".format(max_score))
            
            # If score is not very reliable, support service is called to decide if such new audio sample has to be added
            if max_score < th_max+0.10:
                rospy.wait_for_service('manage_audio_identity')
                self.microphone_sub.unregister()
                try:
                    resp = self.client.call(String(id_label),audio_data)
                    if not resp.success:
                        print(resp.error)
                    else:
                        print("Score < 0.65, this audio is now stored in db.\n")
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
                finally:
                    self.microphone_sub = rospy.Subscriber('voice_data', Int16MultiArray, self.callback)
     

##############  MAIN ############## 
if __name__ == '__main__':
    rospy.init_node('speaker_reidentification', anonymous=True)
    sp = SpeakerReidentification('voice_data','result_topic')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    