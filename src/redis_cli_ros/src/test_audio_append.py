#!/usr/bin/env python3
import rospy
import time
import redis
import pickle
import json
import os
from io import BytesIO
import numpy as np
from scipy.io import wavfile
from redis_cli_ros.srv import *

if __name__ == '__main__':
    rospy.init_node('test_audio_append_append', anonymous=True)
    
    
    
    WAV_PATH=os.path.join(os.path.dirname(__file__),'../record_00001.wav')
    
    samplerate, original = wavfile.read(WAV_PATH)
    print (original.dtype)
    print(str(type(original)))

    memfile = BytesIO()
    np.save(memfile, original)
    memfile.seek(0)
    serialized = json.dumps(memfile.read().decode('latin-1'))

    #print(serialized)
    print(str(type(serialized)))

    
    rospy.wait_for_service('store_data_append')
    try:
        data_append = rospy.ServiceProxy('store_data_append', StoreData)
        resp = data_append("Andrea_Valitutto_20210221160004",serialized )
        print(str(resp))
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    

    
    #rospy.spin()