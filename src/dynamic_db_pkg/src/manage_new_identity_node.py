#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from dynamic_db_pkg.srv import ManageAudioIndentityError, ManageAudioIndentityErrorResponse
from redis_cli_ros.srv import *
from io import BytesIO
import json
import numpy as np
import time 
import sys, select
from datetime import datetime
from datetime import date as da
from identity_handle import * 
  
# Operation utils
def serialize_audio(original):

    memfile = BytesIO()
    np.save(memfile, original)
    memfile.seek(0)
    serialized = json.dumps(memfile.read().decode('latin-1'))

    return serialized

###### DEFINING CLASS ######

class NewIdentityHandler():

    __slots__ = 'server'

    def __init__(self,service):
        rospy.loginfo("Inizializing Server of service AddNewIdentity")
        self.server = rospy.Service(service, ManageAudioIndentityError, self.callback)
        
    def callback(self, req):
        """This callback manages the insertion of a new identity and the insertion of 
            an audio for a specific identity not recognized with certainty by the network 

        Args:
            req ([type]): request of ManageAudioIdentityError

        Returns:
            [req.response]: response of the service (bool and string)
        """
        if req.person.data == "?":
            print('Unrecognized person. Do you want add a new identity? Yes or Not')
            
            value = None
            print("You have five seconds to answer!")
            i, o, e = select.select( [sys.stdin], [], [], 5 )
            if (i):
                value = sys.stdin.readline().strip()
            else:
                print("You said nothing!")

            while(value is not None):
                if value.lower() == "yes":
                    print("\nYou have 20 seconds to answer!")
                    print("Insert full name: ")
                    
                    full_name = None
                    i, o, e = select.select( [sys.stdin], [], [], 20 )
                    if (i):
                        full_name = sys.stdin.readline().strip()
                    else:
                        print("You said nothing!")
                    
                    if full_name is None:
                        print("Person not added!")
                        return False, "Warning: Time out"

                    success,error,cacheId= create_new_identity(full_name)
                    if success:
                        cache_id = cacheId
                    else:
                        return False,error
                    
                    audio = serialize_audio(np.array(req.audio.data).astype(np.int16))
                    rospy.wait_for_service('store_data_append')
                    try:
                        audio_save = rospy.ServiceProxy('store_data_append', StoreData)
                        
                        resp = audio_save(cache_id,audio)
                    except rospy.ServiceException as e:
                        success = False
                        error = "ERROR: " + e
                        print("Service call failed: %s"%e)
                        return success,error
                    if resp.success:
                        print("Person added!")
                    else:
                        return False,resp.error
                    return True, ""
                   
                elif value.lower() == "no":
                    return False, "Person not added!"
                else:
                    print("Person not added!")
                    return False, "Warning: I don't undestand!"
            else:
                print("Person not added!")
                return False, "Warning: Time out"
        else:
            print('Do you want to add a new audio? Yes or Not')
            
            value = None
            print("You have five seconds to answer!")
            i, o, e = select.select( [sys.stdin], [], [], 5 )
            if (i):
                value = sys.stdin.readline().strip()
            else:
                print("You said nothing!")
            
            if value is not None:
                if value.lower() == "yes":
                    audio = serialize_audio(np.array(req.audio.data).astype(np.int16))
                    rospy.wait_for_service('store_data_append')
                    try:
                        audio_save = rospy.ServiceProxy('store_data_append', StoreData)
                        
                        resp = audio_save(req.person.data,audio)
                    except rospy.ServiceException as e:
                        success = False
                        error = "ERROR: " + e
                        print("Service call failed: %s"%e)
                        return success,error
                    if resp.success:
                        print("Audio added!")
                    else:
                        return False,resp.error
                    return True, ""

                elif value.lower() == "no":
                    return False, "Audio not added!"
                else:
                    return False, "Audio not added!"
            else:
                return False, "Warning: I don't undestand!"
        


################ MAIN ################

if __name__ == '__main__':
    rospy.init_node('dynamic_db_node')
    im = NewIdentityHandler('manage_audio_identity')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
