#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from dynamic_db_pkg.srv import ManageAudioIndentityError, ManageAudioIndentityErrorResponse
from speaker_identification2.srv import NewIdentity
from redis_cli_ros.srv import *
from io import BytesIO
import json
import numpy as np

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
        print(req.person)

        if req.person.data == "?":
            print('Unrecognized person. Do you want add a new identity? Yes or Not')
            value = input()
            while(True):
                if value.lower() == "yes":
                    full_name = input("Insert full name: ")
                    rospy.wait_for_service('create_new_identity')
                    print(type(full_name))
                    try:
                        data_save = rospy.ServiceProxy('create_new_identity', NewIdentity)
                        resp = data_save(full_name)
                    except rospy.ServiceException as e:
                        success = False
                        error = "ERROR: " + e
                        print("Service call failed: %s"%e)
                        return success,error
                    
                    if resp.success:
                        cache_id = resp.cache_id
                    else:
                        return False,resp.error

                    audio = serialize_audio(np.array(req.audio.data).astype(np.int16))

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
                    print("Person not added!")
                    return True, ""
        else:
            return False, "Error: Not implemented yet!"
        


################ MAIN ################

if __name__ == '__main__':
    rospy.init_node('dynamic_db_node')
    im = NewIdentityHandler('manage_audio_identity_error')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
