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
from identityHandle import * 
  
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

    def create_new_identity(self, request):
        #print("Adding '"+request.name+"'...")
        print("Adding ")
        
        try:
            cache_id = self.get_cache_id(request)
            self.set_new_identity(cache_id,request,self.get_identities())
            success = True
            error = ""
        except Exception as e:
            success = False
            error = e

        return success, error, cache_id
    
    '''
    def get_cache_id(self,name):
        name = "_".join(name.lower().split(" "))

        test = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
        date = "".join(test.split("-"))
        date = "".join(date.split(":"))

        return name+"_"+date
    
    def set_new_identity(self, cache_id,new_name,ids=[],th=0.60):
        new_ids = {}

        identity = {"cache_id":cache_id,"th":th,"name":new_name}
        ids.append(identity)
        new_ids["ids"] = ids

        self.print_json(json.dumps(new_ids))
    
    def print_json(self,json_data):
        
        rospy.wait_for_service('store_data')
        try:
            data_append = rospy.ServiceProxy('store_data', StoreData)
            resp = data_append("identities",json_data )

        except rospy.ServiceException as e:
            error = "Service json update failed: "+e
            print(error)
            raise Exception(error)
        if not resp.success:
            error = "Service json update failed: "+resp.error
            print(error)
            raise Exception(error)

    def get_identities(self):
        json_file = self.get_json("identities")

        print(json_file)

        return json_file["ids"]

    def get_json(self,name_file):
        
        rospy.wait_for_service('retrieve_data')
        try:
            response = rospy.ServiceProxy('retrieve_data', RetrieveData)
            data = response(name_file)
        except rospy.ServiceException as e:
            error = "Impossible json identities retrieval ERROR: Service call failed: "+e
            print(error)
            raise Exception(error)
            
        
        
        if not data.success:
            error = "WARNING: Impossible json identities retrieval ERROR: "+ data.error
            print(error)

    
        try:
            json_file = json.loads(data.output[0])
        except Exception as ex:
            error = "WARNING: no json configuration has been found "+ str(ex)
            print(error)
            json_file = dict()
            json_file["ids"] = []
        return json_file
'''
    def callback(self, req):
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
                    full_name = input("Insert full name: ")
                    '''
                    rospy.wait_for_service('create_new_identity')
                 
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
                    '''
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
                return False, "Warning: I don't undestand!"
        


################ MAIN ################

if __name__ == '__main__':
    rospy.init_node('dynamic_db_node')
    im = NewIdentityHandler('manage_audio_identity')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
