import os
import rospy
from identification.utils import *
from identification.identities_mng import *
from speaker_identification2.srv import *

class StoreAudioService():

    __slots__ = 'server'
    def __init__(self):
        
       
        #NewIdentity
        self.server = rospy.Service('create_new_identity', NewIdentity, self.create_new_identity )

    def create_new_identity(self, request):
        #print("Adding '"+request.name+"'...")
        print("Adding ")
        
        try:
            cache_id = get_cache_id(request.name)
            set_new_identity(cache_id,request.name,get_identities())
            success = True
            error = ""
        except Exception as e:
            success = False
            error = e

        return success, error, cache_id
