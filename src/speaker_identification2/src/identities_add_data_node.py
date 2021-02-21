#!/usr/bin/env python3
import os
import rospy
from identification.utils import *
from identification.identities_mng import set_new_identity,get_identities
from speaker_identification2.srv import *

class CliRedisService():

    def __init__(self):
        
       
        #NewIdentity
        rospy.Service('create_new_identity', NewIdentity, self.create_new_identity )

    def create_new_identity(self, request):
        print("Adding '"+request.name+"'...")
        
        try:
            set_new_identity(request.name,get_identities())
            success = True
            error = ""
        except Exception as e:
            success = False
            error = e

        return success, error
            
        

    


if __name__ == '__main__':
    rospy.init_node('identities_add_data_node', anonymous=True)



    minimal_service = CliRedisService()

    rospy.spin()