#!/usr/bin/env python3
import rospy
import time
import redis
from redis_cli_ros.srv import *
import sys

class CliRedisService():

    def __init__(self):
        try:
            print(sys.argv[1])
            print(sys.argv[2])
            self.r = redis.Redis(host=sys.argv[1], port=sys.argv[2], db=0)
        except Exception as ex:
            print("ERROR: "+ str(ex))
            exit(1)
        print("CONNECTED TO REDIS")
        
        rospy.Service('store_data_append', StoreData, self.append_list )
        rospy.Service('retrieve_list', RetrieveData, self.retrieve_list)
        rospy.Service('store_data', StoreData, self.set_data)
        rospy.Service('retrieve_data', RetrieveData, self.retrieve_data)

    def append_list(self, request):
        try:
            topic= str(request.topic)
            data = str(request.data)
            print("received topic:'"+topic+"'")
            
            self.r.lpush(topic, data)
            success = True
            error = ""
        except Exception as ex:
            success = False
            error = str(ex)
        finally:
            print("success: "+str(success)+" , error: "+ error)
            return success, error
            
    def set_data(self, request):
        try:
            topic= str(request.topic)
            data = str(request.data)
            print("received topic:'"+topic+"'")
            
            self.r.set(topic, data)
            success = True
            error = ""
        except Exception as ex:
            success = False
            error = str(ex)
        finally:
            print("success: "+str(success)+" , error: "+ error)
            return success, error
            
    def retrieve_list(self, request):
        response = ""
        try:
            topic= str(request.topic)
            print("received topic:'"+topic+"'")
            
            response = self.r.lrange(topic, 0, 1000000000000)
            
            output = []
            
            for r in response:
            
                output.append(r.decode())     
            
            print(output)
                
            success = True
            error = ""
        except Exception as ex:
            success = False
           
            error = str(ex)
        finally:
            print("success: "+str(success)+" , error: "+ error)
            return success, error,output

    def retrieve_data(self, request):
        response = ""
        try:
            topic= str(request.topic)
            print("received topic:'"+topic+"'")
            
            response = self.r.get(topic)
            
            output = []
    
            output.append(response.decode())     
            
            print(output)
                
            success = True
            error = ""
        except Exception as ex:
            success = False
           
            error = str(ex)
        finally:
            print("success: "+str(success)+" , error: "+ error)
            return success, error,output


if __name__ == '__main__':
    rospy.init_node('redis_cli_node', anonymous=True)
    minimal_service = CliRedisService()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down and closing server")
        minimal_service.r.shutdown()
    