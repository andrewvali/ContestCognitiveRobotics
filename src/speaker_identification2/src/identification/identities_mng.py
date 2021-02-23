import json
import os
import numpy as np
from datetime import datetime
from datetime import date as da
import rospy
from redis_cli_ros.srv import *

def get_json(name_file):

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



def print_json(json_data):

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

            

def get_cache_id(name):
    name = "_".join(name.lower().split(" "))

    test = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
    date = "".join(test.split("-"))
    date = "".join(date.split(":"))

    return name+"_"+date

def set_new_identity(cache_id,new_name,ids=[],th=0.60):
    new_ids = {}

    identity = {"cache_id":cache_id,"th":th,"name":new_name}
    ids.append(identity)
    new_ids["ids"] = ids

    print_json(json.dumps(new_ids))

def get_identities():
    
 
    json_file = get_json("identities")

    print(json_file)

    return json_file["ids"]

def get_first_date(id):
    date = dict()
    temp = id.split("_")
    name = temp[0:-1]
    temp = temp[-1]
    date["year"] = temp[0:4]
    date["month"] = temp[4:6]
    date["day"] = temp[6:8]
    date["hour"] = temp[8:10]
    date["minute"] = temp[10:12]
    date["second"] = temp[12:14]
    date["date"] = date["day"]+"/"+date["month"]+"/"+date["year"]
    date["time"] = date["hour"] + ":" + date["minute"] + ":" + date["second"]
    date["datetime"] = date["date"] +"-"+ date["time"]
    d = da(year = int(date["year"]),month= int(date["month"]),day =int(date["day"]))
    return date,d," ".join(name)
