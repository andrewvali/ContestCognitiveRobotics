import json
import os
import numpy as np
from datetime import datetime
from datetime import date as da

def get_json(path):
    with open(path) as json_file:
        data = json.load(json_file)

    return data

def print_json(path,json_data):
    with open(path,"w") as json_file:
        json.dump(json_data,json_file)

def get_cache_id(name):
    name = "_".join(name.lower().split(" "))

    test = datetime.today().strftime('%Y-%m-%d-%H:%M:%S')
    date = "".join(test.split("-"))
    date = "".join(date.split(":"))

    return name+"_"+date

def set_new_identity(new_name,ids=[],th=0.5,ids_folder=".", id_file_name='ids'):
    dict = {}

    id = {"cache_id":get_cache_id(new_name),"th":th,"name":new_name}
    ids.append(id)
    dict ["ids"] = ids

    print_json(ids_folder+"/"+id_file_name+".json",dict)

def get_identities(ids_folder=".", id_file_name='ids.json'):
    
    ids_file = os.path.join(ids_folder, id_file_name)
    json_file = get_json(ids_file)

    return json_file["ids"]

'''def get_identities(ids_folder, read_file, id_file_name='ids.json'):

    ids_file = os.path.join(ids_folder, id_file_name)

    json_file = get_json(ids_file)

    X = []
    y = []
    ths = []

    for id in json_file['ids']:
        for file in id['files']:
            file_path = os.path.join(ids_folder, id['folder'], file)
            X.append(read_file(file_path))
            y.append(id['name'])
            ths.append(id['th'])

    return X, y, ths'''
