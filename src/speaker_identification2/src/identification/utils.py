import numpy as np
import librosa
from scipy.io import wavfile
from io import BytesIO
import json
from redis_cli_ros.srv import *
import rospy

# io utils

class ReadAudio:
    def __init__(self, sr=None, mono=True, res_type='kaiser_best'):
        '''
            Read img
            sr: float. None for native sampling rate
            mono : bool. Convert signal to mono
            res_type: str. resampy resampling mode.
        '''
        self.sr = sr
        self.mono = mono
        self.res_type = res_type

    def __call__(self, audio_path):
        '''
            Read img
            audio_path: string. Path to the file
        '''
        return librosa.load(audio_path, sr=self.sr, mono=self.mono, res_type=self.res_type)

# Operation utils
def serialize_audio(original):

    memfile = BytesIO()
    np.save(memfile, original)
    memfile.seek(0)
    serialized = json.dumps(memfile.read().decode('latin-1'))

    return serialized

def deserialize_audio(serialized):

    memfile = BytesIO()
    memfile.write(json.loads(serialized).encode('latin-1'))
    memfile.seek(0)
    audio = np.load(memfile)

    return audio

def retrieve_audio(cache_id):

    rospy.wait_for_service('retrieve_list')
    try:
        response = rospy.ServiceProxy('retrieve_list', RetrieveData)
        data = response(cache_id)

    except rospy.ServiceException as e:
        error = "Impossible audio retrieval ERROR: Service call failed: %s"%e
        print(error)
        raise Exception(error)
        
    
    
    if not data.success:
        error = "Impossible audio retrieval ERROR: "+ data.error
        print(error)
        raise Exception(error)

    #deserialize
    serialized = data.output

    numpy_audio = []

    for serial in serialized:
        numpy_audio.append(conversion_numpy_librosa(deserialize_audio(serial)))

    return numpy_audio

def conversion_numpy_librosa(arr):
    ret = np.array(arr).astype(np.float32)
    ret /= 32768
    ret[ret > 1] = 1.0
    ret[ret < -1] = -1.
    return (ret,16000)

def append_audio(cache_id,audio):

    rospy.wait_for_service('store_data_append')
    try:
        serialized = serialize_audio(audio)
        data_append = rospy.ServiceProxy('store_data_append', StoreData)
        resp = data_append("prova",serialized )

    except rospy.ServiceException as e:
        error = "Impossible audio append ERROR: Service call failed: %s"%e
        print(error)
        raise Exception(error)

    if not resp.success:
        error = "Impossible audio append, ERROR: "+ resp.error
        print(error)
        raise Exception(error)

    


def batch_cosine_similarity(x1, x2):
    '''
        x1,x2 must be l2 normalized
    '''

    # https://en.wikipedia.org/wiki/Cosine_similarity
    # 1 = equal direction ; -1 = opposite direction
    try:
        mul = np.multiply(x1, x2)
        s = np.sum(mul, axis=1)
    except ValueError:
        print("WARNING: Empty Embedding!!")
        return np.array([-1])

    # l1 = np.sum(np.multiply(x1, x1),axis=1)
    # l2 = np.sum(np.multiply(x2, x2), axis=1)
    # as values have have length 1, we don't need to divide by norm (as it is 1)
    return s


def dist2id(distance, y, ths, norm=False, mode='avg', filter_under_th=True):
    d = distance.copy()
    ths = np.array(ths)
    y = np.array(y)

    # remove elements under the threshold
    if filter_under_th:
        idx = d >= ths
        d = d[idx]
        y = y[idx]
        ths = ths[idx]

        if d.shape[0] == 0:
            return None,None

    if norm:
        # norm in case of different thresholds
        d = (d - ths)/(1-ths)

    ids = list(set(y.tolist()))

    ids_prob = []
    for i in ids:
        if mode == 'max':
            ids_prob.append(np.max(d[y == i]))
        if mode == 'avg':
            ids_prob.append(np.mean(d[y == i]))
        if mode == 'min':
            ids_prob.append(np.min(d[y == i]))

    ids_prob = np.array(ids_prob)
    id_max = np.argmax(ids_prob)
    return ids[id_max] ,ids_prob[id_max]
