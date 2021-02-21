#!/usr/bin/env python3
import os
from identification.identities_mng import get_identities
from identification.embed import embedd
from identification.utils import *
from identification.deep_speaker.audio import get_mel
from identification.deep_speaker.model import get_deep_speaker

import keras.backend as K

import pickle
import numpy as np
import librosa

librosa.load

class DeepSpeakerPreprocess:
    def __init__(self):
        self.sr = 16000

    def __call__(self, audio):
        '''
            audio : resampled audio frame
        '''

        mel = get_mel(audio[0], self.sr)

        return np.expand_dims(mel,0)


if __name__ == "__main__":
    

    #read_fun = ReadAudio(sr=16000)
    
    # X liste dei file, Y le identità nome e cognome ths threshold
    ids_folder=os.path.join(os.path.dirname(__file__),'voice_identities')
    out_path = os.path.join(ids_folder, 'embed.pk')

    identities = get_identities(ids_folder)
    
    X = []
    y = []
    ths = []
    for identity in identities:
        cache_id = identity["cache_id"]
        th = identity["th"]
        audio = retrieve_audio(cache_id)
        print("TIPO: "+str(type(audio[0])))
        X = X + audio
        y = y + [cache_id]*len(audio)
        ths = ths + [float(th)]*len(audio)

    #X, y, ths = get_identities(ids_folder, read_file=read_fun)

   
    model = get_deep_speaker(os.path.join(os.path.dirname(__file__),'deep_speaker.h5'))

    preprocess = DeepSpeakerPreprocess()

    out_dict = embedd(X, y, ths, model, preprocess_input=preprocess)

    with open(out_path, 'wb') as file:
        pickle.dump(out_dict, file)
