#!/usr/bin/env python3
from identification.identities_mng import get_identities
from identification.embed import embedd
from identification.utils import *
from identification.deep_speaker.audio import get_mel
from identification.deep_speaker.model import get_deep_speaker

import keras.backend as K
import os
import pickle
import numpy as np
import librosa



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
    ids_folder = 'voice_identities'
    out_path = os.path.join(ids_folder, 'embed.pk')

    #read_fun = ReadAudio(sr=16000)

    # X liste dei file, Y le identità nome e cognome ths threshold
    identities = get_identities(ids_folder)
    
    X = []
    y = []
    ths = []
    for identity in identities:
        cache_id = identity["cache_id"]
        th = identity["th"]
        audio = retrieve_audio(cache_id)
        X = X + audio
        y = y + [cache_id]*len(audio)
        ths = ths + [float(th)]*len(audio)

    #X, y, ths = get_identities(ids_folder, read_file=read_fun)

    model = get_deep_speaker('deep_speaker.h5')

    preprocess = DeepSpeakerPreprocess()

    out_dict = embedd(X, y, ths, model, preprocess_input=preprocess)

    with open(out_path, 'wb') as file:
        pickle.dump(out_dict, file)
