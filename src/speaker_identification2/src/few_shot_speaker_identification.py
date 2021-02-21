#!/usr/bin/env python3
import time
import speech_recognition as sr
import numpy as np
import pickle

from identification.deep_speaker.audio import get_mel
'''from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id

if __name__ == '__main__':
    RATE = 16000

    # Load model
    model = get_deep_speaker('deep_speaker.h5')

    # Load embeddings, labels and thresholds
    with open('voice_identities/embed.pk', 'rb') as file:
        emb_dict = pickle.load(file)

    n_embs = len(emb_dict['y'])
    X = emb_dict['embeddings']
    y = emb_dict['y']
    ths = emb_dict['ths']

    # this is called from the background thread
    def callback(recognizer, audio):
        audio_data = np.frombuffer(audio.get_raw_data(),dtype=np.int16)
        # to float32
        audio_data = audio_data.astype(np.float32, order='C') / 32768.0

        # Processing
        ukn = get_mel(audio_data, RATE)

        # Prediction
        ukn = model.predict(np.expand_dims(ukn, 0))

        # Distance between the sample and the support set
        emb_voice = np.repeat(ukn, n_embs, 0)
        cos_dist = batch_cosine_similarity(X, emb_voice)
        
        # Matching
        id_label = dist2id(cos_dist, y, ths, mode='avg')

        # Rejection
        if id_label is None:
            id_label = "?"
        
        print(id_label)

    # Initialize a Recognizer
    r = sr.Recognizer()

    # Audio source
    m = sr.Microphone(sample_rate=16000)

    # Calibration within the environment
    # we only need to calibrate once, before we start listening
    print("Calibrating...")
    with m as source:
        r.adjust_for_ambient_noise(source)  
    print("Calibration finished")

    # start listening in the background
    # `stop_listening` is now a function that, when called, stops background listening
    print("Recording...")
    stop_listening = r.listen_in_background(m, callback)

    # do some unrelated computations for 60 seconds
    for _ in range(600): time.sleep(0.1)  # we're still listening even though the main thread is doing other things

    # calling this function requests that the background listener stop listening
    stop_listening(wait_for_stop=True)
    print("Script end.")
'''