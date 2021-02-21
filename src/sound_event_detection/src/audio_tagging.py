import numpy as np
import pyaudio
from matplotlib import pyplot as plt
import pandas as pd
import sounddevice as sd

from keras_yamnet import params
from keras_yamnet.yamnet import YAMNet, class_names, class_id
from keras_yamnet.preprocessing import preprocess_input

from plot import Plotter
import speech_recognition as sr
import time

if __name__ == "__main__":


    ################### SETTINGS ###################
    plt_classes = [0,132,420,494] # Speech, Music, Explosion, Silence 
    class_labels=True
    RATE = params.SAMPLE_RATE
    WIN_SIZE_SEC = 0.975
    CHUNK = int(WIN_SIZE_SEC * RATE)
    

    #################### MODEL #####################
    
    model = YAMNet(weights='keras_yamnet/'+'yamnet.h5')
    yamnet_classes = class_names('keras_yamnet/'+'yamnet_class_map.csv')
    yamnet_classes_id = class_id('keras_yamnet/'+'yamnet_class_map.csv')
    
    for i in range(0,len(yamnet_classes_id)):
        yamnet_classes_id[i] = int(yamnet_classes_id[i])
    
    
    # this is called from the background thread
    def callback(recognizer, audio):
        print("Sono qui")
        audio_data = np.frombuffer(audio.get_raw_data(),dtype=np.int16)
        ret = np.array(audio_data.data).astype(np.float32)
        ret /= 32768
        ret[ret > 1] = 1.0
        ret[ret < -1] = -1.
        
        stream_in_chunk = int(len(ret)/len(ret[:CHUNK]))
        
        speech=0

        for i in range(0,stream_in_chunk):
            if i == 0:
                data = preprocess_input(np.fromstring(
                    ret[:CHUNK], dtype=np.float32), RATE)
            else:
                data = preprocess_input(np.fromstring(
                    ret[i*CHUNK:(i*CHUNK)+CHUNK], dtype=np.float32), RATE)
            prediction = model.predict(np.expand_dims(data,0))[0]
            max_probability = max(np.expand_dims(prediction[yamnet_classes_id],-1))
            if max_probability == prediction[yamnet_classes_id[0]]:
                speech = speech+1
        
        if speech >= stream_in_chunk/2:
            print("Speech")
        else:
            print("No Speech")

    #################### STREAM ####################
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
    if plt_classes is not None:
        plt_classes_lab = yamnet_classes[plt_classes]
        n_classes = len(plt_classes)
    else:
        plt_classes = [k for k in range(len(yamnet_classes))]
        plt_classes_lab = yamnet_classes if class_labels else None
        n_classes = len(yamnet_classes)

    monitor = Plotter(n_classes=n_classes, FIG_SIZE=(12,6), msd_labels=plt_classes_lab)

    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        # Waveform
        data = preprocess_input(np.fromstring(
            stream.read(CHUNK), dtype=np.float32), RATE)
        prediction = model.predict(np.expand_dims(data,0))[0]

        monitor(data.transpose(), np.expand_dims(prediction[plt_classes],-1))

    print("finished recording")

    # stop Recording
    stream.stop_stream()
    stream.close()
    audio.terminate()
    # calling this function requests that the background listener stop listening
    stop_listening(wait_for_stop=True)
    print("Script end.")
'''