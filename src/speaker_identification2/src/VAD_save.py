import time
import speech_recognition as sr
import numpy as np
import pickle

from scipy.io import wavfile

if __name__ == '__main__':
    RATE = 16000

    file_id = 0

    # this is called from the background thread
    def callback(recognizer, audio):
        global file_id

        audio_data = np.frombuffer(audio.get_raw_data(),dtype=np.int16)

        # Salva il nel il file della registrazione nella cartella recordings
        wavfile.write('./recordings/record_%05d.wav'%file_id,RATE,audio_data)

        file_id+=1


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