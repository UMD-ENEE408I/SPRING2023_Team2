#continuous recording test file


import pyaudio
import wave
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import time
from math import log10
import audioop  
from pydub import AudioSegment
from pydub.silence import detect_nonsilent

cutoff = 750#LOWER BOUND
cutoff2 = 1250 #UPPER BOUND

WIDTH = 2
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100 #SAMPLING FREQ
CHUNK = 1024
#RECORD_SECONDS = 2
#WAVE_OUTPUT_FILENAME = "tonetest.wav"


rms = 1
def callback(in_data, frame_count, time_info, status):
    global rms
    rms = audioop.rms(in_data, WIDTH) / 32767
    return in_data, pyaudio.paContinue

#adjust target amplitude
def match_target_amplitude(sound, target_dBFS):
    change_in_dBFS = target_dBFS - sound.dBFS
    return sound.apply_gain(change_in_dBFS)
#==========================================================RECORDING SECTION 
audio = pyaudio.PyAudio()

# start Recording
stream = audio.open(format=audio.get_format_from_width(WIDTH), 
                    channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK)
frames = []
stream.start_stream()

#x axis
freqs = np.linspace(0, RATE,CHUNK)


while  stream.is_active():
    #read data in
    data = stream.read(CHUNK)
    frames.append(data)
    numpydata = np.frombuffer(data, dtype=np.int16)
    n = numpydata.shape[0]

    #Filter
    b, a = signal.butter(2,[cutoff, cutoff2],'bandpass', analog = False, fs = RATE)
    yfilt = signal.lfilter(b,a,numpydata) #THIS CONVOLVEES THE FILTER WITH THE SIGNAL

    #FFT conversions
    yf = np.fft.rfft(yfilt)
    fstep = RATE / n
    freqs = np.arange(yf.shape[0]) * fstep
    yff = np.abs(yf)*2/(11000*CHUNK)

#NO FILTER 
    yfs = np.fft.rfft(numpydata)
    fsteps = RATE / n
    freqss = np.arange(yfs.shape[0]) * fsteps
    yfff = np.abs(yfs)*2/(11000*CHUNK)

    #decibel reading
    db = 160+25 *log10(np.mean(np.abs(yff/4)))
    print(db)
    #plotting filtered signal
    plt.figure(1)
    plt.plot(freqs,yff)# 10*np.log10(yfff))
    plt.title('signal')
    plt.legend(['unfiltered','filtered'])
    plt.ylim([0, .001])
    plt.xlim([0,5000])
    plt.xlabel('Frequency')
    plt.ylabel('Amplitude [dB]')
    plt.margins(0, 0.5)
    plt.grid(which='both', axis='both')
    #plt.axvline(5250, color='green') # cutoff frequency
    
    plt.draw()
    plt.pause(0.00001)
    plt.clf()

#TIME STAMP CODE============================================================
#Convert wav to audio_segment
audio_segment = yfilt

#normalize audio_segment to -20dBFS 
normalized_sound = match_target_amplitude(audio_segment, -20.0)
print("length of audio_segment={} seconds".format(len(normalized_sound)/1000))

#Print detected non-silent chunks, which in our case would be spoken words.
nonsilent_data = detect_nonsilent(normalized_sound, min_silence_len=500, silence_thresh=-20, seek_step=1)

#convert ms to seconds
print("First contact:")
print((nonsilent_data[0]))
#for chunks in nonsilent_data:
# print( [chunk/1000 for chunk in chunks])




# stop Recording
stream.stop_stream()
stream.close()
audio.terminate()