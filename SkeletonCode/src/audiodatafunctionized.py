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
from threading import Thread
cutoff = 4500#LOWER BOUND
cutoff2 = 5000 #UPPER BOUND
WIDTH = 2
FORMAT = pyaudio.paInt16
MIC1 = 1
MIC2 = 3
RATE = 44100 #SAMPLING FREQ
CHUNK = 1024
rms = 1
audio = pyaudio.PyAudio()
freqs = np.linspace(0, RATE,CHUNK)

class triangulation(object):
    def __init__(self):
        self.stream1 = self.open_mic1_stream()
        self.stream2 = self.open_mic2_stream()
        self.data1 = self.data()
        self.fitlter = self.filtered()
        self.conv = self.conversions()
        self.inten = self.intensity()

    def open_mic1_stream():
        stream1 = audio.open(format=audio.get_format_from_width(WIDTH), 
                        channels=1,
                        rate=RATE, input=True,
                        input_device_index=MIC1,
                        frames_per_buffer=CHUNK)
        return stream1
    
    def open_mic2_stream():
        stream2 = audio.open(format=audio.get_format_from_width(WIDTH), 
                        channels=1,
                        rate=RATE, input=True,
                        input_device_index=MIC2,
                        frames_per_buffer=CHUNK)
        return stream2

    def data(stream):
        frames = []
        data = stream.read(CHUNK)
        frames.append(data)
        numpydata = np.frombuffer(data, dtype=np.int16)
        n = numpydata.shape[0]
        return numpydata,n

    def filtered(cutoff,cutoff2,RATE,numpydata,n,mic):
        b, a = signal.butter(2,[cutoff, cutoff2],'bandpass', analog = False, fs = RATE)
        yfilt = signal.lfilter(b,a,numpydata)
        yf = np.fft.rfft(yfilt)
        fstep = RATE / n
        freqs = np.arange(yf.shape[0]) * fstep
        yff = np.abs(yf)*2/(11000*CHUNK)
        db = 160+25 *log10(np.mean(np.abs(yff/4)))
        #print(db)
        tm = time.time()-1682440230.7503483
        if (db > 40):
            print(f"Mic: {mic}",f" dB: {db}", f"time: {tm}")
            #time.sleep(.5) 
        return yff,freqs, tm, db

stream1 = triangulation.open_mic1_stream()
stream2 = triangulation.open_mic2_stream()
#stream = True
#while stream:
stream1threshold = False
stream2threshold = False

while  not stream1threshold  or not stream2threshold:
    data1 = triangulation.data(stream1)
    if not stream1threshold:
        M1filt = triangulation.filtered(cutoff,cutoff2,RATE,data1[0],data1[1],MIC1)
        if(M1filt[3] > 40):
            stream1threshold = True
    data2 = triangulation.data(stream2)
    if not stream2threshold:
        M2filt = triangulation.filtered(cutoff,cutoff2,RATE,data2[0],data2[1],MIC2)
        if(M2filt[3] > 40):
            stream2threshold = True

timediff = abs(M1filt[2]- (M2filt[2]-.001))
print(timediff)
dist1 = 343*M1filt[2]
dist2 = 343*M2filt[2]
time.sleep(3)
# stop Recording
stream1.stop_stream()
stream2.stop_stream()
stream1.close()
stream2.close()
audio.terminate()
