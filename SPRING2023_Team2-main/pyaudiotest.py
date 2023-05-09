import pyaudio
import wave
import numpy as np
import matplotlib.pyplot as plt


FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 512
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "recordedtonetest.wav"
device_index = 1
audio = pyaudio.PyAudio() # sets our audio/Pyaudio device, i think

print("----------------------record device list---------------------")
info = audio.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
for i in range(0, numdevices): # loop thorugh all devices it sees. Then print their index and their name
        if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id ", i, " - ", audio.get_device_info_by_host_api_device_index(0, i).get('name'))

print("-------------------------------------------------------------")

index = 1 #int(input())  index of the usb device we want/are using
print("recording via index "+str(index))

stream = audio.open(format=FORMAT, channels=CHANNELS,
                rate=RATE, input=True,input_device_index = index,
                frames_per_buffer=CHUNK)
print ("recording started")
Recordframes = []
 
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    Recordframes.append(data)
print ("recording stopped")
 
stream.stop_stream()
stream.close()
audio.terminate()


numpydata = np.frombuffer(data, dtype=np.int16)
n = numpydata.shape[0]
yf = np.fft.rfft(numpydata)
fstep = RATE / n
freqs = np.arange(yf.shape[0]) * fstep

yff = np.abs(yf)
 
plt.figure(1)
plt.title("Sign")
plt.plot(freqs,yff)
plt.show()
 
waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
waveFile.setnchannels(CHANNELS)
waveFile.setsampwidth(audio.get_sample_size(FORMAT))
waveFile.setframerate(RATE)
waveFile.writeframes(b''.join(Recordframes))
waveFile.close()
