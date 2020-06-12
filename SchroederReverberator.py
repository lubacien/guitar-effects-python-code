from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math
import matplotlib
import matplotlib.pyplot as plt
import copy

# define necessary utility functions

# parameters
full_buffer_len =12800
half_buffer_len = int(full_buffer_len/2)
# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

#delay parameters:
# first 4 delaytimes between 30 and 45 ms
T=1 #reverberation time
def delaylen(time):
    return int(time*32000)

def gain(time):
    global T
    return np.abs(np.exp(-3*time/T))

time1 = 0.03
delaylen1 = delaylen(time1)
gain1 = gain(time1)

time2 = 0.037
delaycomb2 = delaylen(time2)
gaincomb2 = gain(time2)

time3 = 0.042
delaycomb3 = delaylen(time3)
gaincomb3 = gain(time3)

time4 = 0.045
delaycomb4 = delaylen(time4)
gaincomb4 = gain(time4)

gain2 = 0.7
delaylen2 = int(0.005*32000)

gain3 = 0.7
delaylen3 = int(0.0017*32000)

reverbgain = 1
plot = False#plots the impulse response when true

# state variables

def process(inbuff, buffer_len, case, delaylen, delaylength2, delaylength3, delaylength4, gain, gain2, gain3, gain4):
    global other_buffer
    global other_buffer5
    global other_buffer6
    global other_buffer4

    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                other_buffer[n] = inbuff[n-delaylen] + gain * other_buffer[n-delaylen]
            else:

                other_buffer[n] = inbuff[n-delaylen+full_buffer_len] + gain * other_buffer[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer[n] = inbuff[n-delaylen] + gain * other_buffer[n-delaylen]


    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylength2:
                other_buffer5[n] = inbuff[n-delaylength2] + gain2 * other_buffer5[n-delaylength2]
            else:
                other_buffer5[n] = inbuff[n-delaylength2+full_buffer_len] + gain2 * other_buffer5[n-delaylength2+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer5[n] = inbuff[n-delaylength2] + gain2 * other_buffer5[n-delaylength2]


    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylength3:
                other_buffer6[n] = inbuff[n-delaylength3] + gain3 * other_buffer6[n-delaylength3]
            else:
                other_buffer6[n] = inbuff[n-delaylength3+full_buffer_len]+ gain3 * other_buffer6[n-delaylength3+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer6[n] = inbuff[n-delaylength3] + gain3 * other_buffer6[n-delaylength3]

    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylength4:
                other_buffer4[n] = inbuff[n-delaylength4]+ gain4 * other_buffer4[n-delaylength4]
            else:
                other_buffer4[n] = inbuff[n-delaylength4+full_buffer_len]+ gain4 * other_buffer4[n-delaylength4+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer4[n] = inbuff[n-delaylength4]+ gain4 * other_buffer4[n-delaylength4]

    return (other_buffer + other_buffer5 + other_buffer6 + other_buffer4)



def process2(inbuff, buffer_len, case, delaylen, gain):
    global other_buffer2
    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                other_buffer2[n] = -gain*inbuff[n] + gain * other_buffer2[n-delaylen] + inbuff[n-delaylen]
            else:
                other_buffer2[n] = -gain*inbuff[n] + gain * other_buffer2[n-delaylen+full_buffer_len] + inbuff[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer2[n] = -gain*inbuff[n] + gain * other_buffer2[n-delaylen] + inbuff[n-delaylen]

    return other_buffer2

def process3(inbuff, buffer_len, case, delaylen, gain):
    global other_buffer3
    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                other_buffer3[n] = -gain*inbuff[n] + gain * other_buffer3[n-delaylen] + inbuff[n-delaylen]
            else:
                other_buffer3[n] = -gain*inbuff[n] + gain * other_buffer3[n-delaylen+full_buffer_len] + inbuff[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer3[n] = -gain*inbuff[n] + gain * other_buffer3[n-delaylen] + inbuff[n-delaylen]

    return other_buffer3

if (plot == True):
    signal = np.zeros(48000)
    #time = range(48000)/32000
    signal[0] = 1

n_buffers = int(np.floor(len(signal) / half_buffer_len))
n_buffers = int(np.floor(n_buffers/2)*2) #if uneven n_buffers, nbuffers= nbuffers-1
data_type = int #signal.dtype

# allocate input and output buffers
input_buffer = np.zeros(full_buffer_len, dtype=data_type)
output_buffer = np.zeros(full_buffer_len, dtype=data_type)

other_buffer2 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer3 = np.zeros(full_buffer_len, dtype=data_type)

other_buffer4 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer5 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer6 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer = np.zeros(full_buffer_len, dtype=data_type)

print("Sampling frequency : %d Hz" % samp_freq)
print("Data type          : %s" % signal.dtype)

signal_proc = np.zeros(int(n_buffers*half_buffer_len), dtype=data_type)

for k in range(0, n_buffers, 2):
    # fill and process first half of buffer
    input_buffer[0:half_buffer_len] = signal[k*half_buffer_len : (k+1)*half_buffer_len]

    output_buffer = process(input_buffer, half_buffer_len, True, delaylen1, delaycomb2, \
                            delaycomb3, delaycomb4, gain1, gaincomb2, gaincomb3, gaincomb4)
    output_buffer = process2(output_buffer, half_buffer_len, True, delaylen2, gain2)
    output_buffer = process3(output_buffer, half_buffer_len, True, delaylen3, gain3)

    signal_proc[k*half_buffer_len:(k+1)*half_buffer_len] = output_buffer[0:half_buffer_len]

    # fill and process second half of buffer
    input_buffer[half_buffer_len: 2*half_buffer_len] = signal[(k+1) * half_buffer_len: (k + 2) * half_buffer_len]
    output_buffer = process(input_buffer, half_buffer_len, False, delaylen1, delaycomb2, \
                            delaycomb3, delaycomb4, gain1, gaincomb2, gaincomb3, gaincomb4)
    output_buffer = process2(output_buffer, half_buffer_len, False, delaylen2, gain2)
    output_buffer = process3(output_buffer, half_buffer_len, False, delaylen3, gain3)

    signal_proc[(k+1) * half_buffer_len:(k + 2) * half_buffer_len] = output_buffer[half_buffer_len:2*half_buffer_len]

signal_proc = signal_proc * reverbgain #+ signal[0:len(signal_proc)]

time = np.array(range(len(signal_proc)))
time = time/32000

plt.plot(time, signal_proc/max(signal_proc))
plt.show()
plt.plot(signal)
plt.show()
# write to WAV
wavfile.write("schroederreverb.wav", samp_freq, signal_proc/max(signal_proc))