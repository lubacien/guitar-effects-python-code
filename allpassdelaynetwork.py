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
gain1 = 0.7
delaylen1 = int(0.1*32000)

gain2 = -0.7
delaylen2 = int(0.068*32000)

gain3 = 0.7
delaylen3 = int(0.06*32000)

gain4 = 0.7
delaylen4 = int(0.0197*32000)

gain5 = 0.7
delaylen5 = int(0.00585*32000)

plot = False #plots the impulse response
# state variables
def init():

    # declare variables used in `process`
    # global

    # define variables, lookup tables

    return


# the process function!
def process(inbuff, buffer_len, case, delaylen, gain):
    global other_buffer
    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                other_buffer[n] = -gain*inbuff[n] + gain * other_buffer[n-delaylen] + inbuff[n-delaylen]
            else:
                other_buffer[n] = -gain*inbuff[n] + gain * other_buffer[n-delaylen+full_buffer_len] + inbuff[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer[n] = -gain*inbuff[n] + gain * other_buffer[n-delaylen] + inbuff[n-delaylen]

    return other_buffer

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

def process4(inbuff, buffer_len, case, delaylen, gain):
    global other_buffer4
    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                other_buffer4[n] = -gain*inbuff[n] + gain * other_buffer4[n-delaylen] + inbuff[n-delaylen]
            else:
                other_buffer4[n] = -gain*inbuff[n] + gain * other_buffer4[n-delaylen+full_buffer_len] + inbuff[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer4[n] = -gain*inbuff[n] + gain * other_buffer4[n-delaylen] + inbuff[n-delaylen]

    return other_buffer4

def process5(inbuff, buffer_len, case, delaylen, gain):
    global other_buffer5
    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                other_buffer5[n] = -gain*inbuff[n] + gain * other_buffer5[n-delaylen] + inbuff[n-delaylen]
            else:
                other_buffer5[n] = -gain*inbuff[n] + gain * other_buffer5[n-delaylen+full_buffer_len] + inbuff[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            other_buffer5[n] = -gain*inbuff[n] + gain * other_buffer5[n-delaylen] + inbuff[n-delaylen]

    return other_buffer5
"""
Nothing to touch after this!
"""
init()

if (plot == True):
    signal = np.zeros(48000)
    #time = range(48000)/32000
    signal[0] = 1

n_buffers = int(np.floor(len(signal) / half_buffer_len))
n_buffers = int(np.floor(n_buffers/2)*2) #if uneven n_buffers, nbuffers= nbuffers-1
data_type = signal.dtype

# allocate input and output buffers
input_buffer = np.zeros(full_buffer_len, dtype=data_type)
output_buffer = np.zeros(full_buffer_len, dtype=data_type)
other_buffer = np.zeros(full_buffer_len, dtype=data_type)
other_buffer2 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer3 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer4 = np.zeros(full_buffer_len, dtype=data_type)
other_buffer5 = np.zeros(full_buffer_len, dtype=data_type)
print("Sampling frequency : %d Hz" % samp_freq)
print("Data type          : %s" % signal.dtype)

signal_proc = np.zeros(int(n_buffers*half_buffer_len), dtype=data_type)

for k in range(0, n_buffers, 2):
    # fill and process first half of buffer
    input_buffer[0:half_buffer_len] = signal[k*half_buffer_len : (k+1)*half_buffer_len]
    output_buffer = process(input_buffer, half_buffer_len, True, delaylen1, gain1)
    output_buffer = process2(output_buffer, half_buffer_len, True, delaylen2, gain2)
    output_buffer = process3(output_buffer, half_buffer_len, True, delaylen3, gain3)
    output_buffer = process4(output_buffer, half_buffer_len, True, delaylen4, gain4)
    output_buffer = process5(output_buffer, half_buffer_len, True, delaylen5, gain5)
    signal_proc[k*half_buffer_len:(k+1)*half_buffer_len] = output_buffer[0:half_buffer_len]

    # fill and process second half of buffer
    input_buffer[half_buffer_len: 2*half_buffer_len] = signal[(k+1) * half_buffer_len: (k + 2) * half_buffer_len]
    output_buffer = process(input_buffer, half_buffer_len, False, delaylen1, gain1)
    output_buffer = process2(output_buffer, half_buffer_len, False, delaylen2, gain2)
    output_buffer = process3(output_buffer, half_buffer_len, False, delaylen3, gain3)
    output_buffer = process4(output_buffer, half_buffer_len, False, delaylen4, gain4)
    output_buffer = process5(output_buffer, half_buffer_len, False, delaylen5, gain5)

    signal_proc[(k+1) * half_buffer_len:(k + 2) * half_buffer_len] = output_buffer[half_buffer_len:2*half_buffer_len]

time = np.array(range(len(signal_proc)))
time = time/32000

plt.plot(time, signal_proc)
plt.show()

# write to WAV
wavfile.write("delayallpassnetwork.wav", samp_freq, signal_proc)