from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math
import matplotlib
import matplotlib.pyplot as plt


# define necessary utility functions

# parameters
full_buffer_len =12800
half_buffer_len = int(full_buffer_len/2)
# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

#delay parameters:
gain = 0.5
delaylen = int(0.2*32000) #must be less than half_buffer_len

plot = False
# state variables
def init():

    # declare variables used in `process`
    # global

    # define variables, lookup tables

    return


# the process function!
def process(input_buffer, output_buffer, buffer_len, case ):

    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            if n >= delaylen:
                output_buffer[n] = -gain*input_buffer[n] + gain * output_buffer[n-delaylen] + (1-gain**2)*input_buffer[n-delaylen]
            else:
                output_buffer[n] = -gain*input_buffer[n] + gain * output_buffer[n-delaylen+full_buffer_len] + (1-gain**2)*input_buffer[n-delaylen+full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            output_buffer[n] = -gain*input_buffer[n] + gain * output_buffer[n-delaylen] + (1-gain**2)*input_buffer[n-delaylen]

    return input_buffer, output_buffer

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

print("Sampling frequency : %d Hz" % samp_freq)
print("Data type          : %s" % signal.dtype)

signal_proc = np.zeros(int(n_buffers*half_buffer_len), dtype=data_type)



for k in range(0,n_buffers,2):

    # fill and process first half of buffer
    input_buffer[0:half_buffer_len] = signal[k*half_buffer_len : (k+1)*half_buffer_len]
    input_buffer,output_buffer = process(input_buffer, output_buffer, half_buffer_len,True)
    signal_proc[k*half_buffer_len:(k+1)*half_buffer_len] = output_buffer[0:half_buffer_len]

    #print(input_buffer[half_buffer_len: 2*half_buffer_len].shape)
    # fill and process second half of buffer
    input_buffer[half_buffer_len: 2*half_buffer_len] = signal[(k+1) * half_buffer_len: (k + 2) * half_buffer_len]
    input_buffer, output_buffer = process(input_buffer, output_buffer, half_buffer_len,False)
    signal_proc[(k+1) * half_buffer_len:(k + 2) * half_buffer_len] = output_buffer[half_buffer_len:2*half_buffer_len]

time = np.array(range(len(signal_proc)))
time = time/32000

plt.plot(time,signal_proc)
plt.show()

# write to WAV
wavfile.write("delayallpass.wav", samp_freq, signal_proc)