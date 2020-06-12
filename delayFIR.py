from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math


# define necessary utility functions

# parameters
full_buffer_len = 10240
half_buffer_len = int(full_buffer_len/2)
# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

#delay parameters:
delaygain = 1
delaylen = 5120 #must be less than half_buffer_len. 5120-> 160ms
print("delay is ", delaylen/samp_freq,'s')
n_buffers = int(np.floor(len(signal) / half_buffer_len))
n_buffers = int(np.floor(n_buffers/2)*2) #if uneven n_buffers, nbuffers= nbuffers-1
data_type = signal.dtype

print("Sampling frequency : %d Hz" % samp_freq)
print("Data type          : %s" % signal.dtype)

# allocate input and output buffers
input_buffer = np.zeros(full_buffer_len, dtype=data_type)
output_buffer = np.zeros(full_buffer_len, dtype=data_type)


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
                output_buffer[n] = input_buffer[n] + delaygain * input_buffer[n-delaylen]
            else:
                output_buffer[n] = input_buffer[n] + delaygain * input_buffer[n - delaylen + full_buffer_len]
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            output_buffer[n] = input_buffer[n] + delaygain * input_buffer[n-delaylen]

    return input_buffer, output_buffer

"""
Nothing to touch after this!
"""
init()
# simulate block based processing
print(n_buffers*half_buffer_len)
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

# write to WAV
wavfile.write("delayFIR.wav", samp_freq, signal_proc)