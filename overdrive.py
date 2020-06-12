from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math


# define necessary utility functions

# parameters
full_buffer_len = 1024
half_buffer_len = int(full_buffer_len/2)
# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

gain = 50

n_buffers = len(signal) // half_buffer_len
data_type = signal.dtype

print("Sampling frequency : %d Hz" % samp_freq)
print("Data type          : %s" % signal.dtype)

# allocate input and output buffers
input_buffer = np.zeros(full_buffer_len, dtype=data_type)
output_buffer = np.zeros(full_buffer_len, dtype=data_type)

MAX_INT = 32767
MAX_TH23 = MAX_INT/3*2
MAX_TH13 = MAX_INT/3

signal = 10*signal #gain 10 for hard,
# state variables
def init():

    # declare variables used in `process`
    # global

    # define variables, lookup tables

    return

def softdistort(x16):
    if (x16 > MAX_TH23) :
        return MAX_INT;

    elif (x16 < -MAX_TH23):
        return -MAX_INT - 1;

    elif (x16 > MAX_TH13 and x16 <= MAX_TH23):
        return (3-(2-3*x16)**2)/3

    elif (x16 < -MAX_TH13 and x16 >= -MAX_TH23):
        return -(3 - (2 - 3 * x16) ** 2) / 3
    else :
        return 2*x16

def harddistort(x16):
    if (x16 > MAX_TH23) :
        return MAX_INT;

    elif (x16 < -MAX_TH23):
        return -MAX_INT - 1;

    else :
        return x16*3/2

# the process function!
def process(input_buffer, output_buffer, buffer_len, case ):

    # specify global variables modified here
    if(case == False):
        # process one sample at a time
        for n in range(0,buffer_len):
            output_buffer[n] = softdistort(input_buffer[n])
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            output_buffer[n] = softdistort(input_buffer[n])

    return input_buffer, output_buffer

"""
Nothing to touch after this!
"""
init()
# simulate block based processing
print(n_buffers*half_buffer_len)
signal_proc = np.zeros(int(n_buffers*half_buffer_len), dtype=data_type)

for k in range(0,int(n_buffers),2):

    # fill and process first half of buffer
    input_buffer[0:half_buffer_len] = signal[k*half_buffer_len : (k+1)*half_buffer_len]
    input_buffer,output_buffer = process(input_buffer, output_buffer, half_buffer_len,False)
    signal_proc[k*half_buffer_len:(k+1)*half_buffer_len] = output_buffer[0:half_buffer_len]

    # fill and process second half of buffer
    input_buffer[half_buffer_len: 2*half_buffer_len] = signal[(k+1) * half_buffer_len: (k + 2) * half_buffer_len]
    input_buffer, output_buffer = process(input_buffer, output_buffer, half_buffer_len,True)
    signal_proc[(k+1) * half_buffer_len:(k + 2) * half_buffer_len] = output_buffer[half_buffer_len:2*half_buffer_len]

# write to WAV
wavfile.write("softclip.wav", samp_freq, signal_proc)