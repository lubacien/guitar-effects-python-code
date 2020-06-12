from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math


# define necessary utility functions

# parameters
buffer_len = 5100
delay_buffer_len=5100
delay_buffer_slice=510
delay=1000 #must be larger than bufferlen/10 and smaller than 510
# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

gain = 1

n_buffers = len(signal) // buffer_len
data_type = signal.dtype

print("Sampling frequency : %d Hz" % samp_freq)
print("Data type          : %s" % signal.dtype)

# allocate input and output buffers
input_buffer = np.zeros(buffer_len, dtype=data_type)
output_buffer = np.zeros(buffer_len, dtype=data_type)
delay_buffer = np.zeros(delay_buffer_len, dtype=data_type)


# state variables
def init():

    # declare variables used in `process`
    # global

    # define variables, lookup tables

    return


# the process function!
def process(input_buffer, output_buffer, buffer_len):

    # specify global variables modified here

#from input to buffer:

    #filtering

    #we put the end of the buffer at the beginning
    delay_buffer[0:delay_buffer_len-delay_buffer_slice] = delay_buffer[delay_buffer_slice : delay_buffer_len]

    #we lowpass and downsample the signal and put it at the end of the buffer.
    b,a = scisig.butter(10, (math.pi/10)/math.pi) #the cutoff frequency is pi/10 because we will downsample 10 times. order 10.
    input_bufferlf = scisig.lfilter(b,a,input_buffer)

    buff = np.empty(delay_buffer_slice)
    for i in range(delay_buffer_slice):
        buff[i] = input_bufferlf[i*10]
    delay_buffer[delay_buffer_len-delay_buffer_slice:delay_buffer_len] = buff


#from buffer to output:

    delaysig = np.zeros(buffer_len, dtype=data_type)
    # process one sample at a time
    for n in range(buffer_len):
        if np.mod(n,10) == 0:
            delaysig[n] = delay_buffer[int(np.floor(n / 10)) + delay_buffer_len - delay]
    #filtering after reconstruction
    b, a = scisig.butter(10, (math.pi / 10) / (math.pi))  # the cutoff frequency is pi/10 because we will downsample 10 times. order 10.
    delaysig = scisig.lfilter(b, a, delaysig)

    output_buffer =   input_buffer+ 5*delaysig

    return input_buffer, output_buffer


"""
Nothing to touch after this!
"""
init()
# simulate block based processing
signal_proc = np.zeros(n_buffers*buffer_len, dtype=data_type)

print(n_buffers)

for k in range(n_buffers):
    # index the appropriate samples
    input_buffer = signal[k*buffer_len : (k+1)*buffer_len]
    input_buffer,output_buffer = process(input_buffer, output_buffer, buffer_len)
    signal_proc[k*buffer_len:(k+1)*buffer_len] = output_buffer

# write to WAV
wavfile.write("delaydecimator.wav", samp_freq, signal_proc)