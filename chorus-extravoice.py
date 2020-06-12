from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math


# define necessary utility functions

# parameters
full_buffer_len = 2000
half_buffer_len = int(full_buffer_len/2)
# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

#delay parameters:
delaygain = 0.5
ingain=0.5

offset= 0.01
depth = 0.015
delayfreq = 0.7

offset2= 0.01
depth2 = 0.02
delayfreq2 = 0.3

offset3= 0.01
depth3 = 0.03
delayfreq3 = 0.5

maxnum = 60000

nglob = 0

#print('delay between :', maxdelaylen/samp_freq ,'seconds and ',mindelaylen/samp_freq)
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
def process(input_buffer, output_buffer, buffer_len, case):
    global nglob

    # specify global variables modified here
    if(case == True):
        # process one sample at a time
        for n in range(0,buffer_len):
            y = delaygain * delaysig(input_buffer,n, case) + ingain*input_buffer[n]
            output_buffer[n] =(y/maxnum) *32767
            nglob = nglob + 1
    else:
        # process one sample at a time
        for n in range(buffer_len,2*buffer_len):
            y = delaygain * delaysig(input_buffer, n, case) + ingain * input_buffer[n]
            output_buffer[n] = (y / maxnum) * 32767
            nglob = nglob+1

    return input_buffer, output_buffer

def delaysig(input_buffer, n, case):
    if (case == True):
        if (n >= delaylen(nglob)) :

            delaysig = input_buffer[n - delaylen(nglob)]
        else:

            delaysig = input_buffer[n - delaylen(nglob) + full_buffer_len]

        if (n >= delaylen2(nglob)) :

            delaysig = delaysig + input_buffer[n - delaylen2(nglob)]
        else:

            delaysig = delaysig + input_buffer[n - delaylen2(nglob)+full_buffer_len]

        if (n >= delaylen3(nglob)) :

            delaysig = delaysig + input_buffer[n - delaylen3(nglob)]
        else:

            delaysig = delaysig + input_buffer[n - delaylen3(nglob)+full_buffer_len]
    else:
        delaysig = input_buffer[n - delaylen(nglob)] + input_buffer[n - delaylen2(nglob)] + input_buffer[n - delaylen3(nglob)]

    return delaysig

def delaylen(n):
    delaylen = offset + (depth/2)*(1-np.cos(n*2*np.pi*delayfreq/samp_freq))
    if n %10 == 0:
        print('delay:',delaylen,'seconds')
    delaylen = delaylen*samp_freq
    return int(delaylen)

def delaylen2(n):
    delaylen = offset2 + (depth2/2)*(1-np.cos(n*2*np.pi*delayfreq2/samp_freq))
    if n %10 == 0:
        print('delay2:',delaylen,'seconds')
    delaylen = delaylen*samp_freq
    return int(delaylen)
def delaylen3(n):
    delaylen = offset3 + (depth3/2)*(1-np.cos(n*2*np.pi*delayfreq3/samp_freq))
    if n %10 == 0:
        print('delay3:',delaylen,'seconds')
    delaylen = delaylen*samp_freq
    return int(delaylen)

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
wavfile.write("chorus-extravoice.wav", samp_freq, signal_proc)