from scipy.io import wavfile
import numpy as np
import scipy.signal as scisig
import math

# test signal
input_wav = "guitar32000.wav"
samp_freq, signal = wavfile.read(input_wav)
signal = signal[:,0]  # get first channel

offset= 0.01
depth = 0.015
chorusfreq = 0.7

def make_cos_table(period):
    c = 0x7FFF * np.cos(2 * np.pi * np.arange(0, period) / period)
    #c = offset * samp_freq + depth * 0.5 * samp_freq * (1-np.cos(2 * np.pi * np.arange(0, period) / period))
    print('#define COS_TABLE_LEN {}'.format(period))
    print('static int16_t COS_TABLE[COS_TABLE_LEN] = {', end='\n\t')
    for n in range(period - 1):
        print('0x{:04X}, '.format(np.uint16(c[n])), \
            end='' + '\n\t' if (n+1) % 12 == 0 else '')
    print('0x{:04X}}};'.format(np.uint16(c[period-1])))

"""
Nothing to touch after this!
"""
make_cos_table(int(samp_freq/chorusfreq))