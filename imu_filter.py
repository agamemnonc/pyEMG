# -*- coding: utf-8 -*-
"""
Created on Thu Mar 31 14:29:55 2016

@author: Agamemnon
"""

import numpy as np
from scipy.signal import butter, filtfilt

def imu_filter_lowpass(x, order = 4, sRate = 148.148148148148, highcut = 20.0):
    """ Forward-backward band-pass filtering (IIR butterworth filter) """
    nyq = 0.5 * sRate
    high = highcut/nyq
    b, a = butter(N =order, Wn = high, btype = 'low')
    return filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd',
                    padlen = np.minimum(3*len(a)*len(b), x.shape[0]-1))
    
def imu_filter_highpass(x, order = 4, sRate = 148.148148148148, lowcut = 0.01):
    """ Forward-backward band-pass filtering (IIR butterworth filter) """
    nyq = 0.5 * sRate
    low = lowcut/nyq
    b, a = butter(N =order, Wn = low, btype = 'high')
    return filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd',
                    padlen = np.minimum(3*len(a)*len(b), x.shape[0]-1))
    
def imu_filter_bandpass(x, order = 4, sRate = 148.148148148148, lowcut = 1., highcut = 20.):
    """ Forward-backward band-pass filtering (IIR butterworth filter) """
    nyq = 0.5 * sRate
    low = lowcut/nyq
    high = highcut/nyq
    b, a = butter(N =order, Wn = [low, high], btype = 'band')
    return filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd',
                    padlen = np.minimum(3*len(a)*len(b), x.shape[0]-1))
    
def imu_filter_comb(x):
    """ Comb filtering at 50 Hz. Coefficients are computed with MATLAB."""
    b = np.zeros(41)
    a = np.zeros(41)
    b[0] = 0.941160767899653
    b[-1] = -0.941160767899653
    a[0] = 1.
    a[-1] = -0.882321535799305
    return filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd',
    padlen = np.minimum(3*len(a)*len(b), x.shape[0]-1))