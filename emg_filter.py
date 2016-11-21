# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 15:02:44 2016

@author: Agamemnon
"""
import numpy as np
from scipy.signal import butter, filtfilt

def emg_filter_bandpass(x, order = 4, sRate = 2000., lowcut = 10., highcut = 500.):
    """ Forward-backward band-pass filtering (IIR butterworth filter) """
    nyq = 0.5 * sRate
    low = lowcut/nyq
    high = highcut/nyq
    b, a = butter(order, [low, high], btype = 'band')
    return filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd', 
                    padlen = np.minimum(3*np.maximum(len(a),len(b)), x.shape[0]-1))
    
def emg_filter_comb(x):
    """ Comb filtering at 50 Hz, for 2KHz sampling frequency. 
        Coefficients are computed with MATLAB."""
    b = np.zeros(41)
    a = np.zeros(41)
    b[0] = 0.941160767899653
    b[-1] = -0.941160767899653
    a[0] = 1.
    a[-1] = -0.882321535799305
    return filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd',
                    padlen = np.minimum(3*np.maximum(len(a),len(b)), x.shape[0]-1))