# -*- coding: utf-8 -*-
"""
Created on Sat Mar 19 15:02:44 2016

@author: Agamemnon
"""

from scipy.signal import butter, filtfilt

def glove_filter_lowpass(x, order = 4, sRate = 2000., highcut = 1.):
    """ Forward-backward band-pass filtering (IIR butterworth filter) """
    nyq = 0.5 * sRate
    high = highcut/nyq
    b, a = butter(order, high, btype = 'low')
    filtered = filtfilt(b=b, a=a, x=x, axis=0, method = 'pad', padtype = 'odd')
    return filtered