# -*- coding: utf-8 -*-
"""
Created on Sun Apr 03 13:47:27 2016

@author: Agamemnon
"""

import pandas as pd
import numpy as np

def calibrate_glove(x, cal_fname, glove_type):
    """Perform glove data calibration.
    """

    assert glove_type == 18 or glove_type == 22, 'Unknown glove type (must be either 18 or 22).'
    idx = {'18' : np.hstack((np.arange(0,6), np.arange(8,10), np.arange(11,14),
                             np.arange(15,18), np.arange(19,23))),
            '22' : np.hstack((np.arange(0,7), np.arange(8,23)))}
        
    cal_data = pd.read_csv(cal_fname, skiprows=[0,1,6,11,16,21,26], header = None, sep = ' ')
    offset = cal_data[6].values
    gain = cal_data[9].values *(180/np.pi)
    
    offset = -offset[idx[str(glove_type)]]
    gain = gain[idx[str(glove_type)]]
    
    x = x + offset
    x = x*gain
    
    return x
    

