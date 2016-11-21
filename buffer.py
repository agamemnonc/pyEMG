""" 
Simple buffer implementation

Author:
Agamemnon Krasoulis
agamemnon.krasoulis@gmail.com

"""

import numpy as np

class Buffer(object):
    """Basic buffer class for data streaming.
    Adds incoming data at the end.
    Can only handle 1D or 2D numpy arrays.
    
    Parameters
    ----------
    
    size : tuple
        buffer size
        
    
    Attributes
    ----------
    
    buffer : numpy array
        buffered data
    
    
    """
    
    def __init__(self, size):
        self.size = tuple(size)
        self.buffer = np.zeros(size)
    
    def push(self, data, axis = 0):
        data = np.asarray(data)
        # Handle both 1D and 2D arrays
        if len(self.size) == 1:
            l = data.shape[0]
            self.buffer[:-l] = self.buffer[l:]
            self.buffer[-l:] = data
        if len(self.size) == 2:
            if data.ndim == 1:
                data = data[np.newaxis, :]
            l = data.shape[axis]
            if axis == 0:
                self.buffer[:-l,:] = self.buffer[l:,:]
                self.buffer[-l:,:] = data
            if axis == 1:
                self.buffer[:,:-l] = self.buffer[:,l:]
                self.buffer[:,-l:] = data