# Authors: Agamemnon Krasoulis <agamemnon.krasoulis@gmail.com>
# TODO: add support for multi-dimensional buffers
# TODO: commit and add to imports

from __future__ import division, print_function
import numpy as np
from pyEMG.time_buffer import Buffer

class MovingAverage(object):
    """Moving average smoother. 
    
    Parameters
    ----------
    
    shape : tuple
        Shape of signal to be smoothed.
    
    k : integer
        Number of windows to use for smoothing.
        
    weights : array, optional (default ones), shape = (k,)
        Weight vector. First element corresponds to weight for most recent
        observation.
        
    """
    
    def __init__(self, shape, k, weights=None):
        
        self.shape = shape
        self.__check_k_weights(k, weights)
        self.k = k 
        self.weights = np.ones((k,)) if weights is None else weights
        self.buffer_ = Buffer(size=self.__get_buf_size())
        
    def smooth(self, x):
        
        """Smooths data in x.
        
        Parameters
        ----------
        
        x : array
            Most recent raw measurement.
        
        Returns
        -------
        
        x_smoothed : array
            Smoothed measurement.
            
        """
        
        self.buffer_.push(x)
        return np.dot(np.flipud(self.weights), self.buffer_.buffer) / self.k
    
    def __check_k_weights(self, k, weights):
        
        if not isinstance(k, int):
            raise ValueError("k must be integer.")
            
        if weights is not None:
            weights = np.asarray(weights)
            if weights.ndim != 1 or weights.size != k:
                raise ValueError("Weight array must have shape ({},)".format(k))

    def __get_buf_size(self):
        """Returns a flattened tuple to be used as the buffer size."""
        
        return tuple(np.append(self.k, 
                               np.asarray([element for element in self.shape])))
        

class ExponentialSmoothing(object):
    
    def __init__(self, shape, alpha):
        
        self.shape = tuple(shape)
        self.__check_smoothing_parameter(alpha)
        self.alpha = alpha
        self.current_ = np.zeros(shape)
        self.previous_ = np.zeros(shape)
    
    def smooth(self, x):
        
        # Smoothing
        self.current_ = self.alpha * x + (1-self.alpha)*self.previous_
        
        # Update
        self.previous_ = self.current_
        
        return self.current_
    
    def __check_smoothing_parameter(self, parameter):
        if (parameter < 0. or parameter > 1.):
            raise ValueError("alpha must be between 0 and 1.")

class DoubleExponentialSmoothing(object):
    
    def __init__(self, shape, alpha, beta):
        
        self.shape = tuple(shape)
        [self.__check_smoothing_parameter(parameter) for parameter in [alpha, beta]]
        self.alpha = alpha
        self.beta = beta
        self.current_ = np.zeros(shape)
        self.previous_ = np.zeros(shape)
        self.current_b_ = np.zeros(shape)
        self.previous_b_ = np.zeros(shape)
    
    def smooth(self, x):
        
        # Smoothing
        self.current_ = self.alpha * x + (1-self.alpha)*(self.previous_ + self.previous_b_)
        self.current_b_ = self.beta*(self.current_ - self.previous_) + (1-self.beta)*self.previous_b_
        
        # Update
        self.previous_ = self.current_
        self.previous_b_ = self.current_b_
        
        return self.current_
    
    def __check_smoothing_parameter(self, parameter):
        if (parameter < 0. or parameter > 1.):
            raise ValueError("alpha must be between 0 and 1.")
            
    

        
    
