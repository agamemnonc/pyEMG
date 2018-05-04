""" 
Myoelectric data binning parameters class

Author:
Agamemnon Krasoulis
agamemnon.krasoulis@gmail.com

"""

class BinParm(object):
    '''
    Myoelectric data binning parameters class
    
    Parameters
    ----------
    
    winsize : float
        window size (in ms)
    
    wininc : float
        window increment/shift (in ms)
    
    '''
    def __init__(self, winsize, wininc):
        self.winsize = float(winsize)
        self.wininc = float(wininc)