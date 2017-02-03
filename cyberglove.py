# Authors: Agamemnon Krasoulis <agamemnon.krasoulis@gmail.com>

from __future__ import division, print_function
import serial, serial.tools.list_ports
import numpy as np
import struct
from pyEMG.time_buffer import Buffer
import timeit
import warnings
import thread
import time

def load_calibration_file(calibration_file, n_df):
    """Reads a calibration file and returns calibration_offset and 
    calibration_gain values. Gains are converted from  radians to degrees. 
    
    The Finger1_3 and Finger5_3 values are not used as they do not
    correspond to any DOFs currently implemented in the Cyberglove.
    
    There must be also a bug in how DCU stores the gain parameter for
    Finger2_3 as this is saved in the Finger1_3 field. For this reaso,n
    the indexes are slightly different for offset and gain.
    
    TODO: test with 22-DOF CyberGlove.
    
    Parameters
    ----------
    
    calibration_file : string
        Path where Cyberglove calibration file is stored.
        
    n_df : integer (18 or 22)
        Degrees-of-freedom (DOFs) of the data glove.
    
    """
    
    f = open(calibration_file, 'r')
    lines = f.readlines()
    if n_df == 18:
        lines_idx_offset = [2, 3, 4, 5, 7, 8, 12, 13, 15, 17, 18, 20, 22, 23, 25, 27, 28, 29]
        lines_idx_gain = [2, 3, 4, 5, 7, 8, 12, 13, 9, 17, 18, 20, 22, 23, 25, 27, 28, 29]
    elif n_df == 22:
        lines_idx_offset = [2, 3, 4, 5, 7, 8, 9, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29]
        lines_idx_gain = [2, 3, 4, 5, 7, 8, 9, 12, 13, 14, 10, 17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29]
    else:
        raise ValueError("Cyberglove can have either 18 or 22 DOFs.")
    offset = []
    gain = []
    for line in lines_idx_offset:
        offset.append(-float(lines[line].split(' ')[6]))
    for line in lines_idx_gain:
        gain.append(float(lines[line].split(' ')[9]) * (180 / np.pi)) # Convert from radians to degrees
    calibration_offset = np.asarray(offset)
    calibration_gain = np.asarray(gain)
    return (calibration_offset, calibration_gain)

def calibrate_data(data, calibration_offset, calibration_gain):
    """Calibrates raw data.
    
    Parameters
    ----------
    data : array
        Raw cyberglove data.
    
    calibration_offset : array
        Sensor offsets.
    
    calibration_gain : array
        Sensor offsets.
        
    """
    
    data = data * calibration_gain
    data = data + calibration_offset
    return data
        
class CyberGlove(object):
    """Interface the Cyberglove via a serial port. 
        
        Parameters (TODO)
        ----------
        s_port : str, optional, default: None
            Serial port name (e.g., 'COM1' in Windows). If set to None, the 
            first one available will be used.
        
        baud_rate : int, optional, default: 115200
            Baud rate.
        
            
        Attributes
        ----------
        
        TODO
        
    """ 

    def __init__(self, n_df=None, s_port=None, baud_rate=115200,
                 buffered=True, buf_size=1., calibration_file=None):
        
        # If n_df is not given assume 18-DOF Cyberglove but issue warning
        if n_df == None:
            warnings.warn("Cyberglove: number of DOFs not given, assuming 18.")
            self.n_df = 18
        else:
            if n_df not in [18, 22]:
                raise ValueError("Cyberglove can have either 18 or 22 degrees-of-freedom.")
            else:
                self.n_df = n_df
            
        # if port is not given use the first one available
        if s_port == None:
            try:
                s_port = serial.tools.list_ports.comports().next()[0]
            except StopIteration:
                print("No serial ports found.")

        self.si = serial.Serial(port=None, baudrate=baud_rate, timeout=0.05, writeTimeout=0.05)
        self.si.port = s_port
        self.buffered = buffered
        self.buf_size = buf_size
        self.calibration_file = calibration_file
        
        
        self.__srate = 100 # Hardware sampling rate. TODO: Double-check this is correct
        if self.n_df == 18:
            self.__bytesPerRead = 20 # First and last bytes are reserved
        elif self.n_df == 22:
            self.__bytesPerRead = 24 # First and last bytes are reserved

        if self.buffered:
            self.__buf_size_samples = int(np.ceil(self.__srate * self.buf_size))
            self.data = Buffer((self.__buf_size_samples, self.n_df))
            self.time = Buffer((self.__buf_size_samples,))
        else:
            self.data = np.zeros((self.n_df,))
            self.time = np.zeros((1,))
        
        if self.calibration_file is None:
            self.calibration_ = False
        else:
            self.calibration_ = True
            (self.calibration_offset_, self.calibration_gain_) = load_calibration_file(calibration_file, self.n_df)
                        
    def __repr__(self):
        """TODO"""
        raise NotImplementedError
    
    def __str__(self):
        """TODO"""
        raise NotImplementedError

    def __del__(self):
        """Call stop() on destruct."""
        self.stop()
    
    def start(self):
        """Open port and perform check."""
        self.__networking = True
        self.si.open()
        self._startTime_ = timeit.default_timer()
        self.si.flushOutput()
        self.si.flushInput()
        thread.start_new_thread(self.networking, ())
    
    def stop(self):
        """Close port."""
        self.__networking = False
        time.sleep(0.1) # Wait 100 ms before closing the port just in case data are being transmitted
        self._stopTime_ = timeit.default_timer()
        if self.si.isOpen():
            self.si.flushInput()
            self.si.flushOutput()
            self.si.close()
    
    def networking(self):
        while self.__networking:
            data = self.raw_measurement()
            if self.calibration_ is True:
                data = calibrate_data(data, self.calibration_offset_, self.calibration_gain_)
            timestamp = np.asarray([timeit.default_timer()])

            if self.buffered is True:
                self.data.push(data)
                self.time.push(timestamp)
            else:
                self.data = data
                self.time = timestamp
            time.sleep(1./self.__srate) # Wait 10 ms until before sending the next command
            
    def raw_measurement(self):
        """Performs a single measurment read from device (all sensor values). 
        If this fails, it tries again.
        Returns the raw data (after reserved bytes have been removed).
        """

        fmt = '@' + "B"*self.__bytesPerRead # Format for unpacking binary data
        self.si.flushInput()
        raw_measurement = None
        while raw_measurement is None:
            nb = self.si.write(bytes('\x47'))
            if nb == 1:
                msg = self.si.read(size=self.__bytesPerRead)
                if len(msg) is self.__bytesPerRead:
                    raw_measurement = struct.unpack(fmt, msg)
                    raw_measurement = np.asarray(raw_measurement)
        return raw_measurement[1:-1] # First and last bytes are reserved
    
    
        
