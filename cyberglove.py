# Authors: Agamemnon Krasoulis <agamemnon.krasoulis@gmail.com>

from __future__ import division, print_function
import serial, serial.tools.list_ports
import numpy as np
import struct
from pyEMG import Buffer
import timeit
import warnings
import thread
import time

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

    def __init__(self, n_df=None, s_port=None, baud_rate=115200, s_rate=30, 
                 buffered=True, buf_size=1.):
        
        # if n_df is not given assume 18-DOF Cyberglove but issue warning
        if n_df == None:
            warnings.warn("Cyberglove: number of DOFs not given, assuming 18.")
            self.n_df = 18
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
        self.s_rate = s_rate
        self.buffered = buffered
        self.buf_size = buf_size
        
        self._startTime_ = None
        self._stopTime_ = None
        
        self.__exitFlag = False
        self.__bytesPerRead = 20 # First and last bytes are reserved

        if self.buffered:
            self.__buf_size_samples = int(np.ceil(self.s_rate * self.buf_size))
            self.data = Buffer((self.__buf_size_samples, self.n_df))
            self.time = Buffer((self.__buf_size_samples,))
        else:
            self.data = np.zeros((0, self.n_df))
            self.time = np.zeros((0,))
        
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
        self.si.open()
        self._startTime_ = timeit.default_timer()
        self.si.flushInput()
        self.si.flushOutput()
        thread.start_new_thread(self.networking, ())
    
    def stop(self):
        """Close port."""
        self.__exitFlag = True
        time.sleep(0.1) # Wait 100 ms before closing the port just in case requests are being transmitted
        if self.si.isOpen():
            self.si.flushInput()
            self.si.flushOutput()
            self.si.close()
    
    def networking(self):
        while not self.__exitFlag:
            raw_data = self.raw_measurement()
            timestamp = np.asarray([timeit.default_timer() - self._startTime_])

            if self.buffered is True:
                self.data.push(raw_data)
                self.time.push(timestamp)
            else:
                self.data = np.vstack((self.data, raw_data))
                self.time = np.hstack((self.time, timestamp))
            time.sleep(0.02) # Wait until sending the next request
            
    def raw_measurement(self):
        """Performs a single measurment read from device (all sensor values). 
        If this fails, it tries again.
        Returns the raw data (after reserved bytes have been removed).
        """

        fmt = '@' + "B"*self.__bytesPerRead # Format for unpacking binary data
        raw_measurement = None
        while raw_measurement is None:
            nb = self.si.write(bytes('\x47'))
            if nb == 1:
                msg = self.si.read(size=self.__bytesPerRead)
                if len(msg) is self.__bytesPerRead:
                    raw_measurement = struct.unpack(fmt, msg)
                    raw_measurement = np.asarray(raw_measurement)
        return raw_measurement[1:-1] # First and last bytes are reserved
        
