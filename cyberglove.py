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
        self.s_rate = s_rate
        self.buffered = buffered
        self.buf_size = buf_size
        self.calibration_file = calibration_file
        
        self._startTime_ = None
        self._stopTime_ = None
        
        self.__exitFlag = False
        if self.n_df == 18:
            self.__bytesPerRead = 20 # First and last bytes are reserved
        elif self.n_df == 22:
            self.__bytesPerRead = 24 # First and last bytes are reserved

        if self.buffered:
            self.__buf_size_samples = int(np.ceil(self.s_rate * self.buf_size))
            self.data = Buffer((self.__buf_size_samples, self.n_df))
            self.time = Buffer((self.__buf_size_samples,))
        else:
            #self.data = np.zeros((0, self.n_df))
            self.data = np.zeros((self.n_df,))
            #self.time = np.zeros((0,))
            self.time = np.zeros((1,))
        
        if self.calibration_file is None:
            self.calibration_ = False
        else:
            self.calibration_ = True
            self.load_calibration_file()
                        
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
        self.si.flushOutput()
        self.si.flushInput()
        thread.start_new_thread(self.networking, ())
    
    def stop(self):
        """Close port."""
        self.__exitFlag = True
        time.sleep(0.1) # Wait 100 ms before closing the port just in case data are being transmitted
        if self.si.isOpen():
            self.si.flushInput()
            self.si.flushOutput()
            self.si.close()
    
    def networking(self):
        while not self.__exitFlag:
            raw_data = self.raw_measurement()
            cal_data = self.calibrate_data(raw_data)
            timestamp = np.asarray([timeit.default_timer() - self._startTime_])

            if self.buffered is True:
                self.data.push(cal_data)
                self.time.push(timestamp)
            else:
                #self.data = np.vstack((self.data, cal_data))
                self.data = cal_data
                #self.time = np.hstack((self.time, timestamp))
                self.time = timestamp
            time.sleep(1/self.s_rate) # Wait 20 ms before sending the next request
            
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
    
    def load_calibration_file(self):
        """Reads a calibration file and stores values into attributes
        calibration_offset_ and calibration_gain_. Gains are converted from 
        radians to degrees. 
        
        The Finger1_3 and Finger5_3 values are not used as they do not
        correspond to any DOFs currently implemented in the Cyberglove.
        
        There must be also a bug in how DCU stores the gain parameter for
        Finger2_3 as this is saved in the Finger1_3 field. For this reason
        the indexes are slightly different for offset and gain.
        
        TODO: test with 22-DOF CyberGlove.
        
        """
        f = open(self.calibration_file, 'r')
        lines = f.readlines()
        if self.n_df == 18:
            lines_idx_offset = [2, 3, 4, 5, 7, 8, 12, 13, 15, 17, 18, 20, 22, 23, 25, 27, 28, 29]
            lines_idx_gain = [2, 3, 4, 5, 7, 8, 12, 13, 9, 17, 18, 20, 22, 23, 25, 27, 28, 29]
        elif self.n_dx == 22:
            lines_idx_offset = [2, 3, 4, 5, 7, 8, 9, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29]
            lines_idx_gain = [2, 3, 4, 5, 7, 8, 9, 12, 13, 14, 10, 17, 18, 19, 20, 22, 23, 24, 25, 27, 28, 29]
        offset = []
        gain = []
        for line in lines_idx_offset:
            offset.append(-float(lines[line].split(' ')[6]))
        for line in lines_idx_gain:
            gain.append(float(lines[line].split(' ')[9]) * (180 / np.pi)) # Convert from radians to degrees
        self.calibration_offset_ = np.asarray(offset)
        self.calibration_gain_ = np.asarray(gain)
    
    def calibrate_data(self, data):
        """Calibrates raw data if a calibration file is provided."""
        if self.calibration_ == True:
            data = data + self.calibration_offset_
            data = data * self.calibration_gain_
            return data
        else:
            return data
        
