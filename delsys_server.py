""" 
Delsys Trigno server implementation

Author:
unknown, https://github.com/Yakisoba007

Modified by:
Agamemnon Krasoulis
agamemnon.krasoulis@gmail.com

TODO: 
Implement DelsysStation for EMG/Acc sensors for Trigno


"""

import socket
import thread
from struct import unpack
import numpy as np
import timeit
from pyEMG.time_buffer import Buffer


class DelsysStation(object):
    '''
    Class to receive data from delsys station. Connect to station, buffer received data.
    
    Parameters
    ----------
    
    buffered : boolean
        option for buffering/appending new data
    
    host_ip : string
        IP address of the machine running the Delsys Trigno Server
    
    bufsize : float
        buffer length in seconds
        
    samplesPerPacket : int
        number of samples in each packet received from the Trigno base
    
    Attributes
    ----------
    
    data : numpy array / buffer
        list of buffered data (1st: EMG, 2nd: IMU)
    
    time : numpy array / buffer
        list of timestamps (1st: EMG, 2nd: IMU)  
    '''
    def __init__(self, buffered=True, host_ip = '127.0.0.1', bufsize = 1.,
                 samplesPerPacket = 1, imu_type=None, start_time=None):
        
        self.host = host_ip
        self.dataPort = 50043
        self.imuPort = 50044
        
        self.sdkPort = 50040
        self.emg = None
        self.imu = None
        self.sdk = None
        self.buffered = buffered
        self.bufsize = bufsize
        self.samplesPerPacket = samplesPerPacket
        self.imuType = 'raw' if imu_type is None else imu_type
        self.__numSensors = 16
        self.__emgRate = 2000
        self.__imuRate = 148.148148148148148148148148148148148148
        self.__bytesPerSample = 4
        self.__signalsPerEmgSensor = 1
        if self.imuType == 'raw':
            self.__signalsPerImuSensor = 9 
            self.__signalsPerImuSensorTransmitted = 9
        elif self.imuType == 'quat':
            self.__signalsPerImuSensor = 4
            self.__signalsPerImuSensorTransmitted = 5
        elif self.imuType == 'pry':
            self.__signalsPerImuSensor = 3
            self.__signalsPerImuSensorTransmitted = 5
        else:
            raise ValueError('Unrecognised type of IMU transmission.')
        
        self._startTime = start_time
        self._stopTime = None
        if self.buffered:
            self._emgBufSize = int(np.ceil(self.__emgRate * self.bufsize))
            self._imuBufSize = int(np.ceil(self.__imuRate * self.bufsize))
            self.data = [Buffer((self._emgBufSize, self.__numSensors)),  \
            Buffer((self._imuBufSize, self.__numSensors * self.__signalsPerImuSensor))]
            self.time = [Buffer((self._emgBufSize, 1)), \
            Buffer((self._imuBufSize, 1))]
        else:
            self.data = [np.zeros((self.__numSensors,)), \
            np.zeros((self.__numSensors * self.__signalsPerImuSensor,))]
            self.time = [np.zeros((1,)), np.zeros((1,))]
        self.exitFlag = False
        
    def start(self):
        ''' establish connection:
        sdk - port
        emg - port
        imu - port
        '''
        
        self.flush() # Reset buffer
        print "connect to " + str(self.host)
        self.sdk = socket.create_connection((self.host, self.sdkPort))
        self.imu = socket.create_connection((self.host, self.imuPort))
        self.emg = socket.create_connection((self.host, self.dataPort))
        
        print "connected"
        self.sdk.send('START\r\n\r\n')
        self.sdk.recv(1024)
        self._startTime = timeit.default_timer() if self._startTime is None else self._startTime
        thread.start_new_thread(self.networking, (self.emg, 'emg'))
        thread.start_new_thread(self.networking, (self.imu, 'imu'))
        
    def networking(self, server, mode):
        ''' receive packets of data and fill buffer '''
        if mode == 'emg':
            shp = (-1,self.__numSensors)
            recSize = self.samplesPerPacket * self.__numSensors * \
            self.__bytesPerSample * self.__signalsPerEmgSensor
            buf_index = 0
            dummy_cols = [] # No reserved bytes
        elif mode == 'imu':
            shp = (-1, self.__numSensors * self.__signalsPerImuSensorTransmitted)
            recSize = self.samplesPerPacket * self.__numSensors * \
            self.__bytesPerSample * self.__signalsPerImuSensorTransmitted
            buf_index = 1
            if self.imuType == 'raw':
                dummy_cols = [] # No reserved bytes
            elif self.imuType == 'quat':
                dummy_cols = self.__signalsPerImuSensorTransmitted * np.arange(self.__numSensors) + 4 # 5th byte is reserved
            elif self.imuType == 'pry':
                dummy_cols = np.sort(np.concatenate((self.__signalsPerImuSensorTransmitted*np.arange(self.__numSensors)+3, 
                                                     self.__signalsPerImuSensorTransmitted*np.arange(self.__numSensors)+4))) # 4th and 5th bytes are reserved
            
        while not self.exitFlag:
            data = server.recv(recSize)

            length = len(data)
            while length < recSize:
                data += server.recv(recSize-length)
                length = len(data)

            data = np.asarray(unpack('<'+'f'*(recSize/self.__bytesPerSample), data))
            data = data.reshape((shp))
            data = np.delete(data, dummy_cols, axis=1)
            timestamp = np.asarray([timeit.default_timer()])
            
            if self.buffered:
                self.data[buf_index].push(data)
                self.time[buf_index].push(np.ones((data.shape[0],1))*timestamp)
            else:
                self.data[buf_index] = data
                self.time[buf_index] = timestamp
            
    
    def stop(self):
        ''' close connections to server '''
        self.exitFlag = True
        self.sdk.send("QUIT\r\n\r\n")
        self._stopTime = timeit.default_timer() - self._startTime
        self.emg.close()
        self.sdk.close()
        self.imu.close()
        
    def flush(self):
        ''' reset buffer '''
        if self.buffered:
            self.data = [Buffer((self._emgBufSize, self.__numSensors)),  \
            Buffer((self._imuBufSize, self.__numSensors*self.__signalsPerImuSensor))]
            self.time = [Buffer((self._emgBufSize, 1)), \
            Buffer((self._imuBufSize, 1))]
        else:
             self.data = [np.zeros((self.__numSensors,)), np.zeros((self.__numSensors*self.__signalsPerImuSensor,))]   
             self.time = [np.zeros((1,)), np.zeros((1,))]