import numpy as np
from bin_parm import BinParm
from scipy.signal import butter, lfilter, filtfilt

class Dataset(object):
    
    def __init__(self, data_dict, imu_type=None):
        self.imuType = imu_type
        if data_dict.has_key('emg'):
            self.emg = data_dict['emg']
        if data_dict.has_key('acc'):
            self.acc = data_dict['acc']
        if data_dict.has_key('gyro'):
            self.gyro = data_dict['gyro']
        if data_dict.has_key('mag'):
            self.mag = data_dict['mag']
        if data_dict.has_key('imu'):
            self.imu = data_dict['imu']
        if data_dict.has_key('stimulus'):
            self.stimulus = data_dict['stimulus']
        if data_dict.has_key('restimulus'):
            self.restimulus = data_dict['restimulus']
        if data_dict.has_key('glove'):
            self.glove = data_dict['glove']
        if data_dict.has_key('repetition'):
            self.repetition = data_dict['repetition']
        if data_dict.has_key('rerepetition'):
            self.rerepetition = data_dict['rerepetition']
        if data_dict.has_key('exercise'):
            self.exercise = data_dict['exercise']
        if data_dict.has_key('subject'):
            self.subject = data_dict['subject']
        self.electrodes = self._get_active_electrodes()
        self.sRate = {'emg':2e3, 'acc':2e3, 'gyro':2e3, 'mag':2e3, 'glove':2e3}
        
    def _get_active_electrodes(self):
        var = np.var(self.emg, axis = 0)
        active = np.where(var > 0.)
        return active
    
    def set_glove_sensors(self, sensors):
        self.glove_sensors = sensors
        self.glove = self.glove[:,sensors]
    
    def map_glove_to_hand(self, A):
        """Transform glove data into another representation. A mapping matrix A is required."""
        self.glove = np.dot(self.glove, A)
    
    def set_electrodes(self, electrodes, imu_type='quat'):
        """Select a subset of sensors."""
        self.electrodes = np.asarray(electrodes)
        if hasattr(self, 'emg'):
            self.emg = self.emg[:,electrodes]
        if self.imuType == 'raw':
            indices = np.zeros(0, dtype = int)
            for electrode in electrodes:
                indices = np.append(indices, np.arange(electrode*3,(electrode+1)*3))            
            if hasattr(self, 'acc'):
                self.acc = self.acc[:, indices]
            if hasattr(self, 'gyro'):
                self.gyro = self.gyro[:, indices]
            if hasattr(self, 'mag'):
                self.mag = self.mag[:, indices]
        elif self.imuType == 'pry':
            indices = np.zeros(0, dtype = int)
            for electrode in electrodes:
                indices = np.append(indices, np.arange(electrode*3,(electrode+1)*3))   
            self.imu = self.imu[:, indices]
        elif self.imuType == 'quat':
            indices = np.zeros(0, dtype = int)
            for electrode in electrodes:
                indices = np.append(indices, np.arange(electrode*4,(electrode+1)*4))   
            self.imu = self.imu[:, indices]

class DatasetRaw(Dataset):
    
    def __init__(self, data_dict, imu_type):
        super(DatasetRaw, self).__init__(data_dict, imu_type)
        self.sRate = {'emg':2e3, 'acc':2e3, 'gyro':2e3, 'mag':2e3, 'imu':2e3, 'glove':2e3} # TODO FIX THIS
        
    def emg_filter(self,order = 4, lowcut = 10., highcut = 500.):
        nyq = 0.5 * self.sRate['emg']
        low = lowcut/nyq
        high = highcut/nyq
        b, a = butter(order, [low, high], btype = 'band')
        self.emg = lfilter(b=b, a=a, x=self.emg, axis=0)
    
    def glove_filter(self, order = 4, highcut = 2):
        nyq = 0.5 * self.sRate['glove']
        high = highcut/nyq
        b, a = butter(N=order, Wn = high, btype = 'lowpass')
        self.glove = filtfilt(b=b, a=a, x=self.glove, axis=0)
        
class DatasetBinned(Dataset):
    
    def __init__(self, datasetraw, binparm):
        if hasattr(datasetraw, 'emg'):
            self.emg = self._bin(datasetraw.emg, binparm, datasetraw.sRate['emg'])
        if hasattr(datasetraw, 'acc'):
            self.acc = self._bin(datasetraw.acc, binparm, datasetraw.sRate['acc'])
        if hasattr(datasetraw, 'gyro'):
            self.gyro = self._bin(datasetraw.gyro, binparm, datasetraw.sRate['gyro'])
        if hasattr(datasetraw, 'mag'):
            self.mag = self._bin(datasetraw.mag, binparm, datasetraw.sRate['mag'])
        if hasattr(datasetraw, 'imu'):
            self.imu = self._bin(datasetraw.imu, binparm, datasetraw.sRate['imu'])
        if hasattr(datasetraw, 'glove'):
            self.glove = self._bin(datasetraw.glove, binparm, datasetraw.sRate['glove'])
        if hasattr(datasetraw, 'stimulus'):
            self.stimulus = self._bin_integer(datasetraw.stimulus, binparm, datasetraw.sRate['emg'])
        if hasattr(datasetraw, 'restimulus'):
            self.restimulus = self._bin_integer(datasetraw.restimulus, binparm, datasetraw.sRate['emg'])
        if hasattr(datasetraw, 'repetition'):
            self.repetition = self._bin_integer(datasetraw.repetition, binparm, datasetraw.sRate['emg'])
        if hasattr(datasetraw, 'rerepetition'):
            self.rerepetition = self._bin(datasetraw.rerepetition, binparm, datasetraw.sRate['emg'])
        if hasattr(datasetraw, 'exercise'):
            self.exercise = datasetraw.exercise
        if hasattr(datasetraw, 'subject'):
            self.subject = datasetraw.subject
        if hasattr(datasetraw, 'electrodes'):
            self.electrodes = datasetraw.electrodes
    
    def _bin(self, x, binparm, sRate):
        x = np.asarray(x)
        
        win_size = binparm.winsize*1e-3*sRate
        win_inc = binparm.wininc*1e-3*sRate
        
        num_sam, num_dim = np.shape(x)
        num_win = int(np.floor((num_sam-win_size)/win_inc))+1
        y = np.zeros((num_win,num_dim))
        
        st = 0
        en = win_size-1
        for ii in xrange(num_win):
            curwin = x[st:en,:]
            y[ii,:] = np.mean(curwin, axis = 0)
            st += win_inc
            en += win_inc
        return y
    
    def _bin_integer(self, x, binparm, sRate):
        x = np.asarray(x)
        
        win_size = binparm.winsize*1e-3*sRate
        win_inc = binparm.wininc*1e-3*sRate
        
        num_sam, num_dim = np.shape(x)
        num_win = int(np.floor((num_sam-win_size)/win_inc))+1
        y = np.zeros((num_win,num_dim))
        
        st = 0
        en = win_size-1
        for ii in xrange(num_win):
            curwin = x[st:en,:]
            num_non_zeros = np.count_nonzero(curwin)
            if num_non_zeros == 0:
                y[ii] = 0
            else:
                y[ii] = curwin[np.nonzero(curwin)[0][0]]
            st += win_inc
            en += win_inc
        return y