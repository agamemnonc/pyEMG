# -*- coding: utf-8 -*-
"""
Created on Wed Mar 30 18:28:02 2016

@author: Agamemnon
"""
import numpy as np
import time
from sklearn.preprocessing import MinMaxScaler

def interpolate_time_vector(x):
    """ Time vector interpolation for time series signal.

    x : numpy array
        Time - vector.

    """

    x = np.asarray(x)
    y = np.zeros(x.shape)
    i = 0
    while i < x.size:
        val = x[i]
        tmp = (x == val)
        num_elem = tmp.sum()
        if i + num_elem < x.size:
            next_val = x[i + num_elem]
            y[i:i+num_elem] = np.linspace(start=val, stop=next_val, num=num_elem, endpoint=False)
        else:
            step = y[i-1]-y[i-2]
            num_rem = x.size - i
            y[i:] = np.arange(start=val, stop=val +step*num_rem, step=step)
        i  += num_elem
    return y

#    x = np.asarray(x)
#    y = np.zeros_like(x)
#    y = np.linspace(start = x[0], stop = x[-1], num = x.size, endpoint = True)
#
#    return y
#

def get_number_imu_signals(imu_type):
    """Returns the number of IMU signals per sensor, depending on the
    IMU transmission configuration."""
    if imu_type == 'raw':
        signals_per_sensor = 9
    elif imu_type == 'quat':
        signals_per_sensor = 4
    elif imu_type == 'pry':
        signals_per_sensor = 3
    else:
        raise ValueError('Unrecognised IMU transmission configuration.')
    return signals_per_sensor

def get_acc_indices(sensors, signals_per_channel = 9):
    """Returns the indices for accelerometry data."""
    sensors = np.asarray(sensors)
    indices = np.zeros(0, dtype = int)
    for sens in sensors:
        indices = np.append(indices, np.arange(sens*signals_per_channel,sens*signals_per_channel + 3))
    return indices

def get_gyro_indices(sensors, signals_per_channel = 9):
    """Returns the indices for accelerometry data."""
    sensors = np.asarray(sensors)
    indices = np.zeros(0, dtype = int)
    for sens in sensors:
        indices = np.append(indices, np.arange(sens*signals_per_channel,sens*signals_per_channel + 3) + 3)
    return indices

def get_mag_indices(sensors, signals_per_channel = 9):
    """Returns the indices for accelerometry data."""
    sensors = np.asarray(sensors)
    indices = np.zeros(0, dtype = int)
    for sens in sensors:
        indices = np.append(indices, np.arange(sens*signals_per_channel,sens*signals_per_channel + 3) + 6)
    return indices

def get_imu_indices(sensors, imu_type='quat'):
    """Returns the indices for imu data.
    imu_type can be one of {'quat', 'pyr'}. """
    sensors = np.asarray(sensors)
    if imu_type == 'raw':
        signals_per_channel = 9
    elif imu_type == 'quat':
        signals_per_channel = 4
    elif imu_type == 'pyr':
        signals_per_channel = 3
    else:
        raise ValueError('Unrecognised IMU transmission type.')
    indices = np.zeros(0, dtype = int)
    for sens in sensors:
        indices = np.append(indices, np.arange(sens*signals_per_channel,(sens+1)*signals_per_channel))
    return indices

def strip_inactive(emg, imu, glove, stimulus):
    """Strips inactive samples. All inputs are pandas data frames where the
    first column is the time vector. """

    startall = 0
    endall = 1e7
    for x in [emg, imu, glove, stimulus]:
        if x.iloc[0,0] > startall: # First timestamp
            startall = x.iloc[0,0]
        if x.iloc[-1,0] < endall: # Last timestamp
            endall = x.iloc[-1,0]

    emg = emg[emg['Time'] > startall]
    imu = imu[imu['Time'] > startall]
    glove = glove[glove['Time'] > startall]
    stimulus = stimulus[stimulus['Time'] > startall]

    emg = emg[emg['Time'] < endall]
    imu = imu[imu['Time'] < endall]
    glove = glove[glove['Time'] < endall]
    stimulus = stimulus[stimulus['Time'] < endall]

    return emg, imu, glove, stimulus

def get_num_windows(datasize, sRate, winsize, wininc):
    """ Gets the total number of windows for processing data for a given stream and specified winsize and wininc."""
    winsize_samples = sRate * winsize * 1e-3
    wininc_samples = sRate * wininc * 1e-3
    return int(np.floor((datasize - winsize_samples)/wininc_samples))

def nextpow2(x):
    """ Next power of 2."""
    return 2**np.ceil(np.log2(x)).astype('int')

def ismember(a, b):
    """ Returns the indices of a which are equal to any element of b. """
    bind = {}
    for i, elt in enumerate(b):
        if elt not in bind:
            bind[elt] = i
    return np.where(np.asarray([bind.get(itm, None) for itm in a]) >= 0)[0]

def write_to_txt(outfile, array, fmt='%.18f'):
    """Write numpy array data into text files."""
    with open(outfile, 'a') as f_handle:
        np.savetxt(
        f_handle,
        array,
        fmt=fmt,             # formatting, 2 digits in this case
        delimiter=',',          # column delimiter
        newline='\n',           # new line character
        footer='',   # file footer
        comments='# ',          # character to use for comments
        header='')      # file header

    
def stimulus_presentation(subject, trial_num, n_trials=15, n_objects=3, object_dict=None):
    """ Returns stimulus presentation order in a reproducible manner."""
    np.random.seed(seed=subject) # Reproducible results

    # Check consistency
    assert(trial_num > 0 and trial_num <= n_trials)

    stimuli = np.zeros((n_trials, n_objects), dtype=int)
    for i in range(n_trials):
        stimuli[i] = np.random.choice(range(1, n_objects+1), size=(n_objects), replace=False)

    if object_dict == None:
        return stimuli[trial_num-1]
    else:
        stim_object = []
        for i in stimuli[trial_num-1]:
            stim_object.append(object_dict[i])
        return stim_object

def dump_raw_data(streamer, outfile_emg, outfile_imu, time_interval = 1, start_point = [0., 0.]):
    while not streamer.exitFlag:
        # Make copies of two arrays as they consantly get udpated
        time_copy = [np.copy(streamer.time[0].buffer), np.copy(streamer.time[1].buffer)]
        data_copy = [np.copy(streamer.data[0].buffer), np.copy(streamer.data[1].buffer)]

        # Find start and end
        idx_start = [np.where(time_copy[0] > start_point[0])[0][0], np.where(time_copy[1] > start_point[1])[0][0]]
        idx_end =   [np.where(time_copy[0] > start_point[0])[0][-1], np.where(time_copy[1] > start_point[1])[0][-1]]

        write_to_txt(outfile_emg, np.concatenate((time_copy[0][idx_start[0]:], data_copy[0][idx_start[0]:]), axis = 1))
        write_to_txt(outfile_imu, np.concatenate((time_copy[1][idx_start[1]:], data_copy[1][idx_start[1]:]), axis = 1))

        # Update new start point and wait
        start_point = [time_copy[0][idx_end[0]][0], time_copy[1][idx_end[1]][0]]
        time.sleep(time_interval)

        # Loop
        dump_raw_data(streamer=streamer, outfile_emg=outfile_emg, outfile_imu=outfile_imu, time_interval=time_interval, start_point=start_point)

class RobustMinMaxScaler(MinMaxScaler):
    """MinMaxScaler with offset."""
    def __init__(self, desired_feature_range=(0,1), offset=(0.1, 0.1), copy=True):
        super(RobustMinMaxScaler, self).__init__(feature_range=(desired_feature_range[0] - offset[0], desired_feature_range[1] + offset[1]), copy=copy)
        self.desired_feature_range = desired_feature_range
        self.offset = offset

    def transform(self, x):
        x_sc = super(RobustMinMaxScaler, self).transform(x)
        x_sc[x_sc < self.feature_range[0] + self.offset[0]] = self.feature_range[0] + self.offset[0]
        x_sc[x_sc > self.feature_range[1] - self.offset[1]] = self.feature_range[1] - self.offset[1]
        return x_sc
