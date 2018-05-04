"""
Teleoperate the smarthand with a Cyberglove (18-DOF version).

Mapping from Cyberglove to Smarthand DOFs has been performed empirically.


TODO: write this as a class

"""

from __future__ import print_function, division
import numpy as np
from pyEMG.smarthand import SmartHand
from pyEMG.cyberglove import CyberGlove
import threading
import time

calibration_file = "C:\\Users\\Agamemnon\\Documents\\Smarthand_experiment\\models\\Subject_0\\Subject_0.cal"
map_file = "C:\\Users\\Agamemnon\\Documents\\Smarthand_experiment\\models\\Subject_0\\map.csv"
gs_map = np.loadtxt(map_file, delimiter=',')
cg_n_df = 18
cg_buffered = False
cg_buf_size = 0.1
cg_port = 'COM5'

def threshold_position(position, min_value=0., max_value=1.):
    position[position < min_value] = min_value
    position[position > max_value] = max_value
    return position

def bin_position(position, n=20):
    """Discretises prediction in n bins."""
    return np.floor(position*n)/n

def perform_min_max_calibration():
    """Min-max calibration for smarthand."""

    cg = CyberGlove(s_port=cg_port,n_df=cg_n_df, buffered=False,
                    buf_size=cg_buf_size, calibration_file=calibration_file)
    cg.start()
    min_values = 1e3*np.ones((5,))
    max_values = -1e3*np.ones((5,))

    try:
        # Thumb rotation
        print("Move thumb rotation to extreme open position.")
        for i in range(30):
            if cg.buffered is True:
                data = np.mean(np.copy(cg.data.buffer), axis=0)
            else:
                data = np.copy(cg.data)
            th_rot = np.dot(data, gs_map)[0]
            if th_rot < min_values[0]:
                min_values[0] = th_rot
            time.sleep(0.1)

        print("Move thumb rotation to extreme closed position.")
        for i in range(30):
            if cg.buffered is True:
                data = np.mean(np.copy(cg.data.buffer), axis=0)
            else:
                data = np.copy(cg.data)
            th_rot = np.dot(data, gs_map)[0]
            if th_rot > max_values[0]:
                max_values[0] = th_rot
            time.sleep(0.1)

        # Thumb flexion
        print("Move thumb finger to extreme open position.")
        for i in range(30):
            if cg.buffered is True:
                data = np.mean(np.copy(cg.data.buffer), axis=0)
            else:
                data = np.copy(cg.data)
            th = np.dot(data, gs_map)[1]
            if th < min_values[1]:
                min_values[1] = th
            time.sleep(0.1)

        print("Move thumb finger to extreme closed position.")
        for i in range(30):
            if cg.buffered is True:
                data = np.mean(np.copy(cg.data.buffer), axis=0)
            else:
                data = np.copy(cg.data)
            th = np.dot(data, gs_map)[1]
            if th > max_values[1]:
                max_values[1] = th
            time.sleep(0.1)

        # Index/Middle/Ring-pinky flexion
        print("Move index, middle, ring and pinky fingers to extreme open position.")
        for i in range(30):
            if cg.buffered is True:
                data = np.mean(np.copy(cg.data.buffer), axis=0)
            else:
                data = np.copy(cg.data)
            for joint in [2,3,4]:
                ind = np.dot(data, gs_map)[joint]
                if ind < min_values[joint]:
                    min_values[joint] = ind
            time.sleep(0.1)

        print("Move index, middle, ring and pinky fingers to extreme closed position.")
        for i in range(30):
            if cg.buffered is True:
                data = np.mean(np.copy(cg.data.buffer), axis=0)
            else:
                data = np.copy(cg.data)
            for joint in [2,3,4]:
                ind = np.dot(data, gs_map)[joint]
                if ind > max_values[joint]:
                    max_values[joint] = ind
            time.sleep(0.1)

        cg.stop()
        return (min_values, max_values)

    except KeyboardInterrupt:
        cg.stop()


def teleoperate(cyberglove, smarthand, gs_map, min_values, max_values):
    """Reads data from glove and sets smarthand posture."""
    if cyberglove.buffered is True:
        data = np.mean(np.copy(cyberglove.data.buffer), axis=0) # 0-based indexing
    else:
        data = np.copy(cyberglove.data)
    position = np.dot(data, gs_map)
    position = (position - min_values) / (max_values - min_values)
    position = threshold_position(position)
    smarthand.set_finger_pos(position)

def main():
    min_values, max_values = perform_min_max_calibration()

    # Teleoperation
    cg = CyberGlove(s_port=cg_port,n_df=cg_n_df, buffered=cg_buffered,
                        buf_size=cg_buf_size, calibration_file=calibration_file)
    cg.start()
    s = SmartHand(s_port='COM10')
    s.start()
    try:
        while True:
            t = threading.Thread(target=teleoperate, args=(cg, s, gs_map, min_values, max_values))
            t.daemon = True
            t.start()
            time.sleep(0.02)
    except KeyboardInterrupt:
        cg.stop()
        s.open_all()
        s.stop()

if __name__ == "__main__":
    main()
