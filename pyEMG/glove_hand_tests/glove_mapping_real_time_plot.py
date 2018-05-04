from __future__ import print_function, division
import numpy as np
import time
from pyEMG.cyberglove import CyberGlove
from matplotlib import pyplot as plt

calibration_file = "C:\\Users\\Agamemnon\\Documents\\Smarthand_experiment\\models\\Subject_0\\Subject_0.cal"
map_file = "C:\\Users\\Agamemnon\\Documents\\Smarthand_experiment\\models\\Subject_0\\map.csv"
gs_map = np.loadtxt(map_file, delimiter=',')
cg_n_df = 18
cg_buffered = True
cg_buf_size = 0.2
cg_port = 'COM12'

def threshold_position(position, min_value=0., max_value=1.):
    position[position < min_value] = min_value
    position[position > max_value] = max_value
    return position

def perform_min_max_calibration():
    """Min-max calibration for smarthand."""

    cg = CyberGlove(s_port='COM12',n_df=cg_n_df, buffered=False,
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

        # Index flexion
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

#        # Middle flexion
#        print("Move middle finger to extreme open position.")
#        for i in range(30):
#            if cg.buffered is True:
#                data = np.mean(np.copy(cg.data.buffer), axis=0)
#            else:
#                data = np.copy(cg.data)
#            mid = np.dot(data, gs_map)[3]
#            if mid < min_values[3]:
#                min_values[3] = mid
#            time.sleep(0.1)
#
#        print("Move middle finger to extreme closed position.")
#        for i in range(30):
#            if cg.buffered is True:
#                data = np.mean(np.copy(cg.data.buffer), axis=0)
#            else:
#                data = np.copy(cg.data)
#            mid = np.dot(data, gs_map)[3]
#            if mid > max_values[3]:
#                max_values[3] = mid
#            time.sleep(0.1)
#
#        # Ring and pinky flexion
#        print("Move ring and pinky fingers to extreme open position.")
#        for i in range(30):
#            if cg.buffered is True:
#                data = np.mean(np.copy(cg.data.buffer), axis=0)
#            else:
#                data = np.copy(cg.data)
#            rinpin = np.dot(data, gs_map)[4]
#            if rinpin < min_values[4]:
#                min_values[4] = rinpin
#            time.sleep(0.1)
#
#        print("Move ring and pinky fingers to extreme closed position.")
#        for i in range(30):
#            if cg.buffered is True:
#                data = np.mean(np.copy(cg.data.buffer), axis=0)
#            else:
#                data = np.copy(cg.data)
#            rinpin = np.dot(data, gs_map)[4]
#            if rinpin > max_values[4]:
#                max_values[4] = rinpin
#            time.sleep(0.1)

        cg.stop()
        return (min_values, max_values)

    except KeyboardInterrupt:
        cg.stop()

def get_position(cg, min_values, max_values):
    data = np.copy(cg.data.buffer)
    position = np.dot(data, gs_map)
    position = (position - min_values) / (max_values - min_values)
    #position = threshold_position(position)
    return position

def draw_position(position, ax):
    ax.plot(range(position.shape[0]), position)
    ax.set_ylim([-0.5, 1.5])
    ax.legend(labels=['Tumb rotation', 'Thumb', 'index', 'Middle', 'Ring-pinky'])
    major_ticks = np.arange(-1.5, 1.5, 0.5)
    minor_ticks = np.arange(-1.5, 1.5, 0.1)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)
    ax.grid(which='both')

#def main():

min_values, max_values = perform_min_max_calibration()

# Teleoperation
cg = CyberGlove(s_port='COM12',n_df=cg_n_df, buffered=cg_buffered,
                    buf_size=cg_buf_size, calibration_file=calibration_file)
cg.start()
print("Starting...")
# Create figure instance
plt.figure()
plt.ion() # set plot to animated
ax=plt.axes()
plt.hold(False)



try:
    while True:
        position = get_position(cg, min_values, max_values)
        draw_position(position, ax)
        plt.pause(1e-9)
except KeyboardInterrupt:
    cg.stop()
    plt.close()
    print("Exiting...")


#if __name__ == "__main__":
#    main()
