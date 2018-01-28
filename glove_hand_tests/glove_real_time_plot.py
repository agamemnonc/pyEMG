from __future__ import print_function, division
import numpy as np
from matplotlib import pyplot as plt
from pyEMG.cyberglove import CyberGlove

dfs = np.asarray([1, 2, 3, 4])

calibration_file = "C:\\Users\\Agamemnon\\Documents\\Smarthand_experiment\\models\\Subject_0\\Subject_0.cal"
cg = CyberGlove(s_port='COM12',n_df=18, buffered=True, buf_size=0.2, calibration_file=calibration_file)
cg.start()

plt.ion() # set plot to animated
ax1=plt.axes() 
plt.hold(False)
ax1.grid()

try:
    while True: 
        data = np.copy(cg.data.buffer) # 0-based indexing
        plot_data = data[:,dfs-1]
        plt.plot(range(plot_data.shape[0]), plot_data)
        plt.ylim([-100, 100])
        ax1.legend(labels=dfs)
        plt.grid()
        plt.pause(1e-9)
except KeyboardInterrupt:
    cg.stop()
    plt.close()