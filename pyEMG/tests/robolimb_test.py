import numpy as np
from pyEMG import RoboLimb

""" Initialise an object and send some random grasp commands."""
robo = RoboLimb()
grasps = ['rest', 'open', 'cylindrical', 'lateral', 'tridigit', 'pointer']

for __ in range(10):
   robo.grasp(grasps[np.random.randint(len(grasps))])
   time.sleep(2)

robo.stop()
