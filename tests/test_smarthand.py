# -*- coding: utf-8 -*-
"""
Created on Tue Nov 29 12:44:30 2016

@author: Agamemnon
"""

from __future__ import print_function, division
from pyEMG import SmartHand
import time
import numpy as np

def random_position():
    pos = np.abs(np.random.randn(5))
    pos[pos>1.] = 1
    return pos

# Test 1: set position for one finger
s = SmartHand()
s.start()
s.set_finger_pos([0.9], 1)
time.sleep(0.5)
print('finger_set_: {}, \nfinger_pos_: {}, \nget_finger_pos: {}'.format(s.finger_set_, s.finger_pos_ , s.get_finger_pos()))
s.stop()

# Test 2: set positions for all fingers by using both posture() and set_finger_pos()
s = SmartHand()
s.start()
pos = random_position()
print('Random position: {}'.format(pos))
s.posture(pos)
time.sleep(0.5)
print('finger_set_: {}, \nfinger_pos_: {}, \nget_finger_pos: {}'.format(s.finger_set_, s.finger_pos_ , s.get_finger_pos()))
time.sleep(2)
s.open_all()
time.sleep(1)
s.set_finger_pos(pos)
time.sleep(0.5)
print('finger_set_: {}, \nfinger_pos_: {}, \nget_finger_pos: {}'.format(s.finger_set_, s.finger_pos_ , s.get_finger_pos()))
s.stop()

# Test 3: finger state and is_executing() method
s = SmartHand()
s.start()
finger = 3
pos = [.9]
s.set_finger_pos(pos, finger)
print(s.get_finger_status(finger), s.is_executing())
time.sleep(1)
print(s.get_finger_status(finger), s.is_executing())
s.stop()

# Test 4: move motors
finger = 3
speed = 0.8
direction = 0
s = SmartHand()
s.start()
s.move_motor(finger=finger, direction=direction,speed=speed)
time.sleep(1)
s.stop()
