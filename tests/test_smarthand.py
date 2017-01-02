# -*- coding: utf-8 -*-
"""
Created on Tue Nov 29 12:44:30 2016

@author: Agamemnon
"""

from __future__ import print_function, division
from pyEMG import SmartHand
import time
import numpy as np
from matplotlib import pyplot as plt

def random_position():
    pos = np.abs(np.random.randn(5))
    pos[pos>1.] = 1
    return pos

# Test 0: 
s = SmartHand(s_port='COM10')
s.start()
s.posture([0., 1., 1., 1., 1])
time.sleep(2)
s.open_all()
time.sleep(2)
s.stop()

# Test 1: set position for one finger
s = SmartHand(s_port='COM10')
s.start()
s.set_finger_pos([0.7], 1)
time.sleep(0.5)
print('finger_set_: {}, \nfinger_pos_: {}, \nget_finger_pos: {}'.format(s.finger_pos_set_, s.finger_pos_ , s.get_finger_pos()))
s.stop()

# Test 2: set positions for all fingers by using both posture() and set_finger_pos()
s = SmartHand(s_port='COM10')
s.start()
s.first_calibration()
pos = random_position()
print('Random position: {}'.format(pos))
s.posture(pos)
time.sleep(0.8)
print('finger_set_: {}, \nfinger_pos_: {}, \nget_finger_pos: {}'.format(s.finger_set_, s.finger_pos_ , s.get_finger_pos()))
time.sleep(2)
s.open_all()
time.sleep(1)
s.set_finger_pos(pos)
time.sleep(0.8)
print('finger_set_: {}, \nfinger_pos_: {}, \nget_finger_pos: {}'.format(s.finger_set_, s.finger_pos_ , s.get_finger_pos()))
s.stop()

# Test 3: finger state_ and executing_ attributes
s = SmartHand(s_port='COM10')
s.start()
finger = 2
pos = [.9]
print(s.finger_state_[finger], s.executing_)
s.set_finger_pos(pos, finger)
time.sleep(0.2)
print(s.finger_state_[finger], s.executing_)
time.sleep(0.4)
print(s.finger_state_[finger], s.executing_)
s.stop()

# Test 3b: finger state and is_executing() method (all fingers)
s = SmartHand(s_port='COM10')
s.start()
print(s.finger_state_, s.executing_)
s.set_finger_pos([1., 0., 0.3, 0.6, 0.4])
time.sleep(0.2)
print(s.finger_state_, s.executing_)
time.sleep(0.3)
print(s.finger_state_, s.executing_)
s.stop()

# Test 4: move motors
finger = 3
speed = 0.8
direction = 1
s = SmartHand(s_port='COM10')
s.start()
s.move_motor(finger=finger, direction=direction,speed=speed)
time.sleep(1)
s.stop()

# Test 5: Close all test
s = SmartHand(s_port='COM10')
s.start()
s.close_digits()
time.sleep(1)
s.open_all()
time.sleep(1)
s.stop()

# Test 6: finger_pos_
finger = 2
close_speed = 0.4
s = SmartHand(s_port='COM10')
s.start()
s.close_finger(finger, speed=close_speed)
print(s.finger_pos_)
time.sleep(0.2)
print(s.finger_pos_[finger])
time.sleep(0.4)
print(s.finger_pos_)
time.sleep(1)
print(s.finger_pos_[finger])
s.stop()

# Test 7: set current and read current
s = SmartHand(s_port='COM10')
s.start()
finger = 3
curr=[]
for i in xrange(100):
    if i == 5:
        s.set_motor_curr([0.6], motor=finger) # Current mdoe
#        s.set_motor_curr_pos([0.6], motor=finger) # Current position mode
    curr.append(s.get_motor_curr(motor=finger))
    time.sleep(0.015)
time.sleep(1)
s.open_finger(finger)
time.sleep(1)
print(s.motor_curr_set_, s.motor_curr_)
s.stop()
plt.figure()
plt.plot(np.arange(len(curr)), curr)
plt.show()

# Test 8: Read finger force
s = SmartHand(s_port='COM10')
s.start()
finger = 3
force=[]
for i in xrange(100):
    if i == 5:
        s.close_finger(finger=finger,speed=0.8)
    force.append(s.get_finger_force(finger=finger))
    time.sleep(0.015)
time.sleep(1)
s.open_finger(finger)
time.sleep(1)
print(s.finger_force_)
s.stop()
plt.figure()
plt.plot(np.arange(len(force)), force)
plt.show()