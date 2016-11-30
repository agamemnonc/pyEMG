"""Interface the SmartHand of the BioRobotics Institute of the Scuola
Superiore Sant' Anna (Pontedera/Pisa) via serial port."""
from __future__ import division
import serial, serial.tools.list_ports
import numpy as np
import struct
import time


class SmartHand(object):
    """Handle some high level commands to the robotic hand via serial interface
    and read out finger positions.
    TODO:   Implement __repr__, __str__
            Update self.finger_pos after each update. 
            """

    def __init__(self, s_port='COM10', b_rate=115200, n_df=5, settings=None):
        """Arguments:
        s_port - String argument containing serial port name (e.g., 'COM1' in Windows)
                 If set to None, tries to find available ports and takes the first one.
        b_rate - Baud rate
        n_df - degrees of freedom of the robotic hand."""

        # if port is not given try to find out yourself
        if s_port == None:
            try:
                s_port = serial.tools.list_ports.comports().next()[0]
            except StopIteration:
                print "No serial ports found."

        self.si = serial.Serial(port=None, baudrate=b_rate, timeout=0.05, writeTimeout=0.05)
        self.si.port = s_port
        
        self.n_df = n_df

        self.finger_status_ = np.chararray((n_df,), itemsize=6)
        self.finger_status_[:] = 'steady'
        self.finger_pos_ = np.zeros(n_df, dtype=float) # Finger positions
        self.finger_set_ = np.zeros(n_df, dtype=float) # Desired finger positions

        self.pose_ = None
        self.__executing = False


    def start(self):
        """Open port and perform fast calibration."""
        self.si.open()
        self.fast_calibration()

    def __del__(self):
        """Call stop_all() on destruct."""
        self.stop()

    def fast_calibration(self):
        """Should always be done once the hand is switched on and should be used.
        Will be called in constructor. Fingers will completely open."""

        nb = self.si.write(bytes('\x46'))
        time.sleep(2.0)
        if nb == 1:
            self.finger_pos_ = self.get_finger_pos()

    def first_calibration(self):
        """Mandatory calibration procedure after mechanical variations on the hand."""

        nb = self.si.write(bytes('\x42'))
        time.sleep(10.0)
        if nb == 1:
            self.finger_pos_ = self.get_finger_pos()

    def get_finger_status(self, finger=None):
        """Argument finger: use None to read out all n_df statuses,
        or a number between 0 and n_df-1 to read out a single status.
        Any read out finger statuses will be updated in the objects
        array finger_status_.
        Returns an array of the updated finger status(es)

        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension
        """

        if finger == None:
            ifingers = range(self.n_df)
        else:
            ifingers = [finger,]

        for f in ifingers:
            nb = self.si.write(bytearray(('\x4B', f)))
            if nb == 2:
                p = self.si.read()
                if p != '':
                    hex_value = struct.unpack('@B', p)[0] 
                    bin_value = bin(int(str(hex_value), 16))[2:].zfill(8)
                    moving_flag = int(bin_value[-1])
                    if moving_flag:
                        self.finger_status_[f] = 'moving'
                    else:
                        self.finger_status_[f] = 'steady'

        return self.finger_status_[ifingers]
        
    def get_finger_pos(self, finger=None):
        """Argument finger: use None to read out all n_df positions,
        or a number between 0 and n_df-1 to read out a single position.
        Any read out finger positions will be updated in the objects
        array finger_pos_.
        Returns an array of the updated finger position(s)

        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension

        Return values of 0 refer to opened/extended state,
        1 to closed/flexed state."""

        if finger == None:
            ifingers = range(self.n_df)
        else:
            ifingers = [finger,]

        for f in ifingers:
            nb = self.si.write(bytearray(('\x45', f)))
            if nb == 2:
                p = self.si.read()
                if p != '':
                    self.finger_pos_[f] = float(struct.unpack('@B', p)[0] / 255.0)

        return self.finger_pos_[ifingers]


    def set_finger_pos(self, pos_array, finger=None):
        """Transmits a command to set new finger positions and updates the
        setpoint variable finger_set of the object.
        pos_array - array of positions (floats between 0.0 and 1.0)
        finger - single integer between 0 and n_df-1 (expects a single value in
                pos_array); None sets all n_df positions and expects a corresponding
                size of pos_array
        Returns True if transmission through serial port was successful and False otherwise.

        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension

        Position values of 0 refer to opened/extended state,
        1 to closed/flexed state."""
        

        if finger == None:
            ifingers = range(self.n_df)
        else:
            ifingers = [finger,]

        pos_array = self.__ignore_inf_nan(pos_array) # Ignore nan's and inf's
        
        nb = []
        for f, pos in zip(ifingers, pos_array):
            nb.append(self.si.write(bytearray(('\x44', f, int(pos * 255.0)))))

        self.finger_pos_ = self.get_finger_pos()
        if nb.count(3) == len(nb):
            self.finger_set_[ifingers] = pos_array
            return True
        else:
            return False
            
    def move_motor(self, finger, direction, speed=1.):
        """Moves a DOA at specified direction and speed.
        
        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension
        
        Direction can be either binary (0/1) or string ("open"/"close").
        Speed is in the range 0 (no movement) to 1 (full speed).
        """
        if isinstance(direction, str):
            if direction == 'close':
                S = 1
            elif direction == 'open':
                S = 0
            else:
                raise ValueError('Unrecognized direction')
        else:
            S = direction
            
        byte_seq = "1" + str(S) + "{0:04b}".format(finger) + str(0) +  "{0:04b}".format(int(speed * 511)) # 9 bits --> 512 vaules
        byte_1, byte_2 = int(byte_seq[:8],2), int(byte_seq[8:],2)
        self.si.write(bytearray((byte_1, byte_2))) 
    
    def open_finger(self, finger, speed=1.):
        """Opens a DOA at specified speed.
        
        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension
        
        Speed is in the range 0 (no movement) to 1 (full speed).
        """
        
        self.move_motor(finger, direction="open", speed=speed)
    
    def close_finger(self, finger, speed=1.):
        """Closes a DOA at specified speed.
        
        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension
        
        Speed is in the range 0 (no movement) to 1 (full speed).
        """
        
        self.move_motor(finger, direction="close", speed=speed)
    
        
    def open_digits(self):
        """ Resets all DOAs except thumb rotation to open position."""
        nb = self.si.write(bytearray('\x4C')) # OpenALL command
        if nb == 1:
            self.finger_pos_ = self.get_finger_pos()
        
    def open_all(self):
        """ Resets all DOAs to open position."""
        
        nb = []
        nb.append(self.si.write(bytearray('\x4C'))) # OpenALL command
        nb.append(self.set_finger_pos([0.0], finger = 0))
        if nb.count(1) == len(nb):
            self.finger_pos_ = self.get_finger_pos()
        
    def close_all(self):
        """ Sets all DOFs to closed position."""
        
        nb = self.set_finger_pos_(np.asarray([1.0, 1.0, 1.0, 1.0, 1.0]), finger = None)
        if nb == True:
            self.finger_pos_ = self.get_finger_pos()

    def posture(self, pos_array):
        """ Sets all DOFs to desired position. """
        pos_array = np.asarray(pos_array)
        pos_array = self.__ignore_inf_nan(pos_array) # Ignore nan's and inf's
        
        nb = self.si.write(bytearray(('\x48', int(pos_array[0]*255), int(pos_array[1]*255), 
                                 int(pos_array[2]*255), int(pos_array[3]*255), 
                                 int(pos_array[4]*255), '\x48')))
        if nb == 7:
            self.finger_set_ = pos_array
        self.finger_pos_ = self.get_finger_pos()
        

    def stop_all(self):
        """Stop all robot hand movement"""
        self.si.write(bytes('\x41'))
        
    def stop(self):
        """Stop all robot hand movement and control and close port"""
        if self.si.isOpen():
            self.stop_all()
            self.si.close()
    
    def grasp(self, grasp_name, grasp_force = 200, grasp_steps = 20):
        
        # If input is string, get the grasp code
        if isinstance(grasp_name, str):
            grasp_dict = {'cylindrical' : 4, 'lateral' : 1, 'tridigit' : 31,
                          'bidigit' : 2, 'tridigit_ext' : 31, 
                          'bidigit_ext' : 21, 'buffet' : 11, 'three' : 6, 
                          'pistol' : 7, 'thumb_up' : 8, 'relax' : 0}
            try:
                grasp_code = grasp_dict[grasp_name]
            except KeyError:
                print 'Unrecognised grasp name.'        
        else:
            grasp_code = grasp_name
        nb = self.si.write(bytearray(('\x6f', grasp_code, grasp_force, grasp_steps)))
        if nb == 4:
           self.finger_pos_ = self.get_finger_pos()
        self.pose_ = grasp_name
    
    def is_executing(self):
        """Update the private attribute __executing based on finger_status_"""
        if 'moving' in self.finger_status_:
            self.__executing = True
        else:
            self.__executing = False
        return self.__executing
           
    def __ignore_inf_nan(self, pos_array):
        """Ignore nan or inf commands when setting position or posture. """
        
        nan_idx = np.where(np.isnan(pos_array))
        inf_idx =  np.where(np.isinf(pos_array))
        if nan_idx[0].size > 0:
            pos_array[nan_idx] = self.get_finger_pos()[nan_idx]
        if inf_idx[0].size > 0:
            pos_array[inf_idx] = self.get_finger_pos()[inf_idx]
        
        return pos_array
            
