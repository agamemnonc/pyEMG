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
            """

    def __init__(self, s_port=None, b_rate=115200, n_df=5, settings=None):
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

        self.finger_pos_set_ = np.zeros(n_df, dtype=float) # Set finger positions
        self.finger_force_set_ = np.zeros(n_df, dtype=float) # Set finger forces
        self.motor_current_set_ = np.zeros(n_df, dtype=float) # Set finger currents
        self.pose_ = None
    
    @property
    def finger_state_(self):
        """Finger states attribute. """
        return self.get_finger_state()
            
    @property
    def finger_pos_(self):
        """Finger positions attribute. """
        return self.get_finger_pos()
    
    @property
    def finger_force_(self):
        """Finger forces (tendon tension force sensors). """
        return self.get_finger_force()

    @property
    def motor_curr_(self):
        """Finger positions attribute. """
        return self.get_motor_current()
        
    @property
    def executing_(self):
        """Finger moving attribute."""
        return self.__is_executing()

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

        self.si.write(bytes('\x46'))
        time.sleep(2.0)

    def first_calibration(self):
        """Mandatory calibration procedure after mechanical variations on the hand."""

        self.si.write(bytes('\x42'))
        time.sleep(10.0)

    def get_finger_state(self, finger=None):
        """Argument finger: use None to read out all n_df states,
        or a number between 0 and n_df-1 to read out a single state.
        Any read out finger states will be updated in the objects
        array finger_state_.
        Returns an array of the updated finger state(s)

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

        state = []
        for f in ifingers:
            nb = self.si.write(bytearray(('\x4B', f)))
            if nb == 2:
                p = self.si.read()
                if p != '':
                    hex_value = struct.unpack('@B', p)[0] 
                    bin_value = bin(int(str(hex_value), 16))[2:].zfill(8)
                    moving_flag = int(bin_value[-1])
                    if moving_flag:
                        state.append('moving')
                    else:
                        state.append('stop')

        return state
        
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

        pos = []
        for f in ifingers:
            nb = self.si.write(bytearray(('\x45', f)))
            if nb == 2:
                p = self.si.read()
                if p != '':
                    pos.append(float(struct.unpack('@B', p)[0] / 255.0))
        return pos


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

        if nb.count(3) == len(nb):
            self.finger_pos_set_[ifingers] = pos_array

            
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
            
        byte_seq = "1" + str(S) + "{0:04b}".format(finger) + str(0) +  "{0:09b}".format(int(speed * 511)) # 9 bits --> 512 vaules
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
        self.si.write(bytearray('\x4C')) # OpenALL command
        
    def open_all(self):
        """ Resets all DOAs to open position."""
        
        self.si.write(bytearray('\x4C')) # OpenALL command
        self.open_finger(finger=0)
    
    def close_digits(self):
        """ Sets all DOFs except thumb rotation to closed position."""
        
        for finger in range(1, self.n_df):
            self.close_finger(finger)
            
    def close_all(self):
        """ Sets all DOFs to closed position."""
        
        for finger in range(self.n_df):
            self.close_finger(finger)
    
    def set_motor_curr(self, curr_array, motor=None):
        """ Sets all DOFs to desired currents.
        
        cur_array - array of currents (floats between 0.0 and 1.0)
        finger - single integer between 0 and n_df-1 (expects a single value in
                pos_array); None sets all n_df currents and expects a corresponding
                size of pos_array
                
        """

        if motor == None:
            imotors = range(self.n_df)
        else:
            imotors = [motor,]
            
        curr_array = np.asarray(curr_array)        
        
        nb = []
        for m, curr in zip(imotors, curr_array):
            curr = int(curr*1023) # 10-bit encoding
            byte_1, byte_2 = self.__int_to_two_byte_int(curr)
            nb.append(self.si.write(bytearray(('\x5F', m, '\x61', int(byte_1,2), int(byte_2,2), m))))

        if nb.count(6) == len(nb):
            self.motor_current_set_[imotors] = curr_array
    
    def get_motor_curr(self, motor=None):
        """Argument finger: use None to read out all n_df currents,
        or a number between 0 and n_df-1 to read out a single current.
        Returns an array of the finger current(s)

        Values for 'finger' identify:
        0 Thumb ab-/adduction
        1 Thumb flexion/extension
        2 Index finger flexion/extension
        3 Middle finger flexion/extension
        4 Ring+little finger flexion/extension
        
        """

        if motor == None:
            imotors = range(self.n_df)
        else:
            imotors = [motor,]

        curr = []
        for m in imotors:
            nb = self.si.write(bytearray(('\x49', m)))
            if nb == 2:
                p = self.si.read(size=2)
                if p != '':
                    byte_1, byte_2 = struct.unpack('@BB', p)
                    curr_m = self.__two_byte_int_to_int(byte_1, byte_2) / 1023.
                    curr.append(curr_m)
                    
        return curr
                               
    def get_finger_force(self, finger=None):
        """Argument finger: use None to read out all n_df forces,
        or a number between 0 and n_df-1 to read out a single force.
        Returns an array of the finger force(s)

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

        force = []
        for f in ifingers:
            nb = self.si.write(bytes(f))
            if nb == 1:
                p = self.si.read(size=2)
                if p != '':
                    byte_1, byte_2 = struct.unpack('@BB', p)
                    force_f = self.__two_byte_int_to_int(byte_1, byte_2) / 1023.
                    force.append(force_f)
                    
        return force
    
    def set_finger_force(self, force_array, finger=None):
        """Argument finger: use None to set all n_df forces,
        or a number between 0 and n_df-1 to set a single force.

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

        force_array = self.__ignore_inf_nan(force_array) # Ignore nan's and inf's
        
        nb = []
        for f, force in zip(ifingers, force_array):
            f_bin = "{0:b}".format(f).zfill(6)
            force_bin="{0:b}".format(force).zfill(10)
            T9, T8 = force_bin[0], force_bin[1]
            byte_1 = '\x4A'
            byte_2 = struct.pack('@B', int(T9 + T8 + f_bin,2))
            byte_3 = struct.pack('@B', int(f_bin[2:],2))
            nb.append(self.si.write(bytearray((byte_1, byte_2, byte_3))))

        if nb.count(3) == len(nb):
            self.finger_force_set_[ifingers] = force_array
        
    def posture(self, pos_array):
        """ Sets all DOFs to desired position. """
        pos_array = np.asarray(pos_array)
        pos_array = self.__ignore_inf_nan(pos_array) # Ignore nan's and inf's
        
        nb = self.si.write(bytearray(('\x48', int(pos_array[0]*255), int(pos_array[1]*255), 
                                 int(pos_array[2]*255), int(pos_array[3]*255), 
                                 int(pos_array[4]*255), '\x48')))
        if nb == 7:
            self.finger_pos_set_ = pos_array

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
        self.si.write(bytearray(('\x6f', grasp_code, grasp_force, grasp_steps)))
        self.pose_ = grasp_name
    
    def __is_executing(self):
        """Returns true if at least one df is moving."""
        return True if 'moving' in self.finger_state_ else False
    
    def __int_to_two_byte_int(self, integer):
        """Converts an integer value into two integer-formatted bytes."""
        bin_format = "{0:b}".format(integer).zfill(16)
        byte_1 = bin_format[:8]
        byte_2 = bin_format[8:]
        
        return byte_1, byte_2
    
    def __two_byte_int_to_int(self, byte_1, byte_2):
        """Converts two integer-formatted bytes into an integer."""
        byte_1_bin = "{0:b}".format(byte_1).zfill(8)
        byte_2_bin = "{0:b}".format(byte_2).zfill(8)
        bytes_combined = byte_1_bin + byte_2_bin
        
        return int(bytes_combined, 2)
    
    def __ignore_inf_nan(self, pos_array):
        """Ignore nan or inf values when setting position or posture. """
        
        nan_idx = np.where(np.isnan(pos_array))
        inf_idx =  np.where(np.isinf(pos_array))
        if nan_idx[0].size > 0:
            pos_array[nan_idx] = self.get_finger_pos()[nan_idx]
        if inf_idx[0].size > 0:
            pos_array[inf_idx] = self.get_finger_pos()[inf_idx]
            
        return pos_array
