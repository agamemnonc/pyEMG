"""Interface the Touch Bionics Robo-limb hand via a can bus interface (Peak-can USB).
"""

import numpy as np
import time
import threading
from can.interfaces import pcan
from pyEMG.time_repeater import TimerRepeater

# Define some useful dictionaries (clf Robo-limb manual)
finger_dict = {'thumb' : 1, 'index' : 2, 'middle' : 3, 'ring' : 4,
               'little' : 5, 'rotator' : 6} # Robolimb DOFs
action_dict = {'stop' : 0, 'close' : 1, 'open' : 2} # DOF action
status_dict = {0 : 'stop', 1 : 'closing', 2 : 'opening', 3 : 'stalled close', 4 : 'stalled open'} # DOF status

class RoboLimb(object):
    """ Robo-limb hand control via can bus interface.

    Parameters
    ----------
    def_vel : int
        Default velocity for finger control

    read_rate : float
        Update rate for incoming CAN messages

    channel : pcan definition
        CAN communication channel

    b_rate : pcan definition
        CAN baud rate

    hw_type : pcan definition
        CAN hardware type

    io_port : hex
        CAN input-output port

    interrupt : int
        CAN interrupt handler

    Attributes
    ----------
    finger_status : list
        Finger status

    finger_current : numpy array
        Finger currents

    __moving : boolean
        Flag indicating whether at least one finger is moving

    __executing_grasp : boolean
        Flag indicating whether a movement command is being executed
    """

    def __init__(self,  def_vel=297, read_rate=0.02, channel=pcan.PCAN_USBBUS1, b_rate=pcan.PCAN_BAUD_1M, hw_type=pcan.PCAN_TYPE_ISA, io_port=0x3BC, interrupt=3):
        """Class constructor."""
        self.channel = channel
        self.b_rate = b_rate
        self.hw_type = hw_type
        self.io_port = io_port
        self.interrupt = interrupt
        self.def_vel = def_vel
        self.read_rate = read_rate
        self.finger_status = [None]*6
        self.finger_current = np.zeros(6)
        self.msg_read = TimerRepeater('msgRead', self.read_rate, self.__read_messages)
        self.pose = None
        self.__moving = False
        self.__executing_grasp = False

    def start(self):
        """Starts the connection."""
        self.bus = pcan.PCANBasic()
        self.bus.Initialize(Channel=self.channel, Btr0Btr1=self.b_rate, HwType=self.hw_type, IOPort=self.io_port, Interrupt=self.interrupt)
        self.msg_read.start()
        time.sleep(1)
        self.open_all(force=True)
        time.sleep(1.5)

    def stop(self):
        """Stops reading incoming CAN messages and shuts down the connection."""
        self.msg_read.stop()
        self.bus.Uninitialize(Channel=self.channel)

    def __read_messages(self):
        """Reads at least one time the queue looking for messages. If a
        message is found, looks again until queue is empty or an error occurs.
        """
        stsResult = 0
        while not (stsResult & pcan.PCAN_ERROR_QRCVEMPTY):
            can_msg = self.bus.Read(self.channel)
            if can_msg[0] == pcan.PCAN_ERROR_OK:
                self.__process_message(can_msg)
            stsResult = can_msg[0]

    def __process_message(self, can_msg):
        """Processes an incoming CAN message and updates finger_status and
        finger_current attributes.
        """
        finger_id = self._get_read_id(hex(can_msg[1].ID)) # Get finger ID
        self.finger_status[finger_id-1] = status_dict[can_msg[1].DATA[1]] # Update finger status (0-based indexing)
        self.__moving = bool(len(set(self.finger_status) & {'opening', 'closing'})) # Update __moving flag
        current_hex = str(can_msg[1].DATA[2])+str(can_msg[1].DATA[3]) # Get finger current
        self.finger_current[finger_id-1] = int(current_hex, 16) / 21.825 # Update finger current (mA)

    def open_finger(self, finger, velocity=None, force=True):
        """Opens single digit at specified velocity."""
        velocity = self.def_vel if velocity == None else int(velocity)
        finger = finger_dict[finger] if type(finger) == str else int(finger)
        if self.finger_status[finger-1] in ['opening', 'stalled open'] and force is False:
            pass
        else:
            CANMsg = pcan.TPCANMsg()
            CANMsg.ID = self._get_send_id(finger)
            CANMsg.LEN = 4
            CANMsg.MSGTYPE = pcan.PCAN_MESSAGE_STANDARD
            msg = self.__get_message(finger, action='open', velocity=velocity)
            for i in range(CANMsg.LEN):
                CANMsg.DATA[i] = int(msg[i],16)
            self.bus.Write(self.channel,CANMsg)

    def close_finger(self, finger, velocity=None, force=True):
        """Closes single digit at specified velocity."""
        velocity = self.def_vel if velocity == None else int(velocity)
        finger = finger_dict[finger] if type(finger) == str else int(finger)
        if self.finger_status[finger-1] in ['closing', 'stalled close'] and force is False:
            pass
        else:
            CANMsg = pcan.TPCANMsg()
            CANMsg.ID = self._get_send_id(finger)
            CANMsg.LEN = 4
            CANMsg.MSGTYPE = pcan.PCAN_MESSAGE_STANDARD
            msg = self.__get_message(finger, action='close', velocity=velocity)
            for i in range(CANMsg.LEN):
                CANMsg.DATA[i] = int(msg[i],16)
            self.bus.Write(self.channel,CANMsg)

    def stop_finger(self, finger, force=True):
        """Stops execution of digit movement."""
        finger = finger_dict[finger] if type(finger) == str else int(finger)
        if self.finger_status[finger-1] is 'stop' and force is False:
            pass
        elif self.finger_status[finger-1] in ['stalled open', 'stalled closed'] and force is False:
            self.finger_status[finger-1] = 'stop'
        else:
            CANMsg = pcan.TPCANMsg()
            CANMsg.ID = self._get_send_id(finger)
            CANMsg.LEN = 4
            CANMsg.MSGTYPE = pcan.PCAN_MESSAGE_STANDARD
            msg = self.__get_message(finger, action='stop', velocity=290)
            for i in range(CANMsg.LEN):
                CANMsg.DATA[i] = int(msg[i],16)
            self.bus.Write(self.channel,CANMsg)

    def open_fingers(self, velocity=None, force=True):
        """Opens all digits at specified velocity."""
        velocity = self.def_vel if velocity == None else int(velocity)
        [self.open_finger(i, velocity=velocity, force=force) for i in range(1,6)]


    def open_all(self, velocity=None, force=True):
        """Opens all digits and thumb rotator at specified velocity."""
        velocity = self.def_vel if velocity == None else int(velocity)
        self.open_fingers(velocity=velocity, force=force)
        time.sleep(0.5)
        self.open_finger(6, velocity=velocity, force=force)


    def close_fingers(self, velocity=None, force=True):
        """Closes all digits at specified velocity."""
        velocity = self.def_vel if velocity == None else int(velocity)
        [self.close_finger(i, velocity=velocity, force=force) for i in range(1,6)]

    def close_all(self, velocity=None, force=True):
        """Closes all digits and thumb rotator at specified velocity."""
        velocity = self.def_vel if velocity == None else int(velocity)
        self.close_finger(6, velocity=velocity, force=force)
        time.sleep(0.5)
        self.close_fingers(velocity=velocity, force=force)

    def stop_fingers(self, velocity=None, force=True):
        """Stops execution of movement for all digits."""
        velocity = self.def_vel if velocity == None else int(velocity)
        [self.stop_finger(i, force=force) for i in range(1,6)]

    def stop_all(self, velocity=None, force=True):
        """Stops execution of movement for all digits and thumb rotator."""
        velocity = self.def_vel if velocity == None else int(velocity)
        [self.stop_finger(i, force=force) for i in range(1,7)]

    def grasp(self, grasp_name, force=True, print_action=False):
        """Initiates a new thread to perform a grasp movement. This is done
        in order to avoid program execution while time.sleep() commands are
        used for grasp execution (pre-grasp/closing).
        """
        if grasp_name == 'rest' or grasp_name == None:
            pass
        else:
            if force is False and self.__executing_grasp is True:
                if print_action is True:
                    print("Currently executing, skpping command...")
            else:
                if print_action is True:
                    print("Executing " + grasp_name + " grasp...")
                threading.Thread(target=self.__execute_grasp, args=(grasp_name,)).start()

    def __execute_grasp(self, grasp_name):
        """Performs grasp movement at full velocity."""
        self.__executing_grasp = True    # Update execution flag
        velocity = 297
        self.stop_all()
        if grasp_name == 'open':
            self.open_fingers(velocity=velocity)
            time.sleep(1)
            self.pose = 'open'
        elif grasp_name == 'cylindrical':
            # Pre-grasp
            [self.open_finger(i, velocity=velocity) for i in range(1,6)]
            time.sleep(0.2)
            self.close_finger(6, velocity=velocity)
            time.sleep(1.3)
            # Grasp
            self.stop_all()
            self.close_fingers(velocity=velocity, force=True)
            time.sleep(1)
            self.pose = 'cylindrical'
        elif grasp_name == 'lateral':
            # Pre-grasp
            [self.open_finger(i, velocity=velocity) for i in range(1,4)]
            time.sleep(0.2)
            #self.open_finger(1, velocity = velocity)
            #time.sleep(0.1)
            self.open_finger(6, velocity=velocity, force=True)
            time.sleep(0.1)
            [self.stop_finger(i) for i in range(2,4)]
            [self.close_finger(i, velocity=velocity) for i in range(2,6)]
            time.sleep(1.2)
            # Grasp
            self.stop_all()
            self.close_finger(1, velocity=velocity, force=True)
            time.sleep(1)
            self.pose = 'lateral'
        elif grasp_name == 'tridigit':
            # Pre-grasp
            [self.open_finger(i, velocity=velocity) for i in range(1,4)]
            time.sleep(0.1)
            [self.stop_finger(i) for i in range(1,4)]
            [self.close_finger(i, velocity=velocity) for i in range(4,7)]
            time.sleep(1.4)
            # Grasp
            self.stop_all()
            [self.close_finger(i, velocity=velocity, force=True) for i in range(1,4)]
            time.sleep(1)
            self.pose = 'tridigit'
        elif grasp_name == 'tridigit_ext':
            # Pre-grasp
            self.open_fingers(velocity=velocity)
            time.sleep(0.1)
            self.stop_fingers()
            self.close_finger(6, velocity=velocity)
            time.sleep(1.4)
            # Grasp
            self.stop_all()
            [self.close_finger(i, velocity=velocity, force=True) for i in range(1,4)]
            time.sleep(1)
            self.pose = 'tridigit_ext'
        elif grasp_name == 'bidigit':
            # Pre-grasp
            [self.open_finger(i, velocity=velocity) for i in range(1,3)]
            time.sleep(0.1)
            [self.close_finger(i, velocity=velocity) for i in range(3,7)]
            time.sleep(1.3)
            self.stop_finger(6)
            self.open_finger(6)
            time.sleep(0.1)
            self.stop_finger(6)
            # Grasp
            self.stop_all()
            [self.close_finger(i, velocity=velocity, force=True) for i in range(1,4)]
            time.sleep(1)
            self.pose = 'bidigit'
        elif grasp_name == 'bidigit_ext':
            # Pre-grasp
            [self.open_finger(i, velocity=velocity) for i in range(1,6)]
            time.sleep(0.1)
            self.close_finger(6, velocity=velocity)
            time.sleep(1.3)
            self.stop_finger(6)
            self.open_finger(6)
            time.sleep(0.1)
            self.stop_finger(6)
            # Grasp
            self.stop_all()
            [self.close_finger(i, velocity=velocity, force=True) for i in range(1,3)]
            time.sleep(1)
            self.pose = 'bidigit_ext'
        elif grasp_name == 'pointer':
            # Pre-grasp
            [self.open_finger(i, velocity=velocity) for i in range(1,3)]
            time.sleep(0.1)
            self.open_finger(6, velocity=velocity)
            time.sleep(1.4)
            # Grasp
            self.stop_all()
            [self.close_finger(i, velocity=velocity, force=True) for i in [1,3,4,5]]
            time.sleep(1)
            self.pose = 'pointer'
        elif grasp_name == 'thumbs_up':
            # Pre-grasp
            self.open_finger(6, velocity=velocity)
            time.sleep(0.1)
            [self.close_finger(i, velocity=velocity) for i in range(1,6)]
            time.sleep(1.4)
            # Grasp
            self.stop_all()
            self.open_finger(1, velocity=velocity, force=True)
            time.sleep(1)
            self.pose = 'thumbs_up'
        else:
            print("Unrecognized grasp, skipping...")

        self.__executing_grasp = False


    def __get_message(self, finger, action, velocity):
        """Converts a CAN message to appropriate format."""
        action = action_dict[action] if type(action) == str else action
        velocity = format(velocity, '04x')
        msg = [0]*4
        msg[0] = '00' # Empty
        msg[1] = str(action)
        msg[2] = velocity[0:2]
        msg[3] = velocity[2:4]
        return msg

    def _get_send_id(self,finger):
        """Returns the ID for sending a CAN message to specified finger."""
        return  int('0x10' + str(finger),16)

    def _get_read_id(self,id_string):
        """Returns the finger index from a corresponding CAN ID when reading a message."""
        return int(id_string[4])

    def is_executing(self):
        """Read the value of private attribute self.__executing_grasp."""
        return self.__executing_grasp

    def is_moving(self):
        """Read the value of private attribute self.__moving."""
        return self.__moving
