#!/usr/bin/env python3
from __future__ import print_function
import re

from odrive_interface.msg import VelocityControl
from odrive_interface.srv import *
import sys
import threading
import time
import rospy
import odrive
from odrive.enums import *
from std_msgs.msg import String

# Global variables used throughout the program
requested_state_resolved = False

def handle_change_control_mode(req: ChangeControlModeRequest):
    # error handling
    if (req.axis != 0 and req.axis != 1) or (req.requestedControlMode > 3 or req.requestedControlMode < 0):
        print("Incorrect service call to: ChangeControlMode\nNo action performed.")
        return ChangeControlModeResponse(False)
    
    global my_drive    

    # TODO: do the stuff
    
    return ChangeControlModeResponse(True)

def resolve_requested_state(axis, state):
    global my_drive, requested_state_resolved

    print("Attempting to change the current state to the requested state...\n")

    if (axis == 0):
        my_drive.axis0.requested_state = state
        # wait for the requested state to resolve
        while my_drive.axis0.current_state != state:
            time.sleep(0.1)
    else:
        my_drive.axis1.requested_state = state
        # wait for the requested state to resolve
        while my_drive.axis1.current_state != state:
            time.sleep(0.1)

    requested_state_resolved = True
    


def handle_change_state(req: ChangeStateRequest):
    # error handling
    if (req.axis != 0 and req.axis != 1) or (req.requestedState > 13 or req.requestedState < 0 or req.requestedState == 5):
        print("Incorrect service call to: ChangeState\nNo action performed.")
        return ChangeStateResponse(False)
    
    global my_drive, requested_state_resolved
    
    requested_state_resolved = False
    thread = threading.Thread(target=resolve_requested_state(req.axis, req.requestedState))
    thread.start()
    
    thread.join(timeout=10)

    if not requested_state_resolved:
        print('Requested state took too long to execute.\n')
    else:
        print('Requested state successfully resolved!\n')

    return ChangeStateResponse(requested_state_resolved)


def velocity_callback(data: VelocityControl):
    print("Received axis0 velocity: ", data.axis0_velocity)
    print("Received axis1 velocity: ", data.axis1_velocity)
    # TODO: Have ODrive try to set velocities for each axis

def setup_node():    
    rospy.init_node('odrive_interface', anonymous=True)
    rospy.Subscriber("odrive_cmd_vel", VelocityControl, velocity_callback)
    rospy.Service('change_control_mode', ChangeControlMode, handle_change_control_mode)
    rospy.Service('change_state', ChangeState, handle_change_state)
    print("odrive_interface node launched, ready to receive commands...\n")
    rospy.spin()


# try to find ODrive, if no ODrive can be found within 5 seconds, terminate program/node and notify user
print("Trying to connect to ODrive...\n")
try:
    my_drive = odrive.find_any(timeout=5)
except TimeoutError:
    print("Could not find an ODrive.\nProgram will now terminate.")
    sys.exit()
print("ODrive detected, launching odrive_interface node...\n")
        
if __name__ == '__main__':    
    setup_node()
    