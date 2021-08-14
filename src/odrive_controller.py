#!/usr/bin/env python3
from __future__ import print_function

from odrive_interface.msg import VelocityControl
from odrive_interface.srv import *
import sys
import rospy
import odrive
from odrive.enums import *
from std_msgs.msg import String


def handle_change_control_mode(req: ChangeControlModeRequest):
    # error handling
    if (req.axis != 0 and req.axis != 1) or (req.requestedControlMode > 3 or req.requestedControlMode < 0):
        print("Incorrect service call to: ChangeControlMode\nNo action performed.")
        return ChangeControlModeResponse(False)
    
    global my_drive    

    # TODO: do the stuff
    
    return ChangeControlModeResponse(True)


def handle_change_state(req: ChangeStateRequest):
    # error handling
    if (req.axis != 0 and req.axis != 1) or (req.requestedState > 13 or req.requestedState < 0 or req.requestedState == 5):
        print("Incorrect service call to: ChangeState\nNo action performed.")
        return ChangeStateResponse(False)
    
    global my_drive

    # TODO: do the stuff
    if (req.axis == 0):
        my_drive.axis0.requested_state = req.requestedState 
    
    return ChangeStateResponse(True)


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
    