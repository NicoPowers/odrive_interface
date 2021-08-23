#!/usr/bin/env python3
from __future__ import print_function

from odrive_interface.msg import VelocityControl
from odrive_interface.srv import *
import sys
import threading
import time
import rospy
import odrive
from odrive.enums import *
from fibre import Event

# Global variables used throughout the program
requested_state_resolved = False
shutdown_token = Event()
my_drive = None

def has_errors():
    global my_drive

    if (my_drive.axis0.error > 0):
        print("Error with axis 0")
        return True
    elif (my_drive.axis1.error > 0):
        print("Error with axis 1")
        return True

    return False

def handle_change_control_mode(req: ChangeControlModeRequest):
    # error handling
    if (req.axis != 0 and req.axis != 1) or (req.requestedControlMode > 3 or req.requestedControlMode < 0):
        print("Incorrect service call parameters to: ChangeControlMode\nNo action performed.")
        return ChangeControlModeResponse(False)
    
    global my_drive 
       
    if (req.axis == 0):
        my_drive.axis0.controller.config.control_mode = req.requestedControlMode
    else:
        my_drive.axis1.controller.config.control_mode = req.requestedControlMode

    time.sleep(1.0)    
    
    return ChangeControlModeResponse(not has_errors())

def resolve_requested_state(req: ChangeStateRequest):
    global my_drive, requested_state_resolved    

    print("Attempting to change the current state to the requested state...\n")
    time.sleep(1)
    
    # we need to determine which axis this request is for, and we need to determine
    # if this is a calibration attempt or a regular state change attempt
    # a calibration attempt should return the state back to idle

    if (req.requestedState == 0):
        req.requestedState = 1
        
    if (req.axis == 0):
        my_drive.axis0.requested_state = req.requestedState
        if (req.isCalibration):
            while my_drive.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
        else:            
            while my_drive.axis0.current_state != req.requestedState:
                time.sleep(0.1)
    else:        
        my_drive.axis1.requested_state = req.requestedState        
        if (req.isCalibration):
            while my_drive.axis1.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
        else:
            while my_drive.axis1.current_state != req.requestedState:
                time.sleep(0.1)

    # read for any errors to determine if the requested calibration or state changed worked out ok    
    requested_state_resolved = not has_errors()
    

def handle_change_state(req: ChangeStateRequest):
    # error handling
    if (req.axis != 0 and req.axis != 1) or (req.requestedState > 13 or req.requestedState < 0 or req.requestedState == 5):
        print("Incorrect service call parameters to: ChangeState\nNo action performed.")
        return ChangeStateResponse(False)
    
    global my_drive, requested_state_resolved
    print(my_drive)
    # resolve the request to change the state in another thread so we can set a timeout for it
    requested_state_resolved = False
    thread = threading.Thread(target=resolve_requested_state, args=(req, ))
    thread.start()
    thread.join(timeout=10)

    # check if thread timed out or not
    if (thread.isAlive()):
        print("Requested state could not resolve in time")
        return ChangeStateResponse(False)

    else:
        if requested_state_resolved:
            print('Requested state successfully resolved!\n')    
        else:
            print('Requested state NOT successfully resolved!\n')

    return ChangeStateResponse(requested_state_resolved)


def velocity_callback(data: VelocityControl):
    print("Received axis0 velocity: ", data.axis0_velocity)
    print("Received axis1 velocity: ", data.axis1_velocity)
    # TODO: Have ODrive try to set velocities for each axis
    
    global my_drive
    my_drive.axis0.controller.input_vel = data.axis0_velocity
    my_drive.axis1.controller.input_vel = data.axis1_velocity

def setup_node():    
    rospy.init_node('odrive_interface')
    rospy.Subscriber("odrive_cmd_vel", VelocityControl, velocity_callback)
    rospy.Service('change_control_mode', ChangeControlMode, handle_change_control_mode)
    rospy.Service('change_state', ChangeState, handle_change_state)
    print("odrive_interface node launched, ready to receive commands...\n")
    rospy.spin()

def calibration_routine():
    # start full motor calibration sequence
    
    req = ChangeStateRequest()
    req.axis = 0
    req.requestedState = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    req.isCalibration = True
    print("Attempting to calibrate motor 0")
    res = handle_change_state(req)
    if (res.success):
        req.axis = 1
        req.requestedState = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("Attempting to calibrate motor 1")
        res = handle_change_state(req)
        if (not res.success):
            return False
    else:
        return False

    return True

def release_motors():
    global my_drive
    # start full motor calibration sequence
    req = ChangeStateRequest()
    req.axis = 0
    req.requestedState = AXIS_STATE_IDLE
    req.isCalibration = False
    res = handle_change_state(req)
    if (res.success):
        req.axis = 1
        req.requestedState = AXIS_STATE_IDLE
        res = handle_change_state(req)
        if (not res.success):
            return False
    else:
        return False

    return True



        
if __name__ == '__main__':    
    try:
        # try to find ODrive, if no ODrive can be found within 5 seconds, terminate program/node and notify user
        print("Trying to find an ODrive...\n")
        my_drive = odrive.find_any(timeout=5, channel_termination_token=shutdown_token)
        print("ODrive detected, launching odrive_interface node...\n")    
        if (calibration_routine()):
            setup_node()
        shutdown_token.set()
    except TimeoutError:
        print("Could not find an ODrive.")    
    
    finally:
        release_motors()
        sys.exit()    
    
    
    