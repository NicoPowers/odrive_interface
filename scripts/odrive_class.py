#!/usr/bin/env python3

from re import T
import odrive
from odrive.utils import *
from odrive.enums import *
import threading
from time import sleep
from fibre import Event


class ODrive:
    is_connected = False

    __connected_odrive = None
    __disconnect_token = Event()
    __is_calibrated = False
    __is_engaged = False    

    def __init__(self):
        try:
            # try to find ODrive, if no ODrive can be found within 5 seconds, notify user
            print("STATUS: Trying to find an ODrive...\n")
            self.__connected_odrive = odrive.find_any(timeout=5, channel_termination_token=self.__disconnect_token)
            print("STATUS: ODrive detected, launching odrive_interface node...\n")
            
            if (self.__has_errors()):
                print("STATUS: ODrive module has errors, going to clear errors...\n")
                dump_errors(self.__connected_odrive)
                self.clear_errors()            

            self.is_connected = True

        except TimeoutError:
            print("ERROR: Could not find an ODrive.\n")
            self.is_connected = False                        


    def __has_errors(self):
        if (self.__connected_odrive.axis0.error > 0 or self.__connected_odrive.axis1.error > 0 or self.__connected_odrive.error > 0):        
            return True
        
        return False

    def clear_errors(self):
        print("STATUS: Trying to clear errors for ODrive...\n")
        self.__connected_odrive.clear_errors()


    def __wait_for_calibration(self):
        while (self.__connected_odrive.axis0.current_state != AXIS_STATE_IDLE) or (self.__connected_odrive.axis1.current_state != AXIS_STATE_IDLE):
                time.sleep(0.1)

    def __change_control_mode(self, axis, requested_control_mode):
        
        if (axis == 0):
            self.__connected_odrive.axis0.controller.config.control_mode = requested_control_mode
        elif (axis == 1):
            self.__connected_odrive.axis1.controller.config.control_mode = requested_control_mode
        else:
            print("ERROR: Incorrect axis specified: {}\n".format(axis))
            return False

        # small delay to allow changes take place, but it may not be needed
        sleep(0.50)

        if (axis == 0):
            if (self.__connected_odrive.axis0.controller.config.control_mode != requested_control_mode):
                print("ERROR: Requested control mode change for axis 0 could not resolve. \n")                                
        else:
            if (self.__connected_odrive.axis1.controller.config.control_mode != requested_control_mode):
                print("ERROR: Requested control mode change for axis 1 could not resolve. \n")
                dump_errors(self.__connected_odrive)                
        
        if (self.__has_errors()):
            dump_errors(self.__connected_odrive)
            return False
        
        return True


    def __change_state(self, axis, requested_state):
        
        if (axis == 0):
            self.__connected_odrive.axis0.requested_state = requested_state
        elif (axis == 1):
            self.__connected_odrive.axis1.requested_state = requested_state
        else:
            print("ERROR: Incorrect axis specified: {}\n".format(axis))
            return False

        # small delay to allow changes take place, but it may not be needed
        sleep(0.50)

        if (axis == 0):
            if (self.__connected_odrive.axis0.current_state != requested_state):
                print("ERROR: Requested state change for axis 0 could not resolve. \n")                
                
        else:
            if (self.__connected_odrive.axis1.current_state != requested_state):
                print("ERROR: Requested state change for axis 1 could not resolve. \n")                
                
        
        if (self.__has_errors()):
            dump_errors(self.__connected_odrive)
            return False
        
        return True
        

    def disconnect(self):
        # stop robot if moving
        self.disengage_motors()

        # release USB control from ODrive
        self.__disconnect_token.set()

        self.is_connected = False

    def set_velocity(self, axis, velocity):
        if (self.__is_engaged):

            if (axis == 0):

                self.__connected_odrive.axis0.controller.input_vel = velocity

            elif (axis == 1):
                                    
                self.__connected_odrive.axis1.controller.input_vel = velocity
            else:
                print("ERROR: Incorrect axis specified: {}\n".format(axis))
                return False
            
            if (self.__has_errors()):
                print("ERROR: Errors setting velocity on axis {}.\n".format(axis))
                dump_errors(self.__connected_odrive)
                return False
            
            return True                
        
        else:
            print("ERROR: Motors have not been engaged yet.\n")
            return False


    def calibrate(self):
        if (self.__connected_odrive.axis0.motor.is_calibrated and self.__connected_odrive.axis1.motor.is_calibrated and self.__connected_odrive.axis0.encoder.index_found and self.__connected_odrive.axis1.encoder.index_found):
            print("STATUS: ODrive already calibrated.\n")

        else:           
                        
            # start full motor calibration sequence
            print("STATUS: Beginning full calibration sequence...\n")
            
            self.__connected_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.__connected_odrive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            # calibrated can take a while, so wait around 60 seconds max for both motors to be fully calibrated
            # each axis will return to idle state after calibration
            calibration_thread = threading.Thread(target=self.__wait_for_calibration)
            calibration_thread.start()
            calibration_thread.join(timeout=60)

            # check if thread timed out or not
            if (calibration_thread.isAlive()):
                print("ERROR: Calibration could not resolve in time.\n")
                dump_errors(self.__connected_odrive)
                return False
        
        self.__is_calibrated = True
        return True
        

    def engage_motors(self):
        
        # make sure robot doesn't start running away as soon as we engage the motors
        self.__connected_odrive.axis0.controller.input_vel = 0
        self.__connected_odrive.axis1.controller.input_vel = 0

        print("STATUS: Trying to engage motors...\n")

        # try to change state to closed loop control mode
        if (self.__is_calibrated):
            if (self.__change_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL) and self.__change_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL)):
                if (self.__change_control_mode(0, CONTROL_MODE_VELOCITY_CONTROL) and self.__change_control_mode(1, CONTROL_MODE_VELOCITY_CONTROL)):
                    print("STATUS: Motors are engaged.\n")
                    self.__is_engaged = True
                    return True
        else:
            print("ERROR: Motors have not been calibrated yet.\n")

        print("ERROR: Motors failed to engage.\n")
        dump_errors(self.__connected_odrive)
            
        return False

    def disengage_motors(self):

        # setting velocity to 0 before changing state to idle may help reduce errors in overspeed and unknown torque
        self.set_velocity(0, 0)
        self.set_velocity(1, 0)

        # disengage power from motors
        self.__change_state(0, AXIS_STATE_IDLE)
        self.__change_state(1, AXIS_STATE_IDLE)
        self.__is_engaged = False
        


        