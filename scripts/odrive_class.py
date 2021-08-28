#!/usr/bin/env python3

import odrive
from odrive.utils import *
from odrive.enums import *
import threading
from time import sleep
from fibre import Event


class ODrive:
    is_connected = False

    __connected_odrive = None
    __watchdog_timeout = 10
    __watchdog_enabled = False
    __shutdown_token = Event()
    __is_calibrated = False
    __is_engaged = False    

    def __init__(self, watchdog_timeout=5):
        try:
            # try to find ODrive, if no ODrive can be found within 5 seconds, notify user
            print("STATUS: Trying to find an ODrive...\n")
            self.__connected_odrive = odrive.find_any(timeout=5, channel_termination_token=self.__shutdown_token)
            print("STATUS: ODrive detected, launching odrive_interface node...\n")
            
            if (self.__has_errors()):
                print("STATUS: ODrive module has errors, going to clear errors...\n")
                dump_errors(self.__connected_odrive)
                self.__clear_errors()
                if (self.__has_errors()):
                    print("ERROR: Clearing errors did not working, rebooting Odrive...")
                    self.__connected_odrive.reboot()
            

            # set watchdog timer timeout
            # self.__watchdog_timeout = watchdog_timeout
            # self.__connected_odrive.axis0.config.watchdog_timeout = self.__watchdog_timeout
            # self.__connected_odrive.axis1.config.watchdog_timeout = self.__watchdog_timeout

            self.is_connected = True

        except TimeoutError:
            print("ERROR: Could not find an ODrive.\n")
            self.is_connected = False                        


    def __has_errors(self):

        if (self.__connected_odrive.axis0.error > 0 or self.__connected_odrive.axis1.error > 0 ):        
            return True
        
        return False

    def __clear_errors(self):
        self.__connected_odrive.axis0.config.enable_watchdog = False
        self.__connected_odrive.axis1.config.enable_watchdog = False
        self.__connected_odrive.clear_errors()
        self.__connected_odrive.axis0.error = 0
        self.__connected_odrive.axis1.error = 0
        self.__connected_odrive.error = 0
        self.__watchdog_enabled = False


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
        sleep(0.25)

        if (axis == 0):
            if (self.__connected_odrive.axis0.controller.config.control_mode != requested_control_mode):
                print("ERROR: Requested control mode change for axis 0 could not resolve. \n")
                dump_errors(self.__connected_odrive)
                return False
        else:
            if (self.__connected_odrive.axis1.controller.config.control_mode != requested_control_mode):
                print("ERROR: Requested control mode change for axis 1 could not resolve. \n")
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
        sleep(0.25)

        if (axis == 0):
            if (self.__connected_odrive.axis0.current_state != requested_state):
                print("ERROR: Requested state change for axis 0 could not resolve. \n")
                dump_errors(self.__connected_odrive)
                return False
        else:
            if (self.__connected_odrive.axis1.current_state != requested_state):
                print("ERROR: Requested state change for axis 1 could not resolve. \n")
                dump_errors(self.__connected_odrive)
                return False
        
        return True

    def __handle_response(self, axis, response):
                
        if (response == 'Y'):
            print("STATUS: Resetting Watchdog Timer on axis {}".format(axis))  
            self.__clear_errors()            
            return True

        elif (response == 'N'):
            print("STATUS: You chose to not re-enable the Watchdog Timer.\n")
            return False
        
        else:
            print("ERROR: Incorrect option: {}".format(response))
            return False
        

    def shutdown(self):
        # stop robot if moving
        self.disengage_motors()

        # release USB control from ODrive
        self.__shutdown_token.set()

    def set_velocity(self, axis, velocity):
        if (self.__is_engaged):
            # if (not self.__watchdog_enabled):
            #     self.__connected_odrive.axis0.config.enable_watchdog = True
            #     self.__connected_odrive.axis1.config.enable_watchdog = True
            #     self.__watchdog_enabled = True

            if (axis == 0):
                # if(self.__connected_odrive.axis0.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED):
                #     response = input("ERROR: Watchdog Timer on axis 0 expired, cannot move axis 0.\n Do you want to reset the Watchdog Timer on axis 0? (Y/N): ")                    
                #     if(self.__handle_response(0, response)):
                #         self.__connected_odrive.axis0.controller.input_vel = velocity
                #         self.__connected_odrive.axis0.watchdog_feed()
                # else:    
                #     self.__connected_odrive.axis0.controller.input_vel = velocity
                #     self.__connected_odrive.axis0.watchdog_feed()

                self.__connected_odrive.axis0.controller.input_vel = velocity

            elif (axis == 1):
                # if(self.__connected_odrive.axis1.error == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED):
                #     response = input("ERROR: Watchdog Timer on axis 1 expired, cannot move axis 1.\n Do you want to reset the Watchdog Timer on axis 1? (Y/N): ")                    
                #     if(self.__handle_response(1, response)):
                #         self.__connected_odrive.axis1.controller.input_vel = velocity
                #         self.__connected_odrive.axis1.watchdog_feed()
                # else:    
                #     self.__connected_odrive.axis1.controller.input_vel = velocity
                #     self.__connected_odrive.axis1.watchdog_feed()
                                    
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
        
        # make sure robot doesn't start running away as soon as we engage them
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

        self.__change_state(0, AXIS_STATE_IDLE)
        self.__change_state(1, AXIS_STATE_IDLE)
        self.__is_engaged = False
        


        