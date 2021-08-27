#!/usr/bin/env python3
from __future__ import print_function

from odrive_interface.msg import VelocityControl
from odrive_interface.srv import *
from odrive_class import ODrive
import sys
import threading
import time
from odrive_tune import engage_motors
import rospy
import odrive
from odrive.enums import *
from odrive.utils import *
from fibre import Event


my_drive = None


def velocity_callback(data: VelocityControl):
    print("STATUS: Received velocity for axis 0: {}".format(data.axis0_velocity))
    print("STATUS: Received velocity for axis 1: {}".format(data.axis1_velocity))    
    
    global my_drive
    my_drive.set_velocity(0, -data.axis0_velocity)
    my_drive.set_velocity(1, data.axis1_velocity)

def setup_node():    
    rospy.init_node('odrive_interface')
    rospy.Subscriber("odrive_cmd_vel", VelocityControl, velocity_callback)
    print("odrive_interface node launched, ready to receive commands...\n")
    rospy.spin()


if __name__ == '__main__':    
    try:
        # try to find ODrive, if no ODrive can be found within 5 seconds, terminate program/node and notify user
        print("STATUS: Trying to find an ODrive...\n")
        my_drive = ODrive(timeout=5)
        print("STATUS: ODrive detected, launching odrive_interface node...\n")    
        if (ODrive.calibrate()):
            if (ODrive.engage_motors()):
                setup_node()       

    except TimeoutError:
        print("ERROR: Could not find an ODrive.\n")    
    finally:
        ODrive.shutdown()
        sys.exit()    
    
    
    