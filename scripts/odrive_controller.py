#!/usr/bin/env python3
from __future__ import print_function

from odrive_interface.msg import VelocityControl
from odrive_interface.srv import *
from odrive_class import ODrive
import sys
import threading
import time
import rospy
from odrive.enums import *
from odrive.utils import *
from fibre import Event


my_drive = None
ignore = False
terminate = False
last = None

def watchdog():
    global last, ignore, terminate, my_drive

    while(True):
        if (terminate):
            break

        if (last != None):
            now = rospy.get_rostime()            
            if ((now.to_sec() - last.to_sec()) > 5):
                ignore = True
                my_drive.disengage_motors()            

        time.sleep(0.1)

def velocity_callback(data: VelocityControl):
    print("STATUS: Received velocity for axis 0: {}".format(data.axis0_velocity))
    print("STATUS: Received velocity for axis 1: {}".format(data.axis1_velocity))        
    global my_drive, last, ignore
    
    last = rospy.get_rostime()

    if (ignore):        
        response = input("ERROR: Watchdog Timer expired.\nPress 'Enter' to reset Watchdog Timer: ")
        if (response == ""):
            my_drive.engage_motors()
            ignore = False

    if (not ignore):
        my_drive.set_velocity(0, -data.axis0_velocity)
        my_drive.set_velocity(1, data.axis1_velocity)

def setup_node():    
    rospy.init_node('odrive_interface')
    rospy.Subscriber("odrive_cmd_vel", VelocityControl, velocity_callback, queue_size=1)
    print("odrive_interface node launched, ready to receive commands...\n")

    watchdog_thread = threading.Thread(target=watchdog)
    watchdog_thread.start()

    rospy.spin()    
    global terminate
    terminate = True
      


if __name__ == '__main__':    
    try:
        
        my_drive = ODrive(watchdog_timeout=5)

        if (not my_drive.is_connected):
            sys.exit()
        
        if (my_drive.calibrate()):
            if (my_drive.engage_motors()):
                setup_node()    

    finally:
        my_drive.shutdown()        
            
    
    
    