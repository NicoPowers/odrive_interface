#!/usr/bin/env python3
# license removed for brevity
import rospy
import odrive
from odrive.enums import *
from std_msgs.msg import String
from odrive_interface.msg import VelocityControl

def velocity_callback(data: VelocityControl):
    print("Received axis0 velocity: ", data.axis0_velocity)
    print("Received axis1 velocity: ", data.axis1_velocity)
    # TODO: Have ODrive try to set velocities for each axis

def setup_node():
    print("Waiting for data...")
    rospy.init_node('odrive_velocity_control', anonymous=True)
    rospy.Subscriber("odrive_cmd", VelocityControl, velocity_callback)
    rospy.spin()


def connect_to_odrive():
    my_drive = odrive.find_any()    
    print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
    
        
if __name__ == '__main__':
    # connect_to_odrive()
    setup_node()
    