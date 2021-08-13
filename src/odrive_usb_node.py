#!/usr/bin/env python3
# license removed for brevity
import rospy
import odrive
from odrive.enums import *
from std_msgs.msg import String

def odrive_cmd_callback(data):
    print("Received command: ", data)

def setup_node():
    print("Waiting for data...")
    rospy.init_node('odrive_usb_interface', anonymous=True)
    rospy.Subscriber("odrive_cmd", String, odrive_cmd_callback)
    rospy.spin()


def connect_to_odrive():
    my_drive = odrive.find_any()    
    print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
    
        
if __name__ == '__main__':
    connect_to_odrive()
    setup_node()
    