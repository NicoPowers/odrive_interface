#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist 

def cmd_vel_callback(data):
    print("Received data: ", data)

def setup_node():
    print("Waiting for data...")
    rospy.init_node('odrive_node', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    rospy.spin()        

if __name__ == '__main__':    
    setup_node()
    