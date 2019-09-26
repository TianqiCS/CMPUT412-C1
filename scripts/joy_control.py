#!/usr/bin/env python
import rospy
import smach
import smach_ros
import math
import ctypes
import struct
import time
from math import copysign
from geometry_msgs.msg import Twist

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, Joy

class Run(smach.State):
    def __init__(self):
        smach.State.__init__(self)
        self.twist = Twist()
    def execute(self, userdata):
        global g_data, g_cmd_vel_pub
        scalers = [0.3,  0.7,  0.7,  -3.14, -3.14, -3.14]	
        while True:
            if (g_data):
                #### Start Mapping from Joy to Twist
                self.twist.angular.z = g_data.axes[0] * scalers[3]
                self.twist.linear.x = g_data.axes[1] * scalers[0]
                g_cmd_vel_pub.publish(self.twist)

def joy_callback(data):
    global g_data
    g_data = data

def main():
    global g_cmd_vel_pub, g_data
    rospy.init_node('joy_bot')
    g_cmd_vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=1)
    rospy.Subscriber("joy", Joy, joy_callback)
    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        smach.StateMachine.add('run', Run())
    outcome = sm.execute()
    rospy.spin()
    return

if __name__ == '__main__':
    try:
        g_data = None
        g_cmd_vel_pub = None
        main()    
    except rospy.ROSInterruptException:
        pass
        