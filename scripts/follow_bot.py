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

## Util Function

def approxEqual(a, b, tol = 0.02):
    return abs(a - b) <= tol
##

##call_back functions

def joy_callback(joy_msg):
    global g_running
    if joy_msg.buttons[5] == 1:
        g_running = not g_running

def filterColor(point):
    rgb_packed = point[3]

    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f' ,rgb_packed)
    i = struct.unpack('>l',s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    pt_r = (pack & 0x00FF0000)>> 16
    pt_g = (pack & 0x0000FF00)>> 8
    pt_b = (pack & 0x000000FF)

    return approxEqual(pt_r, 0, 20) and approxEqual(pt_g, 0, 20) and approxEqual(pt_b, 0, 20)

def PC2_callback(msg):
    global g_x, g_z, g_std_dist
    
    # Initialize the centroid coordinates point count
    x = z = n = 0

    # Read in the x, y, z coordinates of all points in the cloud
    for point in filter(filterColor,point_cloud2.read_points(msg, skip_nans=True)):
    #for point in point_cloud2.read_points(msg, skip_nans=True)
        pt_x = point[0]
        #pt_y = point[1]
        pt_z = point[2]

        # rgb_packed = point[3] 
        # # cast float32 to int so that bitwise operations are possible
        # s = struct.pack('>f' ,rgb_packed)
        # i = struct.unpack('>l',s)[0]
        # # you can get back the float value by the inverse operations
        # pack = ctypes.c_uint32(i).value
        # pt_r = (pack & 0x00FF0000)>> 16
        # pt_g = (pack & 0x0000FF00)>> 8
        # pt_b = (pack & 0x000000FF)
        # print pt_x, pt_z, pt_r, pt_g, pt_b
        #rospy.loginfo('Point: x: {0}, z{2}, r:{3}, g:{4}, b:{5}'.format(pt_x, pt_z, pt_r, pt_g, pt_b))



        x += pt_x
        #y += pt_y
        z += pt_z
        n += 1

    if n > 0:
        x /= n
        #y /= n
        z /= n

        g_x = x
        #g_centroid['y'] = y
        g_z = z
    else:
        g_x = 0
        g_z = 0

##

## State Class

class Wait(smach.State):
    
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['start'])

    def execute(self, userdata):
        global g_running
        while not g_running:
            pass
        return 'start'

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['follow'])
        self.move_cmd = Twist()
        self.move_cmd.angular.z = 3.0

    def execute(self, userdata):
        global g_cmd_vel_pub


        while g_z == 0:
            g_cmd_vel_pub.publish(self.move_cmd)
        return 'follow'    
        
        

class Follow(smach.State):
    
    def __init__(self):

        smach.State.__init__(self, 
                             outcomes=['wait', 'search'])

        self.move_cmd = Twist()
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)

        self.z_sec_threshold = rospy.get_param("~z_sec_threshold", 1.3)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight x-displacement of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 2.5)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 1.0)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
        
        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)

    def execute(self, userdata):

        global g_cmd_vel_pub, g_std_dist, g_running, g_x, g_z

        while g_running:
            if g_x!=0 or g_z!=0:
                # Check our movement thresholds
                if (abs(g_z - g_std_dist) > self.z_threshold):
                    if (abs(g_z - g_std_dist) > self.z_sec_threshold):
                        linear_speed = (g_z - g_std_dist) * self.z_scale * 2
                    else:
                        # Compute the angular component of the movement
                        linear_speed = (g_z - g_std_dist) * self.z_scale
                    
                    # Make sure we meet our min/max specifications
                    self.move_cmd.linear.x = copysign(max(self.min_linear_speed, 
                                            min(self.max_linear_speed, abs(linear_speed))), linear_speed)
                else:
                    self.move_cmd.linear.x *= self.slow_down_factor
                    
                if (abs(g_x) > self.x_threshold):     
                    # Compute the linear component of the movement
                    angular_speed = -g_x * self.x_scale
                    
                    # Make sure we meet our min/max specifications
                    self.move_cmd.angular.z = copysign(max(self.min_angular_speed, 
                                            min(self.max_angular_speed, abs(angular_speed))), angular_speed)
                else:
                    # Stop the rotation smoothly
                    self.move_cmd.angular.z *= self.slow_down_factor
                

            else:
                return 'search'
                # Stop the robot smoothly
                #self.move_cmd.linear.x *= self.slow_down_factor
                #self.move_cmd.angular.z *= self.slow_down_factor

            #print g_centroid
            #print g_x, g_z
            g_cmd_vel_pub.publish(self.move_cmd)

        return 'wait'


##

def main():
    global g_cmd_vel_pub, g_std_dist, g_running

    g_std_dist = 0.8
    g_running = True

    rospy.init_node('follow_bot')

    g_cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber("point_cloud", PointCloud2, PC2_callback, queue_size=1)
    rospy.loginfo("Subscribing to point cloud...")
    
    # Wait for the pointcloud topic to become available
    rospy.wait_for_message('point_cloud', PointCloud2)

    rospy.loginfo("Ready to follow!")

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        smach.StateMachine.add('Wait', Wait(),
                                transitions = {'start':'Search'})
        smach.StateMachine.add('Search', Search(),
                                transitions = {'follow':'Follow'})
        smach.StateMachine.add('Follow', Follow(),
                                transitions = {'wait':'Wait',
                                               'search': 'Search'})  

    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    
    return

if __name__ == '__main__':
    try:
        g_cmd_vel_pub = None
        g_std_dist = None
        g_running = None
        g_x = None
        g_z = None
        main()    
    except rospy.ROSInterruptException:
        pass
    