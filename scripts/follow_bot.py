#!/usr/bin/env python
import rospy
import smach
import smach_ros
import math
import ctypes
import struct
import time
#import numpy as np
from kobuki_msgs.msg import BumperEvent, Led
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

## Util Function

def approxEqual(a, b, tol = 0.001):
    return abs(a - b) <= tol

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
    

##

def readPC(msg):
    
    # 0.5 - 1s per loop (too slow!)
    if msg:

        points = filter(filterColor, point_cloud2.read_points(msg, skip_nans=True))

        for point in points:

            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            rgb_packed = point[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,rgb_packed)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            pt_r = (pack & 0x00FF0000)>> 16
            pt_g = (pack & 0x0000FF00)>> 8
            pt_b = (pack & 0x000000FF)
            rospy.loginfo('Point: x: {0}, y:{1}, z{2}, r:{3}, g:{4}, b:{5}'.format(pt_x, pt_y, pt_z, pt_r, pt_g, pt_b))


##State Classes
class Start(smach.State):
    def __init__(self):
        global g_collision, g_odom, g_cmd_vel_pub, g_PC2
        smach.State.__init__(self, 
                             outcomes=['success'])
    def execute(self, userdata):
        while True:
            rospy.sleep(1.)
            readPC(g_PC2)
        return 'success'


# class Parallel(smach.State):
#     def __init__(self):
#         global g_collision, g_odom, g_cmd_vel_pub
#         smach.State.__init__(self, 
#                              outcomes=['collision', 'end'])

#     def execute(self, userdata):
#         twist = Twist()
#         twist.linear.x = 0.1
#         while True:
#             rospy.loginfo('Executing state PARALLEL' + '\ng_odom:' + str(g_odom) + '\ng_collision:' + str(g_collision))
#             g_cmd_vel_pub.publish(twist)
#             if g_collision:
#                 return 'collision'
#             if g_odom['x'] > 3.4:
#                 twist.linear.x = 0.0
#                 g_cmd_vel_pub.publish(twist)
#                 rospy.loginfo('\nBUMPER_BOT ARRIVED!\n')
#                 return 'end'

# class Backup(smach.State):
#     def __init__(self):
#         global g_collision, g_odom, g_cmd_vel_pub
#         smach.State.__init__(self, 
#                              outcomes=['success'])

#     def execute(self, userdata):
#         ori_x = g_odom['x']
#         ori_y = g_odom['y']
#         twist = Twist()
#         twist.linear.x = -0.1
#         while True:
#             rospy.loginfo('Executing state BACKUP' + '\ng_odom:' + str(g_odom) + '\ng_collision:' + str(g_collision))
#             g_cmd_vel_pub.publish(twist)
#             if abs(ori_x - g_odom['x']) > 0.1 or abs(ori_y - g_odom['y']) > 0.1:
#                 return 'success'
#             #elif g_collision:
#                 #return 'collision'

# class TurnPerpendicular(smach.State):
#     def __init__(self):
#         global g_odom,g_cmd_vel_pub
#         smach.State.__init__(self, 
#                              outcomes=['success'])

#     def execute(self, userdata):
#         init_yaw = g_odom['yaw_z']
#         target_yaw = None
#         if approxEqual(init_yaw, math.pi/2, 0.2):
#             target_yaw = -math.pi/2
#         elif approxEqual(init_yaw, -math.pi/2, 0.2):
#             target_yaw = math.pi/2
#         else:
#             target_yaw = math.pi/2
            
#         twist = Twist()
#         while True:
#             rospy.loginfo('Executing state TURNPERPENDICULAR' + '\ng_odom:' + str(g_odom))
#             if approxEqual(g_odom['yaw_z'], target_yaw):
#                 twist.angular.z = 0
#                 g_cmd_vel_pub.publish(twist)
#                 rospy.sleep(1)
#                 return 'success'

#             if g_odom['yaw_z'] > target_yaw:
#                 twist.angular.z = -0.3
#             else:
#                 twist.angular.z = 0.3
#             g_cmd_vel_pub.publish(twist)

# class Perpendicular(smach.State):
#     def __init__(self):
#         global g_odom, g_collision, g_cmd_vel_pub
#         smach.State.__init__(self, 
#                              outcomes=['collision', 'success'])

#     def execute(self, userdata):
#         old_y = g_odom['y']
#         twist = Twist()
#         twist.linear.x = 0.1
#         while True:
#             rospy.loginfo('Executing state PERPENDICULAR' + '\ng_odom:' + str(g_odom) + '\ng_collision:' + str(g_collision))
#             g_cmd_vel_pub.publish(twist)
#             if abs(g_odom['y'] - old_y) >= 0.6:
#                 return 'success'
#             if g_collision:
#                 return 'collision'


# class TurnParallel(smach.State):
#     def __init__(self):
#         global g_odom, g_cmd_vel_pub
#         smach.State.__init__(self, 
#                              outcomes=['success'])

#     def execute(self, userdata):
#         twist = Twist()
#         target_yaw = 0.0
#         while True:
#             rospy.loginfo('Executing state TURNPARALLEL' + '\ng_odom:' + str(g_odom))
#             if approxEqual(g_odom['yaw_z'], target_yaw):
#                 twist.angular.z = 0
#                 g_cmd_vel_pub.publish(twist)
#                 rospy.sleep(1)
#                 return 'success'

#             if (g_odom['yaw_z'] < target_yaw):
#                 twist.angular.z = 0.3
#             elif (g_odom['yaw_z'] > target_yaw):
#                 twist.angular.z = -0.3
#             g_cmd_vel_pub.publish(twist)
  
##

##call_back functions

def odom_callback(msg):
    global g_odom
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w,
    ])
    
    g_odom['x'] = x
    g_odom['y'] = y
    g_odom['yaw_z'] = yaw[2]

def bumper_callback(msg):
    global g_collision
    g_collision = msg.state

def PC2_callback(msg):
    global g_PC2
    g_PC2 = msg

##

def main():
    global g_collision, g_odom, g_led_pub, g_cmd_vel_pub
    g_collision = False
    g_odom = {'x':0.0, 'y':0.0, 'yaw_z':0.0}

    rospy.init_node('follow_bot',anonymous=True)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('mobile_base/events/bumper', BumperEvent, bumper_callback)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, PC2_callback)

    g_cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    g_led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)

    sm = smach.StateMachine(outcomes=['end'])

    with sm:
        smach.StateMachine.add('Start', Start(), transitions={'success':'end'}) 
        # smach.StateMachine.add('Parallel', Parallel(),
        #                         transitions={'end':'end',
        #                                      'collision': 'Backup'})   

        # smach.StateMachine.add('Backup', Backup(),
        #                         transitions={'success':'TurnPerpendicular'})

        # smach.StateMachine.add('TurnPerpendicular', TurnPerpendicular(),
        #                         transitions={'success':'Perpendicular'})

        # smach.StateMachine.add('Perpendicular', Perpendicular(),
        #                         transitions={'success':'TurnParallel',
        #                                      'collision': 'Backup'})

        # smach.StateMachine.add('TurnParallel', TurnParallel(),
        #                         transitions={'success':'Parallel'}) 

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('root', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    if outcome == 'end':
        led = Led()
        led.value = 1
        g_led_pub.publish(led)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    g_collision = None
    g_odom = None
    g_led_pub = None
    g_cmd_vel_pub = None
    g_PC2 = None
    main()