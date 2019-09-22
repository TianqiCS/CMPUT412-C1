#!/usr/bin/env python 
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Drive(smach.State):
    def __init__(self, cmd_vel_pub):
        global g_range_ahead
        global g_stop_dist
        self.cmd_vel_pub = cmd_vel_pub
        smach.State.__init__(self, 
                             outcomes=['twist_linear','twist_angular'],
                             input_keys=['state_change_time_in'],
                             output_keys=['state_change_time_out'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state DRIVE' + '\nrange_head:' + str(g_range_ahead))
        if g_range_ahead < g_stop_dist or rospy.Time.now() > userdata.state_change_time_in:
            userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(2)
            return 'twist_angular'
        else:
            twist = Twist()
            twist.linear.x = 1
            self.cmd_vel_pub.publish(twist)
            rospy.Rate(10).sleep()
            return 'twist_linear'

class Spin(smach.State):
    def __init__(self, cmd_vel_pub):
        self.cmd_vel_pub = cmd_vel_pub
        smach.State.__init__(self,
                             outcomes=['twist_linear','twist_angular'],
                             input_keys=['state_change_time_in'],
                             output_keys=['state_change_time_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SPIN')
        if rospy.Time.now() > userdata.state_change_time_in:
            userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(2)
            return 'twist_linear'
        else:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            rospy.Rate(10).sleep()
            return 'twist_angular'

def scan_callback(msg):
    global g_range_ahead  
    g_range_ahead = min(msg.ranges)

def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
    return default

def main():

    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.init_node('wanderSM',anonymous=True)

    sm = smach.StateMachine(outcomes=['twist_linear','twist_angular'])
    sm.userdata.sm_state_change_time = rospy.Time.now()

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    with sm:
        smach.StateMachine.add('DRIVE', Drive(cmd_vel_pub),
                                transitions={'twist_linear':'DRIVE',
                                             'twist_angular':'SPIN'},
                                remapping={'state_change_time_in':'sm_state_change_time',
                                           'state_change_time_out':'sm_state_change_time'})

        smach.StateMachine.add('SPIN', Spin(cmd_vel_pub),
                                transitions={'twist_linear':'DRIVE',
                                             'twist_angular':'SPIN'},
                                remapping={'state_change_time_in':'sm_state_change_time',
                                           'state_change_time_out':'sm_state_change_time'})
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('root', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    g_range_ahead = 1
    g_stop_dist = rospy.get_param('stop_dist', 2)
    main()