#!/usr/bin/env python
import roslib; roslib.load_manifest('remapping_test')
import rospy
from std_msgs.msg import String

def callback(data):
    print data.data

rospy.init_node('recv')
rospy.Subscriber('bar', String, callback)
rospy.spin()
