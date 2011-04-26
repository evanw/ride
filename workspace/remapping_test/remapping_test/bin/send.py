#!/usr/bin/env python
import roslib; roslib.load_manifest('remapping_test')
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('foo', String)
rospy.init_node('send')
while not rospy.is_shutdown():
    pub.publish(String('test'))
    rospy.sleep(1)
