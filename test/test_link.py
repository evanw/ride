#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')
import rospy
import std_msgs.msg

def callback(x):
    print x.data + ' to ' + rospy.get_name()

def main():
    rospy.init_node('test')
    sub = rospy.Subscriber('in', std_msgs.msg.String, callback)
    pub = rospy.Publisher('out', std_msgs.msg.String)
    if 'b' in rospy.get_name():
        rospy.sleep(1)
    while not rospy.is_shutdown():
        pub.publish('from ' + rospy.get_name())
        rospy.sleep(2)

if __name__ == '__main__':
    main()
