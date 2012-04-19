#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')
import rospy
import std_msgs.msg

def callback(x):
    print x.data + ' to a'

def main():
    rospy.init_node('a')
    sub = rospy.Subscriber('in', std_msgs.msg.String, callback)
    pub = rospy.Publisher('out', std_msgs.msg.String)
    while not rospy.is_shutdown():
        pub.publish('from a')
        rospy.sleep(2)

if __name__ == '__main__':
    main()
