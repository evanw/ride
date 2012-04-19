#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')
import rospy
import std_msgs.msg

def callback(x):
    print x.data + ' to b'

def main():
    rospy.init_node('b')
    sub = rospy.Subscriber('in', std_msgs.msg.String, callback)
    pub = rospy.Publisher('out', std_msgs.msg.String)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        pub.publish('from b')
        rospy.sleep(2)

if __name__ == '__main__':
    main()
