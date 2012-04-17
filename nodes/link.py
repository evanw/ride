#!/usr/bin/env python
import roslib; roslib.load_manifest('ride')
import getopt
import rospy
import sys

def load_msg_type(type):
    '''Load a message type object from a string'''
    package, message = type.split('/')
    roslib.load_manifest(package)
    module = __import__(package + '.msg._' + message)
    return getattr(getattr(module.msg, '_' + message), message)

def main():
    # Parse command line arguments
    args = dict(getopt.gnu_getopt(sys.argv, '', ['type=', 'from=', 'to='])[0])
    type = args['--type']
    from_topic = args['--from']
    to_topic = args['--to']

    # Create the link node
    rospy.init_node('link')
    type = load_msg_type(type)
    pub = rospy.Publisher(to_topic, type)
    sub = rospy.Subscriber(from_topic, type, lambda x: pub.publish(x))
    rospy.spin()

if __name__ == '__main__':
    main()
