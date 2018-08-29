#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def _callback(msg):
    rospy.loginfo(min(msg.ranges))


def listen():
    # Gather config
    name = rospy.get_param('husky_highlevel_controller/name')
    queue_size = rospy.get_param('husky_highlevel_controller/queue_size')

    rospy.init_node(name, anonymous=True)
    rospy.Subscriber('/scan', LaserScan, _callback)

    rospy.spin()

if __name__ == '__main__':
    listen()
