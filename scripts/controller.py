#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker


def publish_marker(x, y, z):
    marker = Marker(type=Marker.SPHERE,
        id=0,
        action=Marker.ADD,
        pose=Pose(Point(x, y, 0), Quaternion(0, 0, 0, z)),
        header=Header(frame_id='base_link'),
        scale=Vector3(1, 1, 4),
        color=ColorRGBA(0.0, 1.0, 0.0, 1.0))
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    marker_publisher.publish(marker)


def move_towards(x, y, theta):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = rospy.get_param('husky_highlevel_controller/linear_gain')
    twist.linear.y = rospy.get_param('husky_highlevel_controller/linear_gain')
    twist.angular.z = rospy.get_param('husky_highlevel_controller/angular_gain') * (0-theta)
    pub.publish(twist)
    publish_marker(x, y, theta)


def _callback(msg):
    angle_min = msg.angle_min # The first ray of laser strikes at this angle
    angle_increment = msg.angle_increment
    min_distance = min(msg.ranges)
    theta_min_distance = angle_min + angle_increment * msg.ranges.index(min_distance)
    x = float(min_distance) * math.cos(theta_min_distance)
    y = float(min_distance) * math.sin(theta_min_distance)
    rospy.loginfo(str(min_distance) + "\n")
    rospy.loginfo("X-Pillar = %s \n" % x)
    rospy.loginfo("Y-Pillar = %s \n" % y)
    try:
        move_towards(x, y, theta_min_distance)
    except rospy.ROSInterruptException:
        pass


def main():
    # Gather config
    name = rospy.get_param('husky_highlevel_controller/name')
    queue_size = rospy.get_param('husky_highlevel_controller/queue_size')

    rospy.init_node(name, anonymous=True)
    rospy.Subscriber('/scan', LaserScan, _callback)

    rospy.spin()

if __name__ == '__main__':
    main()
