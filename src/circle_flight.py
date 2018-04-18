#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class BebopCircleFlight:
    """
    This class uses bebop_launch node to guide bebop in a
    circle around the windmill. It will not start flying until
    a message on /bebop/start_flight is published.
    """

    def __init__(self):

        # Reference publishers
        self.pos_ref_pub = rospy.Publisher(
            "bebop/pos_ref",
            Vector3,
            queue_size=10)
        self.ang_ref_pub = rospy.Publisher(
            "bebop/pos_ref",
            Vector3,
            queue_size=10)

        # Odometry subscriber
        self.odom_subscriber = rospy.Subscriber(
            "bebop/odometry",
            Odometry,
            self.odometry_callback)

        # Sleep time if no measurement found
        self.sleep_sec = 2

    def odometry_callback(self, data):
        """Callback function for odometry subscriber"""

        self.first_measurement = True

        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y
        self.z_mv = data.pose.pose.position.z

        self.vx_mv = data.twist.twist.linear.x
        self.vy_mv = data.twist.twist.linear.y
        self.vz_mv = data.twist.twist.linear.z

        self.p = data.twist.twist.angular.x
        self.q = data.twist.twist.angular.y
        self.r = data.twist.twist.angular.z

        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w

    def run(self):

        while not self.first_measurement:
            print("BebopCircleFlight.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)


if __name__ == '__main__':
    rospy.init_node('circle_flight', anonymous=True)
