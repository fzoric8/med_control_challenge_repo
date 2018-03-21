#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry


class BebopControl(object):

    # Time in seconds spent sleeping in run()
    # if no measurement is found
    SLEEP_SEC = 0.5

    def __init__(self):
        """
        Constructor for the BebopControl class.
        """

        # Flag signaling when first measurement occurs
        self.first_measurement_flag = False

        # Initialize bebop odometry parameters
        # mv - measured value
        # sp - set point (reference value)
        self.x_mv = 0
        self.y_mv = 0
        self.z_mv = 0

        self.vx_mv = 0
        self.vy_mv = 0
        self.vz_mv = 0

        self.wx_mv = 0
        self.wy_mv = 0
        self.wz_mv = 0

        # Initialize subscribers
        rospy.Subscriber('bebop/odometry', Odometry, self.odometry_callback)

    def run(self):
        """
        Start running the control loop.
        """

        # Wait for first measurement to occur
        if not self.first_measurement_flag:
            print("BebopControl.run() - Waiting for first measurement")
            rospy.sleep(BebopControl.SLEEP_SEC)

        print("BebopControl.run() - Starting the control loop")
        while not rospy.is_shutdown():
            # TODO: Publish control values
            # TODO: Find out where are control values
            pass

    def odometry_callback(self, data):
        """
        Callback function for Bebop odometry publisher. Initializes
        new values for position, linear and angular velocity.

        :param data: Data sent from publisher to the callback function
        """
        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y
        self.z_mv = data.pose.pose.position.z

        self.vx_mv = data.twist.twist.linear.x
        self.vx_mv = data.twist.twist.linear.y
        self.vx_mv = data.twist.twist.linear.z

        self.wx_mv = data.twist.twist.angular.x
        self.wy_mv = data.twist.twist.angular.y
        self.wz_mv = data.twist.twist.angular.z


if __name__ == '__main__':
    """
    Initialize bebop control node and start the controller.
    """
    rospy.init_node("bebop_control")
    bebopControl = BebopControl()
    bebopControl.run()


