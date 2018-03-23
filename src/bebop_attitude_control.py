#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from .pid import PID


class BebopAttitudeControl(object):
    """
    Bebop Attitude control (yaw, pitch, roll, height).
    """

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

        self.x_sp = 0
        self.y_sp = 0
        self.z_sp = 0
        
        self.vx_mv = 0
        self.vy_mv = 0
        self.vz_mv = 0

        self.wx_mv = 0
        self.wy_mv = 0
        self.wz_mv = 0

        self.euler_rate_mv = Vector3()

        # Controller rate
        self.rate = 100
        self.ros_rate = rospy.Rate(self.rate)

        # Initialize subscribers
        rospy.Subscriber('bebop/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('bebop/pos_ref', Vector3, self.pos_ref_callback)

        # Initialize controllers
        self.clock = Clock()

        self.pid_roll = PID()  # roll controller
        self.pid_roll_rate = PID()  # roll rate (wx) controller

        self.pid_pitch = PID()  # pitch controller
        self.pid_pitch_rate = PID()  # pitch rate (wy) controller

        self.pid_yaw = PID()  # yaw controller
        self.pid_yaw_rate = PID()  # yaw rate (wz) controller

        self.pid_z = PID()  # pid instance for z control
        self.pid_vz = PID()  # pid instance for z-velocity control


    def run(self):
        """
        Start running the control loop.
        """

        # Wait for first measurement to occur
        while not self.first_measurement_flag:
            print("BebopControl.run() - Waiting for first measurement")
            rospy.sleep(BebopAttitudeControl.SLEEP_SEC)

        print("BebopControl.run() - Starting the control loop")
        while not rospy.is_shutdown():

            # First sleep
            self.ros_rate.sleep()

            print("Height-", self.z_sp)

    def odometry_callback(self, data):
        """
        Callback function for Bebop odometry publisher. Initializes
        new values for position, linear and angular velocity.

        :param data: Data sent from publisher to the callback function
        """
        self.first_measurement_flag = True

        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y
        self.z_mv = data.pose.pose.position.z

        self.vx_mv = data.twist.twist.linear.x
        self.vx_mv = data.twist.twist.linear.y
        self.vx_mv = data.twist.twist.linear.z

        self.wx_mv = data.twist.twist.angular.x
        self.wy_mv = data.twist.twist.angular.y
        self.wz_mv = data.twist.twist.angular.z

        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.porientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw
                                     - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                                     + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = data.angular_velocity.x
        q = data.angular_velocity.y
        r = data.angular_velocity.z

        sx = math.sin(self.euler_mv.x)  # sin(roll)
        cx = math.cos(self.euler_mv.x)  # cos(roll)
        cy = math.cos(self.euler_mv.y)  # cos(pitch)
        ty = math.tan(self.euler_mv.y)  # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def pos_ref_callback(self, data):
        """
        Callback function for the Position reference topic.

        :param data: Data sent from topic.
        """
        self.x_sp = data.x
        self.y_sp = data.y
        self.z_sp = data.z


if __name__ == '__main__':
    """
    Initialize bebop control node and start the controller.
    """
    rospy.init_node("bebop_control")
    bebopControl = BebopAttitudeControl()
    bebopControl.run()


