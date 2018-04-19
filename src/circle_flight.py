#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math

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
        self.pos_ref_msg = Vector3()

        self.ang_ref_pub = rospy.Publisher(
            "bebop/angle_ref",
            Vector3,
            queue_size=10)
        self.ang_ref_msg = Vector3()

        # Odometry subscriber
        self.odom_subscriber = rospy.Subscriber(
            "bebop/odometry",
            Odometry,
            self.odometry_callback)
        self.first_measurement = False

        # Flight control signal
        self.fc_subscriber = rospy.Subscriber(
            "bebop/flight_control",
            Vector3,
            self.fc_callback)
        self.forward = 1    # 1 Is forward, 0 is backward
        self.start = 0      # 1 is start, 0 is stop

        # Sleep time if no measurement found
        self.sleep_sec = 2

        # Crontroller rate
        self.controller_rate = 1
        self.rate = rospy.Rate(self.controller_rate)

        # ~Windmill height(?)
        self.windmill_height = 2.16
        self.back_dist = - 1
        self.angle_delta = math.pi / 180 * 5
        self.windmill_radius = 2 - self.back_dist

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

    def quaternion2euler(self, qx, qy, qz, qw):
        """
        Calculate roll, pitch and yaw angles/rates with quaternions.

        :returns:
            This function returns following information:
                pitch, roll, yaw,
                pitch_rate, roll_rate, yaw_rate
        """
        # conversion quaternion to euler (yaw - pitch - roll)
        roll = math.atan2(2 * (qw * qx + qy * qz), qw * qw
                          - qx * qx - qy * qy + qz * qz)
        pitch = math.asin(2 * (qw * qy - qx * qz))
        yaw = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                         + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = self.p
        q = self.q
        r = self.r

        sx = math.sin(roll)  # sin(roll)
        cx = math.cos(roll)  # cos(roll)
        cy = math.cos(pitch)  # cos(pitch)
        ty = math.tan(pitch)  # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        roll_rate = p + sx * ty * q + cx * ty * r
        pitch_rate = cx * q - sx * r
        yaw_rate = sx / cy * q + cx / cy * r

        return roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate

    def run(self):

        while not self.first_measurement:
            print("BebopCircleFlight.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)

        # Get initial position
        self.get_current_position()
        self.initial_x = self.curr_x
        self.initial_y = self.curr_y
        self.initial_z = self.curr_z
        self.initial_yaw = self.curr_yaw

        self.takeoff()

        # Initialize angle offset
        angle_offset = - math.pi / 2

        while not rospy.is_shutdown():
            self.rate.sleep()
            self.get_current_position()

            if self.start == 0:
                print("CircleFlight.run() - Waiting for flight_control")
                continue

            self.pos_ref_msg.y = self.initial_y - self.windmill_radius * math.cos(angle_offset)
            self.pos_ref_msg.x = self.initial_x + self.windmill_radius + self.back_dist + self.windmill_radius * math.sin(angle_offset)
            self.ang_ref_msg.z = ( (self.initial_yaw + angle_offset) + math.pi / 2 ) % (2 * math.pi)

            print("X_off: {}, y_off: {}, yaw_off: {}"
                  .format(self.pos_ref_msg.x, self.pos_ref_msg.y, self.ang_ref_msg.z))

            # Publish position
            self.pos_ref_pub.publish(self.pos_ref_msg)

            # Publish angle
            self.ang_ref_pub.publish(self.ang_ref_msg)
            rospy.sleep(0.1)

            # Increase angle delta
            angle_offset += self.forward * self.angle_delta

    def get_current_position(self):
        """
        Get current bebop position
        """
        # Get current position
        _, _, self.curr_yaw, _, _, _ = self.quaternion2euler(
            self.qx, self.qy, self.qz, self.qw)
        self.curr_x = self.x_mv
        self.curr_y = self.y_mv
        self.curr_z = self.z_mv

    def takeoff(self):
        # Take - off
        print("BebopCircleFlight() - takeoff ready.")
        rospy.sleep(self.sleep_sec)

        self.pos_ref_msg.x = self.back_dist * math.cos(self.curr_yaw)
        self.pos_ref_msg.y = self.back_dist * math.sin(self.curr_yaw)
        self.pos_ref_msg.z = self.windmill_height
        self.pos_ref_pub.publish(self.pos_ref_msg)

        print("BebopCircleFlight() - takeff completed.")
        rospy.sleep(self.sleep_sec)

    def fc_callback(self, data):
        self.forward = data.x
        self.start = data.y


if __name__ == '__main__':
    rospy.init_node('circle_flight', anonymous=True)
    try:
        cf = BebopCircleFlight()
        cf.run()
    except rospy.ROSInterruptException:
        pass