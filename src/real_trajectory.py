#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import scipy.linalg
import numpy as np
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class BebopTrajectory:

    def __init__(self):

        # True if node received bebop trajectory, otherwise false
        self.trajectory_received = False
        self.first_measurement = True
        self.controller_info = False

        self.trajectory_subscriber = rospy.Subscriber(
            "/multi_dof_trajectory",
            MultiDOFJointTrajectory,
            self.trajectory_cb)

        # initialize publishers
        self.pos_ref_pub = rospy.Publisher(
            "/bebop/pos_ref",
            Vector3,
            queue_size=10)
        self.ang_ref_pub = rospy.Publisher(
            "/bebop/angle_ref",
            Vector3,
            queue_size=10)

        self.actuator_msg = Actuators()

        # define vector for measured and setpoint values
        self.pose_sp = Vector3(0., 0., 0.)
        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)
        self.p = 0
        self.q = 0
        self.r = 0

        # Controller rate
        # Trajectory points are being given with a frequency of 100Hz
        self.controller_rate = 50
        self.rate = rospy.Rate(self.controller_rate)
        self.t_old = 0

        # Initialize trajectory information
        self.trajectory_index = 0
        self.trajectory_point_count = -1
        self.trajectory_points = []

        self.sleep_sec = 2

    def trajectory_cb(self, data):
        self.trajectory_received = True
        self.trajectory_point_count = len(data.points)
        self.trajectory_points = data.points

    def convert_to_euler(self, qx, qy, qz, qw):
        """Calculate roll, pitch and yaw angles/rates with quaternions"""

        #conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw
                                     - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                                     + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = self.p
        q = self.q
        r = self.r

        sx = math.sin(self.euler_mv.x)  # sin(roll)
        cx = math.cos(self.euler_mv.x)  # cos(roll)
        cy = math.cos(self.euler_mv.y)  # cos(pitch)
        ty = math.tan(self.euler_mv.y)  # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def run(self):
        """
        Run ROS node - computes motor speed for trajectory following using LQR algorithm
        """

        while not self.first_measurement:
            print("BebopTrajectory.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)

        print("BebopTrajectory.run() - Starting trajectory tracking")
        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():
            self.rate.sleep()

            # Calculate new time interval - dt
            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            # Don't evaluate sooner than controller rate specifies
            if dt < 0.99 / self.controller_rate:
                continue

            # Compute conversion to euler
            self.euler_mv.x, self.euler_mv.y, self.euler_mv.z, \
                self.euler_rate_mv.x, self.euler_rate_mv.y, self.euler_rate_mv.z = \
                self.quaternion2euler(self.qx, self.qy, self.qz, self.qw)

            # If trajectory isn't received, LQR is used for point following
            if not self.trajectory_received:
                pose = Vector3(1.13, - 1.05, 1.0)
                angle = Vector3(0., 0., 0.)
                vel = Vector3(0., 0., 0.)

            else:
                current_point = self.trajectory_points[self.trajectory_index]
                pose = current_point.transforms[0].translation
                vel = current_point.velocities[0].linear
                angle_q = current_point.transforms[0].rotation
                angle.x, angle.y, angle.z, _, _, _ = self.quaternion2euler(
                    angle_q.x, angle_q.y, angle_q.z, angle_q.w)

                # Increase trajectory point index, check if finished
                self.trajectory_index += 2
                print("BebopTrajectory.run() - point {} / {}\ndt: {}"
                      .format(self.trajectory_index, self.trajectory_point_count, dt))
                print("BebopTrajectory.run() - velocity at point {}".format(vel))
                if self.trajectory_index >= self.trajectory_point_count:
                    print("BebopTrajectory.run() -"
                          "Trajectory completed.")
                    self.trajectory_received = False
                    self.trajectory_index = 0

            # Publish trajectory point
            point_position = Vector3()
            point_position.x = pose.x
            point_position.y = pose.y
            point_position.z = pose.z
            self.pos_ref_pub.publish(point_position)

            quick_hack_msg2 = Vector3()
            quick_hack_msg2.x = 0
            quick_hack_msg2.y = 0
            quick_hack_msg2.z = angle.z
            self.ang_ref_pub.publish(quick_hack_msg2)


if __name__ == '__main__':
    rospy.init_node("bebop_trajectory")
    try:
        bebop_trajectory = BebopTrajectory()
        bebop_trajectory.run()
    except rospy.ROSInterruptException:
        pass
