#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from std_msgs.msg import *
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from pid import PID


class LaunchBebop():

    def __init__(self):
        """Constructor initializes all needed variables"""
        self.mass = 0.5         # kg --> mass of the quadcopter
        self.Ixx = 0.00389      # kg m^2  --> Quadrotor moment of inertia in body x direction
        self.Iyy = 0.00389      # kg m^2  --> Quadrotor moment of inertia in body y direction
        self.Izz = 0.0078       # kg m^2  --> Quadrotor moment of inertia in body z direction
        self.Tm = 0.0125        # s       --> Time constant of a motor
        self.bf = 8.548e-6      # kg m    --> Thrust constant of a motor
        self.bm = 0.016         # m       --> Moment constant of a motor
        self.l = 0.12905        # m       --> The distance of a motor from a center of mass
        self.gravity = 9.81     # m/s^2   --> Gravity value
        self.sleep_sec = 0.5    # sleep duration while not getting first measurement
        self.first_measurement = False
        self.odom_subscriber = rospy.Subscriber("bebop/odometry", Odometry, self.odometry_callback)
        self.pose_subscriber = rospy.Subscriber("bebop/pos_ref", Vector3, self.setpoint_cb)


        # define vector for measured and setopint values
        self.pose_sp = Vector3(0., 0., 0.)
        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)
        self.t_old = 0

        # define PID for height control
        self.z_ref_filt = 0     # z ref filtered
        self.y_ref_filt = 0
        self.x_ref_filt = 0
        self.z_mv = 0           # z-position measured value
        self.pid_z = PID()      # pid instance for z control

        # define PID for height rate control
        self.vz_sp = 0          # vz velocity set point
        self.vz_mv = 0          # vz velocity measured value
        self.pid_vz = PID()     # pid instance for z-velocity control

        self.pid_x = PID()
        self.pid_y = PID()

        # outer_loops
        self.pitch_PID = PID()
        self.roll_PID = PID()
        self.yaw_PID = PID()

        # inner_loops
        self.pitch_rate_PID = PID()
        self.roll_rate_PID = PID()
        self.yaw_rate_PID = PID()

        self.set_PIDs()


    def set_PIDs(self):
        """ Set gains for all PID controllers

        :return:
        """
        self.pid_x.set_kp(4)
        self.pid_x.set_ki(2)
        self.pid_x.set_kd(2)

        self.pid_y.set_kp(4)
        self.pid_y.set_ki(2)
        self.pid_y.set_kd(2)

        self.pid_y.set_lim_high(0.05)
        self.pid_y.set_lim_low(-0.05)

        # Add parameters for z controller
        self.pid_z.set_kp(2)
        self.pid_z.set_ki(0.5)
        self.pid_z.set_kd(0)

        # Add parameters for vz controller
        self.pid_vz.set_kp(195.8)
        self.pid_vz.set_ki(0)
        self.pid_vz.set_kd(1.958)

        # Set maximum ascend and descend vertical speed
        self.pid_z.set_lim_high(10)
        self.pid_z.set_lim_low(-10)

        # Set maximum angular velocity for  motor
        self.pid_vz.set_lim_high(1000)
        self.pid_vz.set_lim_low(-1000)

        self.pid_x.set_lim_high(0.05)
        self.pid_x.set_lim_low(-0.05)

        self.pitch_PID.set_kp(4.44309)
        self.pitch_PID.set_ki(0.1)
        self.pitch_PID.set_kd(0.2)

        self.pitch_PID.set_lim_high(100)
        self.pitch_PID.set_lim_low(-100)

        self.pitch_rate_PID.set_kp(16.61)
        self.pitch_rate_PID.set_ki(0)
        self.pitch_rate_PID.set_kd(0)

        self.pitch_rate_PID.set_lim_high(200)
        self.pitch_rate_PID.set_lim_low(-200)

        self.roll_PID.set_kp(4.44309)
        self.roll_PID.set_ki(0.1)
        self.roll_PID.set_ki(0.2)

        self.roll_PID.set_lim_high(100)
        self.roll_PID.set_lim_low(-100)

        self.roll_rate_PID.set_kp(16.61)
        self.roll_rate_PID.set_ki(0)
        self.roll_rate_PID.set_kd(0)

        self.roll_rate_PID.set_lim_high(100)
        self.roll_rate_PID.set_lim_low(-100)

        self.yaw_rate_PID.set_kp(4)
        self.yaw_rate_PID.set_ki(0.5)
        self.yaw_rate_PID.set_kd(0.5)

        self.yaw_rate_PID.set_lim_high(20)
        self.yaw_rate_PID.set_lim_low(-20)

        self.yaw_PID.set_lim_high(20)
        self.yaw_PID.set_lim_low(-20)

    def setpoint_cb(self, data):

        self.pose_sp.x = data.x
        self.pose_sp.y = data.y
        self.pose_sp.z = data.z



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


    def get_pitch_roll_yaw(self, qx, qy, qz, qw):
        """Calculate roll, pitch and yaw angles/rates with quaternions"""

        # conversion quaternion to euler (yaw - pitch - roll)
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
        """ Run ROS node - computes PID algorithms for z and vz control """

        # init publisher
        pub = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=10)

        # check Rate for starters 20
        rate = rospy.Rate(10)

        while not self.first_measurement:
            print("Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)
        print("Starting height control")

        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():
            rate.sleep()
            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            print(dt)

            self.t_old = t
            self.hover_speed = math.sqrt(4.905/self.bf/4)

            self.get_pitch_roll_yaw(self.qx, self.qy, self.qz, self.qw)

            # prefilter for reference
            a = 0.1
            self.z_ref_filt = (1 - a) * self.z_ref_filt + a * self.pose_sp.z
            # outer loop height
            vz_ref = self.pid_z.compute(self.z_ref_filt, self.z_mv, dt)
            # inner loop height
            domega_z = self.pid_vz.compute(vz_ref, self.vz_mv, dt)

            # pitch control
            #error_prc = self.euler_sp.y - self.euler_mv.y
            # x - position control
            b = 0.3
            self.x_ref_filt = (1 - b) * self.x_ref_filt + b * self.pose_sp.x
            des_pitch_angle = self.pid_x.compute(self.pose_sp.x, self.x_mv, dt)
            #desired_pitch_angle = math.sin(self.euler_mv.z)*des_pitch_angle + math.cos(self.euler_mv.z)*des_roll_angle
            # outer loop pitch
            pitch_rate_ref = self.pitch_PID.compute(des_pitch_angle, self.euler_mv.y, dt)
            # inner loop pitch
            dwx = self.pitch_rate_PID.compute(pitch_rate_ref,  self.euler_rate_mv.y, dt)

            # roll control
            c = 0.3
            #error_rrc = self.euler_sp.x - self.euler_mv.x
            self.y_ref_filt = (1 - c) * self.y_ref_filt + c * self.pose_sp.y
            print("y_ref_filt {}".format(self.y_ref_filt))
            des_roll_angle = -self.pid_y.compute(self.pose_sp.y, self.y_mv, dt)
            # desired_roll_angle = math.cos(self.euler_mv.z)*des_roll_angle - math.sin(self.euler_mv.z)*des_pitch_angle
            # outer loop roll
            roll_rate_ref = self.roll_PID.compute(des_roll_angle, self.euler_mv.x, dt)
            # inner loop roll
            dwy = self.roll_rate_PID.compute(roll_rate_ref, self.euler_rate_mv.x, dt)

            # yaw control
            error_yrc = self.euler_sp.z - self.euler_mv.z
            dwz = self.yaw_rate_PID.compute(self.euler_sp.z,  self.euler_mv.z, dt)
            print(dwz)

            a = Actuators()

            # angular velocity of certain rotor
            motor_speed1 = self.hover_speed + domega_z - dwy - dwx - dwz
            motor_speed2 = self.hover_speed + domega_z + dwy - dwx + dwz
            motor_speed3 = self.hover_speed + domega_z + dwy + dwx - dwz
            motor_speed4 = self.hover_speed + domega_z - dwy + dwx + dwz
            a.angular_velocities = [0.908*motor_speed1, 0.908*motor_speed2,
                                    motor_speed3, motor_speed4]

            print("Comparison x:{}\nx_m:{}\ny:{}\ny_m:{}\nz:{}\nz_m{}".format(self.pose_sp.x,
                                                                              self.x_mv,
                                                                              self.pose_sp.y,
                                                                              self.y_mv,
                                                                              self.pose_sp.z,
                                                                              self.z_mv))
            print("Motor speeds are {}".format(a.angular_velocities))
            print("Current quadcopter height is: {}".format(self.z_mv))
            print("Hover speed is: {}\nPitch PID output is:{}\nRoll PID output is:{}\nYaw PID output is:{}\n".format(self.hover_speed,
                                                                                               dwx, dwy, dwz))

            # empty for starters
            pub.publish(a)
            # print('Published')


if __name__=="__main__":
    rospy.init_node('bebop_launch', anonymous=True)
    try:
        LB = LaunchBebop()
        LB.run()
    except rospy.ROSInterruptException:
        pass
