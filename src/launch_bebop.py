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
        self.omega1, self.omega2, self.omega3, self.omega4 = 0, 0, 0, 0
        self.angle1, self.angle2, self.angle3, self.angle4 = 0, 0, 0, 0
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

        # define vector for measured and setopint values
        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)

        # define PID for height control
        self.z_sp = 0           # z-position set point
        self.x_sp = 7
        self.y_sp = 2
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
        self.pid_x.set_kp(2)
        self.pid_x.set_ki(2)
        self.pid_x.set_kd(2)

        self.pid_x.set_lim_high(0.02)
        self.pid_x.set_lim_low(-0.02)

        self.pid_y = PID()
        self.pid_y.set_kp(2)
        self.pid_y.set_ki(2)
        self.pid_y.set_kd(2)

        self.pid_y.set_lim_high(0.02)
        self.pid_y.set_lim_low(-0.02)

        # Add parameters for z controller
        self.pid_z.set_kp(5)
        self.pid_z.set_ki(1)
        self.pid_z.set_kd(2)

        # Add parameters for vz controller
        self.pid_vz.set_kp(20)
        self.pid_vz.set_ki(2)
        self.pid_vz.set_kd(4)

        # Set maximum ascend and descend vertical speed
        self.pid_z.set_lim_high(5)
        self.pid_z.set_lim_low(-5)

        # Set maximum angular velocity for  motor
        self.pid_vz.set_lim_high(1000)
        self.pid_vz.set_lim_low(-1000)

        # Create pitch and roll PIDs

        self.pitch_rate_PID = PID()
        self.pitch_PID = PID()

        self.pitch_rate_PID.set_kp(3.5)
        self.pitch_rate_PID.set_ki(1)
        self.pitch_rate_PID.set_kd(0.5)

        self.pitch_rate_PID.set_lim_high(50)
        self.pitch_rate_PID.set_lim_low(-50)

        self.pitch_PID.set_kp(5)
        self.pitch_PID.set_ki(0)
        self.pitch_PID.set_kd(0.5)

        self.pitch_PID.set_lim_high(20)
        self.pitch_PID.set_lim_low(-20)

        self.roll_rate_PID = PID()
        self.roll_PID = PID()

        self.roll_rate_PID.set_kp(3.5)
        self.roll_rate_PID.set_ki(1)
        self.roll_rate_PID.set_ki(0.5)

        self.roll_rate_PID.set_lim_high(20)
        self.roll_rate_PID.set_lim_low(-20)

        self.roll_PID.set_kp(5)
        self.roll_PID.set_ki(0)
        self.roll_PID.set_kd(0.5)

        self.roll_PID.set_lim_high(20)
        self.roll_PID.set_lim_low(-20)

        self.yaw_rate_PID = PID()
        self.yaw_PID = PID()

        self.yaw_rate_PID.set_kp(2)
        self.yaw_rate_PID.set_ki(0.5)
        self.yaw_rate_PID.set_kd(0.5)

        self.yaw_rate_PID.set_lim_high(20)
        self.yaw_rate_PID.set_lim_low(-20)

        self.yaw_PID.set_lim_high(20)
        self.yaw_PID.set_lim_low(-20)

        self.t_old = 0

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


    #h_kd = PID().kp, PID().ki, PID().kd


    def calculate_thrust(self, angular_velocity):
        """ Calculate thrust that motor with certain angular velocity produces
            params :  angular_velocity - motor_speed in rad/s"""

        thrust = self.bf * angular_velocity**2 * np.array([[0], [0], [1]])

        return thrust

    def calculate_drag_moments(self, angular_velocity, rotor_num):
        """ Calculate drag moment that motor with cerrtain angular velocity produces
            params : angular_velocity - motor_speed in rad/s
                     rotor_num - needed for zeta (1 or -1) based on cw or ccw
                     """

        zeta = 1 if rotor_num == (0 or 2) else -1

        drag_moment = zeta * self.bf * self.bm * angular_velocity**2 * np.array([[0], [0], [1]])

        return drag_moment

    def quadrotor_position_model(self, roll, pitch, yaw, angular_velocities):

        thrust = sum(map(calculate_thrust, angular_velocities))
        linear_acc_x = (thrust/self.mass) * (np.cos(roll) * np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw))
        linear_acc_y = (thrust/self.mass) * (np.cos(roll) * np.sin(pitch) * np.sin(yaw) - np.sin(roll) * np.cos(yaw))
        linear_acc_z = (thrust/self.mass) * (np.cos(roll) * np.sin(pitch)) - self.mass * self.gravity

        return np.array([[linear_acc_x], [linear_acc_y], [linear_acc_z]])

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
            des_roll_angle=0
            des_pitch_angle=0

            self.get_pitch_roll_yaw(self.qx, self.qy, self.qz, self.qw)

            # prefilter for reference
            a = 0.1
            self.z_ref_filt = (1 - a) * self.z_ref_filt + a * self.z_sp
            vz_ref = self.pid_z.compute(self.z_ref_filt, self.z_mv, dt)
            domega_z = self.pid_vz.compute(vz_ref, self.vz_mv, dt)

            # pitch control
            error_prc = self.euler_sp.y - self.euler_mv.y
            # x - position control
            b = 0.2
            self.x_ref_filt = (1 - b) * self.x_ref_filt + b * self.x_sp
            des_pitch_angle = self.pid_x.compute(self.x_ref_filt, self.x_mv, dt)
            #desired_pitch_angle = math.sin(self.euler_mv.z)*des_pitch_angle + math.cos(self.euler_mv.z)*des_roll_angle
            pitch_rate_ref = self.pitch_rate_PID.compute(des_pitch_angle, self.euler_mv.y, dt)
            print(self.euler_sp.y,  self.euler_mv.y)
            print(pitch_rate_ref)
            dwx = self.pitch_PID.compute(pitch_rate_ref,  self.euler_rate_mv.y, dt)

            # roll control
            c = 0.2
            error_rrc = self.euler_sp.x - self.euler_mv.x
            self.y_ref_filt = (1 - c) * self.y_ref_filt + c * self.y_sp
            print("y_ref_filt {}".format(self.y_ref_filt))
            des_roll_angle =  -self.pid_y.compute(self.y_ref_filt, self.y_mv, dt)
            # desired_roll_angle = math.cos(self.euler_mv.z)*des_roll_angle - math.sin(self.euler_mv.z)*des_pitch_angle
            roll_rate_ref = self.roll_rate_PID.compute(des_roll_angle, self.euler_mv.x, dt)
            dwy = self.roll_PID.compute(roll_rate_ref, self.euler_rate_mv.x, dt)

            # yaw control
            error_yrc = self.euler_sp.z - self.euler_mv.z
            dwz = self.yaw_rate_PID.compute(self.euler_sp.z,  self.euler_mv.z, dt)
            print(dwz)

            a = Actuators()

            # angular velocity of certain rotor
            motor_speed1 = self.hover_speed + domega_z - dwy/2 - dwx/2 - dwz/2
            motor_speed2 = self.hover_speed + domega_z + dwy/2 - dwx/2 + dwz/2
            motor_speed3 = self.hover_speed + domega_z + dwy/2 + dwx/2 - dwz/2
            motor_speed4 = self.hover_speed + domega_z - dwy/2 + dwx/2 + dwz/2
            a.angular_velocities = [0.908*motor_speed1, 0.908*motor_speed2,
                                    motor_speed3, motor_speed4]

            print("Comparison x:{}\nx_m:{}\ny:{}\ny_m:{}\nz:{}\nz_m{}".format(self.x_sp,
                                                                              self.x_mv,
                                                                              self.y_sp,
                                                                              self.y_mv,
                                                                              self.z_sp,
                                                                              self.z_mv))
            print("Error outputs are: {}, {}, {}".format(error_rrc, error_prc, error_yrc))
            print("Motor speeds are {}".format(a.angular_velocities))
            print("Current quadcopter height is: {}".format(self.z_mv))
            print("Hover speed is: {}\nPitch PID output is:{}\nRoll PID output is:{}\nYaw PID output is:{}\n".format(self.hover_speed,
                                                                                               dwx, dwy, dwz))

            # empty for starters
            pub.publish(a)
            # print('Published')
            self.z_sp = 3


if __name__=="__main__":
    rospy.init_node('bebop_launch', anonymous=True)
    try:
        LB = LaunchBebop()
        LB.run()
    except rospy.ROSInterruptException:
        pass



