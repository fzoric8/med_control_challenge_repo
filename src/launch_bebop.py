#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from std_msgs.msg import *
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry



class LaunchBebop():

    def __init__(self):
        """Constructor initializes all needed variables"""
        self.omega1, self.omega2, self.omega3, self.omega4 = 400, 400, 440, 440
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


    def control_motor_speeds(self):
        """TO DO: controller for motor speeds based on some reference"""

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

    def publish_motor_speeds(self):
        """Create publisher and bebop launch node,
         send Actuators message to motor_speed command topic"""

        # init publisher
        pub = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=10)

        # check Rate for starters 20
        rate = rospy.Rate(20)

        # for starters init control vectors as zeros
        control_vector = [self.omega1, self.omega2, self.omega3, self.omega4]
        angle_control_vector = [self.angle1, self.angle2, self.angle3, self.angle4]

        while not self.first_measurement:
            rospy.sleep(self.sleep_sec)

        while not rospy.is_shutdown():

            a = Actuators()
            # msg header initialization
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id="base_link"
            a.header = header

            # angle of the actuator in [rad/s]
            a.angles = angle_control_vector

            # angular velocity of certain rotor
            a.angular_velocities = control_vector

            # empty for starters
            a.normalized = []
            pub.publish(a)
            # print('Published')
            print(dir(self.odom_subscriber))
            print(self.p)
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('bebop_launch', anonymous=True)
    try:
        LB = LaunchBebop()
        LB.publish_motor_speeds()
    except rospy.ROSInterruptException:
        pass



