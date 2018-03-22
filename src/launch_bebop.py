#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import *
from mav_msgs.msg import Actuators


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
                     rotor_num - needed for zeta (1 or -1) based on cw or ccw"""

        zeta = 1 if rotor_num == (0 or 2) else -1

        drag_moment = zeta * self.bf * self.bm * angular_velocity**2 * np.array([[0], [0], [1]])

        return drag_moment

    def quadrotor_position_model(self, roll, pitch, yaw, angular_velocities):

        thrust = sum(map(calculate_thrust, angular_velocities))
        linear_vel_x = (thrust/self.mass) * (np.cos(roll) * np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw))
        linear_vel_y = (thrust/self.mass) * (np.cos(roll) * np.sin(pitch) * np.sin(yaw) - np.sin(roll) * np.cos(yaw))
        linear_vel_z = (thrust/self.mass) * (np.cos(roll) * np.sin(pitch)) - self.mass * self.gravity

        return np.array([[linear_vel_x], [linear_vel_y], [linear_vel_z]])

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
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('bebop_launch', anonymous=True)
    try:
        LB = LaunchBebop()
        LB.publish_motor_speeds()
    except rospy.ROSInterruptException:
        pass



