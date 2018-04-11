#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pid import PID


class LaunchBebop:

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
        self.hover_speed = math.sqrt(4.905 / self.bf / 4)

        self.first_measurement = False
        self.odom_subscriber = rospy.Subscriber("bebop/odometry",
                                                Odometry,
                                                self.odometry_callback)
        self.pose_subscriber = rospy.Subscriber("bebop/pos_ref",
                                                Vector3,
                                                self.setpoint_cb)
        self.odom_gt_subscriber = rospy.Subscriber("bebop/odometry_gt",
                                                   Odometry,
                                                   self.odometry_gt_callback)

        # initialize publishers
        self.motor_pub = rospy.Publisher('/gazebo/command/motor_speed',
                                         Actuators,
                                         queue_size=10)
        self.error_pub = rospy.Publisher('/bebop/pos_error',
                                         Float64,
                                         queue_size=10)
        self.actuator_msg = Actuators()

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
        self.z_mv = 0

        # Crontroller rate
        self.controller_rate = 50

        # define PID for height rate control
        self.vz_sp = 0          # vz velocity set point
        self.vz_mv = 0          # vz velocity measured value

        # Height controller
        self.pid_z = PID(4, 0.05, 0.1, 10, -10)
        self.pid_vz = PID(195.8, 0, 1.958, 300, -300)

        # Position loop
        self.pid_x = PID(1, 0.03, 0.5, 0.2, -0.2)
        self.pid_y = PID(1, 0.03, 0.5, 0.2, -0.2)

        # outer_loops
        self.pitch_PID = PID(4.44309, 0.1, 0.2, 100, -100)
        self.roll_PID = PID(4.44309, 0.1, 0.2, 100, -100)
        self.yaw_PID = PID(4, 0.5, 0.5, 20, -20)

        # inner_loops
        self.pitch_rate_PID = PID(16.61, 0, 0, 100, -100)
        self.roll_rate_PID = PID(16.61, 0, 0, 100, -100)

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

    def odometry_gt_callback(self, data):
        self.x_gt_mv = data.pose.pose.position.x
        self.y_gt_mv = data.pose.pose.position.y
        self.z_gt_mv = data.pose.pose.position.z

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

        # check Rate for starters 20
        rate = rospy.Rate(self.controller_rate)

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

            if dt < 1.0/self.controller_rate:
                continue

            self.get_pitch_roll_yaw(self.qx, self.qy, self.qz, self.qw)

            # HEIGHT CONTROL
            filt_const_z = 0.1
            self.z_ref_filt = (1 - filt_const_z) * self.z_ref_filt \
                              + filt_const_z * self.pose_sp.z
            vz_sp = self.pid_z.compute(self.z_ref_filt, self.z_mv, dt)
            u_height = self.pid_vz.compute(vz_sp, self.vz_mv, dt)

            # PITCH CONTROL
            # error_prc = self.euler_sp.y - self.euler_mv.y
            # x - position control
            filt_const_pitch = 0.5
            self.x_ref_filt = (1 - filt_const_pitch) * self.x_ref_filt \
                              + filt_const_pitch * self.pose_sp.x
            pitch_sp = self.pid_x.compute(self.pose_sp.x, self.x_mv, dt)
            pitch_rate_sp = self.pitch_PID.compute(pitch_sp, self.euler_mv.y, dt)
            u_pitch = self.pitch_rate_PID.compute(pitch_rate_sp,  self.euler_rate_mv.y, dt)

            # ROLL CONTROL
            # error_rrc = self.euler_sp.x - self.euler_mv.x
            # y position control
            filt_const_roll = 0.5
            self.y_ref_filt = (1 - filt_const_roll) * self.y_ref_filt \
                              + filt_const_roll * self.pose_sp.y
            roll_sp = -self.pid_y.compute(self.pose_sp.y, self.y_mv, dt)
            roll_rate_sp = self.roll_PID.compute(roll_sp, self.euler_mv.x, dt)
            u_roll = self.roll_rate_PID.compute(roll_rate_sp, self.euler_rate_mv.x, dt)

            # YAW CONTROL
            # error_yrc = self.euler_sp.z - self.euler_mv.z
            u_yaw = self.yaw_PID.compute(self.euler_sp.z, self.euler_mv.z, dt)

            # Calculate position error
            error = math.sqrt((self.pose_sp.x - self.x_gt_mv)**2 +
                              (self.pose_sp.y - self.y_gt_mv)**2 +
                              (self.pose_sp.z - self.z_gt_mv)**2)
            self.error_pub.publish(error)

            # angular velocity of certain rotor
            motor_speed1 = self.hover_speed + u_height - u_roll - u_pitch - u_yaw
            motor_speed2 = self.hover_speed + u_height + u_roll - u_pitch + u_yaw
            motor_speed3 = self.hover_speed + u_height + u_roll + u_pitch - u_yaw
            motor_speed4 = self.hover_speed + u_height - u_roll + u_pitch + u_yaw
            self.actuator_msg.angular_velocities = \
                [0.908*motor_speed1, 0.908*motor_speed2, motor_speed3, motor_speed4]

            print("Comparison x:{}\nx_m:{}\ny:{}\ny_m:{}\nz:{}\nz_m{}".format(
                self.pose_sp.x,
                self.x_mv,
                self.pose_sp.y,
                self.y_mv,
                self.pose_sp.z,
                self.z_mv))
            print("Motor speeds are {}".format(self.actuator_msg.angular_velocities))
            print("Current quadcopter height is: {}".format(self.z_mv))
            print("Hover speed is: {}\n"
                  "Pitch PID output is:{}\n"
                  "Roll PID output is:{}\n"
                  "Yaw PID output is:{}\n"
                  "pitch_sp: {}, roll_sp: {}\n"
                  "Error: {}".format(
                self.hover_speed, u_pitch, u_roll, u_yaw, pitch_sp, roll_sp, error))

            self.motor_pub.publish(self.actuator_msg)


if __name__ == "__main__":
    rospy.init_node('bebop_launch', anonymous=True)
    try:
        LB = LaunchBebop()
        LB.run()
    except rospy.ROSInterruptException:
        pass

