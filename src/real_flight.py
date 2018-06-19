#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pid import PID
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped

MAX_TILT = 20 * math.pi / 180
MAX_VZ = 1
MAX_ROTV = 100 * math.pi / 180
VERBOSE = True


class RealFlight:

    def __init__(self):
        
        self.pose_sub = rospy.Subscriber(
            "bebop/optitrack/pose",   
            PoseStamped,
            self.pose_cb)
        
        self.vel_sub = rospy.Subscriber(
            "/bebop/optitrack/velocity", 
            TwistStamped,
            self.vel_cb)

        self.pose_subscriber = rospy.Subscriber(
            "/bebop/pos_ref",
            Vector3,
            self.setpoint_cb)
            
        self.angle_subscriber = rospy.Subscriber(
            "/bebop/angle_ref",
            Vector3,
            self.angle_cb)

        self.pose_pub = rospy.Publisher(
            "/bebop/pose_set",
            Vector3,
            queue_size=10)

        self.rms_pub = rospy.Publisher(
            "/bebop/rms",
            Float64,
            queue_size=10
        )
            
        self.vel_publisher = rospy.Publisher(
            '/bebop/cmd_vel',
            Twist,
            queue_size=10)

        self.twist_msg = Twist()
        
        self.sleep_sec = 2
        self.first_measurement1 = False
        self.first_measurement2 = False
        self.controller_rate = 50
        self.rate = rospy.Rate(self.controller_rate)

        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)
        self.pose_sp = Vector3(0., 0., 1.)
        self.qx, self.qy, self.qz, self.qw = 0., 0., 0., 0.
        self.p, self.q, self.r, = 0., 0., 0.
        self.x_mv, self.y_mv, self.z_mv = 0., 0., 0.

        
        # Pre-filter constants
        self.filt_const_x = 0.5
        self.filt_const_y = 0.5
        self.filt_const_z = 0.1

        self.x_filt_sp = 0.
        self.y_filt_sp = 0.
        self.z_filt_sp = 0.
        
        self.pid_z = PID(4, 0.05, 0.1, MAX_VZ, - MAX_VZ)
        self.pid_x = PID(0.25, 0.0, 0.1, MAX_TILT, - MAX_TILT)
        self.pid_y = PID(0.25, 0.0, 0.1, MAX_TILT, - MAX_TILT)
        self.yaw_PID = PID(1, 0, 0.0, MAX_ROTV, - MAX_ROTV)

        self.RMS = 0
        self.counter = 0
       
    def pose_cb(self, data):
        """PoseStamped msg"""
        self.first_measurement1 = True

        self.x_mv = data.pose.position.x
        self.y_mv = data.pose.position.y
        self.z_mv = data.pose.position.z

        self.qx = data.pose.orientation.x
        self.qy = data.pose.orientation.y
        self.qz = data.pose.orientation.z
        self.qw = data.pose.orientation.w

    def vel_cb(self, data):
        """TwistStamped msg"""
        self.first_measurement2 = True

        self.vx_mv = data.twist.linear.x
        self.vy_mv = data.twist.linear.y
        self.vz_mv = data.twist.linear.z

        self.p = data.twist.angular.x
        self.q = data.twist.angular.y
        self.r = data.twist.angular.z


    def convert_to_euler(self, qx, qy, qz, qw):
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

        #while not self.first_measurement1 and not self.first_measurement2:
            #print("LaunchBebop.run() - Waiting for first measurement.")
            #rospy.sleep(self.sleep_sec)

        print("LaunchBebop.run() - Starting position control")
        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():
            self.rate.sleep()
            
            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            if dt < 1.0/self.controller_rate:
                continue
            
            self.convert_to_euler(self.qx, self.qy, self.qz, self.qw)

            # HEIGHT CONTROL
            self.z_filt_sp = prefilter(self.pose_sp.z, self.z_filt_sp, self.filt_const_z)
            vz_sp = self.pid_z.compute(self.z_filt_sp, self.z_mv, dt)
            
            # PITCH CONTROL OUTER LOOP
            # x - position control
            self.x_filt_sp = prefilter(self.pose_sp.x, self.x_filt_sp, self.filt_const_x)
            pitch_sp = self.pid_x.compute(self.pose_sp.x, self.x_mv, dt)

            # ROLL CONTROL OUTER LOOP
            # y position control
            self.y_filt_sp = prefilter(self.pose_sp.y, self.y_filt_sp, self.filt_const_y)
            roll_sp = - self.pid_y.compute(self.pose_sp.y, self.y_mv, dt)
    
            # PITCH AND ROLL YAW ADJUSTMENT
            roll_sp_2 = math.cos(self.euler_mv.z) * roll_sp + \
                        math.sin(self.euler_mv.z) * pitch_sp
            pitch_sp = math.cos(self.euler_mv.z) * pitch_sp - \
                       math.sin(self.euler_mv.z) * roll_sp
            roll_sp = roll_sp_2
            
            # YAW CONTROL
            error_yrc = self.euler_sp.z - self.euler_mv.z
            if math.fabs(error_yrc) > math.pi:
                self.euler_sp.z = (self.euler_mv.z/math.fabs(self.euler_mv.z))*\
                                  (2*math.pi - math.fabs(self.euler_sp.z))
            rot_v_sp = self.yaw_PID.compute(self.euler_sp.z, self.euler_mv.z, dt)
            
            self.twist_msg.linear.x = pitch_sp / MAX_TILT
            self.twist_msg.linear.y = - roll_sp / MAX_TILT
            self.twist_msg.linear.z = vz_sp / MAX_VZ
            self.twist_msg.angular.z = rot_v_sp / MAX_ROTV
            self.RMS += math.sqrt((self.pose_sp.x - self.x_mv)**2 + (self.pose_sp.y - self.y_mv)**2 \
                                  + (self.pose_sp.z - self.z_mv)**2)
            self.counter += 1
            if VERBOSE:

                print("x_sp: {}\n x_mv: {}\n y_sp: {}\n y_mv: {}\n z_sp: {}\n z_mv: {}\n".format(self.pose_sp.x, self.x_mv,
                                                                                                 self.pose_sp.y, self.y_mv,
                                                                                                 self.pose_sp.z, self.z_mv))
                print("Yaw measured: {}\n ".format(self.euler_mv.z))
                print("Pitch setpoint: {}".format(pitch_sp))
                print("Roll setpoint: {}".format(roll_sp))
                print("Yaw setpoint: {}".format(rot_v_sp))
                print("lin_vel command: {}".format(self.twist_msg.linear))
                print("ang_vel command: {}".format(self.twist_msg.angular))
                print("RMS : {}".format(self.RMS/self.counter))
            
            self.vel_publisher.publish(self.twist_msg)
            self.pose_pub.publish(self.pose_sp)
            self.rms_pub.publish(Float64(self.RMS))
            
            
    def angle_cb(self, data):
        self.euler_sp = Vector3(data.x, data.y, data.z)
    
    
    def setpoint_cb(self, data):
        self.pose_sp.x = data.x
        self.pose_sp.y = data.y
        self.pose_sp.z = data.z


def prefilter(x_k, x_k_1, a):
    """
    First order filter.
    (1 - a) * xK-1 + a * xK

    :param x_k: Current value
    :param x_k_1: Previous value
    :param a: Filter constant
    :return:
    """
    return (1 - a) * x_k_1 + a * x_k


if __name__ == "__main__":
    rospy.init_node('bebop_launch', anonymous=True)
    try:
        launch_bebop = RealFlight()
        launch_bebop.run()
    except rospy.ROSInterruptException:
        pass
