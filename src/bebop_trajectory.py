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

        # Bebop state-space model
        self.mass = 0.5  # kg --> mass of the quadcopter
        self.Ix = 0.00389  # kg m^2  --> Quadrotor moment of inertia in body x direction
        self.Iy = 0.00389  # kg m^2  --> Quadrotor moment of inertia in body y direction
        self.Iz = 0.0078  # kg m^2  --> Quadrotor moment of inertia in body z direction
        self.Tm = 0.0125  # s       --> Time constant of a motor
        self.bf = 8.548e-6  # kg m    --> Thrust constant of a motor
        self.bm = 0.016  # m       --> Moment constant of a motor
        self.l = 0.12905  # m       --> The distance of a motor from a center of mass
        self.gravity = 9.81  # m/s^2   --> Gravity value
        self.sleep_sec = 0.5  # sleep duration while not getting first measurement
        self.hover_speed = 400

        # Initial control values (u1, u2, u3, u4)
        self.u = np.array([5.45, 0, 0, 0])

        self.motor_speeds = self.u2w(self.u)

        self.arm = self.l * math.cos(math.pi / 4)

        # Define magic thrust number :-)
        self.magic_number = 0.908

        # True if node received bebop trajectory, otherwise false
        self.trajectory_received = False
        self.first_measurement = False
        self.controller_info = False

        self.odom_subscriber = rospy.Subscriber(
            "bebop/odometry",
            Odometry,
            self.odometry_callback)
        self.pose_subscriber = rospy.Subscriber(
            "bebop/pos_ref",
            Vector3,
            self.setpoint_cb)
        self.odom_gt_subscriber = rospy.Subscriber(
            "bebop/odometry_gt",
            Odometry,
            self.odometry_gt_callback)
        self.angle_subscriber = rospy.Subscriber(
            "bebop/angle_ref",
            Vector3,
            self.angle_cb)
        self.trajectory_subscriber = rospy.Subscriber(
            "/bebop/trajectory_reference",
            MultiDOFJointTrajectory,
            self.trajectory_cb)

        # initialize publishers
        self.motor_pub = rospy.Publisher(
            '/gazebo/command/motor_speed',
            Actuators,
            queue_size=10)
        self.error_pub = rospy.Publisher(
            '/bebop/pos_error',
            Float64,
            queue_size=10)
        self.motor_pub = rospy.Publisher(
            '/gazebo/command/motor_speed',
            Actuators,
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

        self.actuator_msg.angular_velocities = \
            [self.magic_number * self.motor_speeds[0], self.magic_number*self.motor_speeds[1],
             self.motor_speeds[2], self.motor_speeds[3]]

        # Controller rate
        # Trajectory points are being given with a frequency of 100Hz
        self.controller_rate = 50
        self.rate = rospy.Rate(self.controller_rate)
        self.t_old = 0

        # Initialize trajectory information
        self.trajectory_index = 0
        self.trajectory_point_count = -1
        self.trajectory_points = []

    def setpoint_cb(self, data):
        self.pose_sp.x = data.x
        self.pose_sp.y = data.y
        self.pose_sp.z = data.z

    def angle_cb(self, data):
        self.euler_sp = Vector3(data.x, data.y, data.z)

    def odometry_callback(self, data):
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

    def trajectory_cb(self, data):
        self.trajectory_received = True
        self.trajectory_point_count = len(data.points)
        self.trajectory_points = data.points

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

    def dlqr(self, Q, R):
        """
        Solve the discrete time lqr controller.

        x[k+1] = A x[k] + B u[k]

        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]

        :param Q: Standard deviation of x[k]
        :param R: Standard deviation of u[k]

        :return:
            K - Gain used for calculating thrust

        """
        phi = self.euler_mv.x
        theta = self.euler_mv.y
        psi = self.euler_mv.z

        p = self.p
        q = self.q
        r = self.r

        w1 = self.actuator_msg.angular_velocities[0]
        w2 = self.actuator_msg.angular_velocities[1]
        w3 = self.actuator_msg.angular_velocities[2]
        w4 = self.actuator_msg.angular_velocities[3]

        u1 = math.cos(phi) * math.cos(theta) * self.bf * (w1**2 + w2**2 + w3**2 + w4**2)

        c1 = (math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi) * math.sin(psi)) / self.mass
        c2 = (math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi) * math.cos(psi)) / self.mass
        c3 = (math.cos(phi) * math.cos(theta)) / self.mass

        self.arm = self.l * math.cos(math.pi / 4)

        b1 = self.arm / self.Ix
        b2 = self.arm / self.Iy
        b3 = self.arm / self.Iz

        a1 = (self.Iy - self.Iz) / self.Ix
        a2 = (self.Iz - self.Ix) / self.Iy
        a3 = (self.Ix - self.Iy) / self.Iz

        gama1 = (u1 * math.sin(psi)) / (self.mass * math.cos(phi)**2 * math.cos(theta))
        gama2 = (u1 * math.cos(psi)) / (self.mass * math.cos(theta)**2)
        gama3 = (u1 * math.tan(phi) * math.cos(psi)) / (self.mass * math.cos(theta))
        gama4 = -(u1 * math.cos(psi)) / (self.mass * math.cos(phi)**2 * math.cos(theta))
        gama5 = -(u1 * math.tan(phi) * math.sin(theta) * math.cos(psi)) / (self.mass * math.cos(theta)**2)
        gama6 = -(u1 * math.tan(theta) * math.cos(psi)) / self.mass
        gama7 = math.sin(phi) * math.tan(theta)
        gama8 = math.cos(phi) * math.tan(theta)
        gama9 = a1 * r
        gama10 = a1 * q
        gama11 = math.cos(phi)
        gama12 = -math.sin(phi)
        gama13 = a2 * r
        gama14 = a2 * p
        gama15 = math.sin(phi) / math.cos(theta)
        gama16 = math.cos(phi) / math.cos(theta)
        gama17 = a3 * q
        gama18 = a3 * p

        A = np.matrix([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, gama1, 0, gama2, 0, gama3, 0],
                      [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, gama4, 0, gama5, 0, gama6, 0],
                      [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0, 0, 1, 0, gama7, 0, gama8],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, gama9, 0, gama10],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, gama11, 0, gama12],
                      [0, 0, 0, 0, 0, 0, 0, gama13, 0, 0, 0, gama14],
                      [0, 0, 0, 0, 0, 0, 0, 0, 0, gama15, 0, gama16],
                      [0, 0, 0, 0, 0, 0, 0, gama17, 0, gama18, 0, 0]])

        B = np.matrix([[0, 0, 0, 0],
                      [c1, 0, 0, 0],
                      [0, 0, 0, 0],
                      [c2, 0, 0, 0],
                      [0, 0, 0, 0],
                      [c3, 0, 0, 0],
                      [0, 0, 0, 0],
                      [0, b1, 0, 0],
                      [0, 0, 0, 0],
                      [0, 0, b2, 0],
                      [0, 0, 0, 0],
                      [0, 0, 0, b3]])

        # first, try to solve the Ricatti equation
        X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

        # compute the LQR gain
        K = np.matrix(scipy.linalg.inv(B.T * X * B + R) * (B.T * X * A))

        return K

    def trajectory_tracking(self, point, angle, lin_vel, ang_vel):
        """
        This function perfomrs trajectory tracking based on the current
        quadrotor information and desired.

        :param point: Desired point transform.
        :param angle: Desired angle.
        :param lin_vel: Desired linear velocity.
        :param ang_vel: Desired angular velocity.

        :return:
            Returns thrust delta required to reach given point.
            As 4 values:
                delta_u1, delta_u2, delta_u3, delta_u4.
        """

        delta_pose = Vector3(self.x_mv - point.x, self.y_mv - point.y, self.z_mv - point.z)
        delta_angle = Vector3(self.euler_mv.x - angle.x, self.euler_mv.y - angle.y,
                              self.euler_mv.z - angle.z)
        delta_lin_vel = Vector3(self.vx_mv - lin_vel.x, self.vy_mv - lin_vel.y, self.vz_mv - lin_vel.z)
        delta_ang_vel = Vector3(self.p - ang_vel.x, self.q - ang_vel.y, self.r - ang_vel.z)

        delta_state = np.matrix([delta_pose.x, delta_lin_vel.x, delta_pose.y, delta_lin_vel.y,
                                 delta_pose.z, delta_lin_vel.z, delta_angle.x, delta_ang_vel.x,
                                 delta_angle.y, delta_ang_vel.y, delta_angle.z, delta_ang_vel.z])
        delta_state = delta_state.T

        # TODO: Finish calculating LQR control
        # LQR calculation in current state
        Q = 1000 * np.identity(12)
        R = np.identity(4)
        K = self.dlqr(Q, R)

        delta_u = np.dot(K, delta_state)
        array_u = np.array(delta_u.T)[0]

        return array_u

    def u2w(self, u):
        """
        Converts control inputs u1, u2, u3 and u4 into motor speeds

        :param u: control inputs
        :return: w: desired motor speeds
        """
        u_bare = [u[0]/self.bf, u[1]/(self.bf*self.l*0.7071),
                  u[2]/(self.bf*self.l*0.7071), u[3]/(self.bm*self.bf)]

        eq = [u_bare[0] - u_bare[1] - u_bare[2] - u_bare[3],
              u_bare[0] + u_bare[1] - u_bare[2] + u_bare[3],
              u_bare[0] + u_bare[1] + u_bare[2] - u_bare[3],
              u_bare[0] - u_bare[1] + u_bare[2] + u_bare[3]]

        w_eq = [0 if i < 0 else i for i in eq]

        w1 = math.sqrt(w_eq[0]) / 2
        w2 = math.sqrt(w_eq[1]) / 2
        w3 = math.sqrt(w_eq[2]) / 2
        w4 = math.sqrt(w_eq[3]) / 2

        w = np.array([w1, w2, w3, w4])

        return w

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
                pose = self.pose_sp
                angle = self.euler_sp
                lin_vel = Vector3(0., 0., 0.)
                ang_vel = Vector3(0., 0., 0.)
            else:
                current_point = self.trajectory_points[self.trajectory_index]
                pose = current_point.transforms[0].translation
                angle_q = current_point.transforms[0].rotation

                angle.x, angle.y, angle.z, _, _, _ = self.quaternion2euler(angle_q.x, angle_q.y, angle_q.z, angle_q.w)
                lin_vel = current_point.velocities[0].linear
                ang_vel = current_point.velocities[0].angular

                # Increase trajectory point index, check if finished
                self.trajectory_index += 1
                print("BebopTrajectory.run() - point {} / {}\ndt: {}"
                      .format(self.trajectory_index, self.trajectory_point_count, dt))
                if self.trajectory_index >= self.trajectory_point_count:
                    print("BebopTrajectory.run() -"
                          "Trajectory completed.")
                    self.trajectory_received = False
                    self.trajectory_index = 0

            delta_u = self.trajectory_tracking(pose, angle, lin_vel, ang_vel)

            # Print out controller information
            if self.controller_info:
                print(dt)
                print("Comparison x:{}\nx_m:{}\ny:{}\ny_m:{}\nz:{}\nz_m{}".format(
                    self.pose_sp.x,
                    self.x_mv,
                    self.pose_sp.y,
                    self.y_mv,
                    self.pose_sp.z,
                    self.z_mv))
                print("Motor speeds are {}".format(self.actuator_msg.angular_velocities))
                print("Current quadcopter height is: {}".format(self.z_mv))

            self.u = self.u + delta_u
            u_temp = self.u
            u_temp[0] = u_temp.item(0)/(math.cos(self.euler_mv.x) * math.cos(self.euler_mv.y))
            self.motor_speeds = self.u2w(u_temp)

            # TODO: Check if self.magic_number^TM is needed in LQR control

            self.actuator_msg.angular_velocities = \
                [self.magic_number * self.motor_speeds[0], self.magic_number * self.motor_speeds[1],
                 self.motor_speeds[2], self.motor_speeds[3]]

            print("Motor speeds are {}".format(self.actuator_msg.angular_velocities))

            # Publish Actuator message
            self.motor_pub.publish(self.actuator_msg)


if __name__ == '__main__':
    rospy.init_node("bebop_trajectory")
    try:
        bebop_trajectory = BebopTrajectory()
        bebop_trajectory.run()
    except rospy.ROSInterruptException:
        pass
