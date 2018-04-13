#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import scipy
from trajectory_msgs.msg import MultiDOFJointTrajectory

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
        self.hover_speed = math.sqrt(4.905 / self.bf / 4)

        self.arm = self.l * math.cos(math.pi / 4)

        # True if node recieved bebop trajectory, otherwise false
        self.trajectory_recieved = False
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

        # define vector for measured and setopint values
        self.pose_sp = Vector3(0., 0., 0.)
        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)
        self.p = 0
        self.q = 0
        self.r = 0
        self.t_old = 0

        # define PID for height control
        self.z_mv = 0

    def dlqr(self, Q, R):
        """Solve the discrete time lqr controller.

        x[k+1] = A x[k] + B u[k]

        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
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

        A = [[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
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
             [0, 0, 0, 0, 0, 0, 0, gama17, 0, gama18, 0, 0]]

        B = [[0, 0, 0, 0],
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
             [0, 0, 0, b3]]

        # first, try to solve the Ricatti equation
        X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

        # compute the LQR gain
        K = np.matrix(scipy.linalg.inv(B.T * X * B + R) * (B.T * X * A))

        eigVals, eigVecs = scipy.linalg.eig(A - B * K)

        return K, X, eigVals

    def run(self):
        pass


if __name__ == '__main__':
    rospy.init_node("bebop_trajectory")
    try:
        bebop_trajectory = BebopTrajectory()
        bebop_trajectory.run()
    except rospy.ROSInterruptException:
        pass
