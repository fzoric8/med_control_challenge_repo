#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory

class BebopTrajectory:

    def __init__(self):

        # True if node recieved bebop trajectory, otherwise false
        self.trajectory_recieved = False

    def run(self):
        pass


if __name__ == '__main__':
    rospy.init_node("bebop_trajectory")
    try:
        bebop_trajectory = BebopTrajectory()
        bebop_trajectory.run()
    except rospy.ROSInterruptException:
        pass
