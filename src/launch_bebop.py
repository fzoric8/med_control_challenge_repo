import rospy
from std_msgs.msg import *
from mav_msgs.msg import Actuators



class LaunchBebop():

    def __init__(self):
        """Constructor initializes all needed variables"""
        self.u1, self.u2, self.u3, self.u4 = 0, 0, 0, 0
        self.angle1, self.angle2, self.angle3, self.angle4 = 0, 0, 0, 0

    def control_motor_speeds(self):
        """TO DO: controller for motor speeds based on some reference"""


    def publish_motor_speeds(self):
        """Create publisher and bebop launch node,
         send Actuators message to motor_speed command topic"""

        # init publisher
        pub = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=10)
        rospy.init_node('bebop_launch', anonymous=True)

        # check Rate for starters 20
        rate = rospy.Rate(20)

        # for starters init control vectors as zeros
        control_vector = [self.u1, self.u2, self.u3, self.u4]
        angle_control_vector = [self.angle1, self.angle2, self.1angle3, self.angle4]

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
    try:
        LB = LaunchBebop()
        LB.publish_motor_speeds()
    except rospy.ROSInterruptException:
        pass



