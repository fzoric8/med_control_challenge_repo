import rospy, roslib
import sys, time
import numpy as np
import math
import cv2
from launch_bebop import LaunchBebop
from sensor_msgs.msg import CompressedImage, LaserScan

VERBOSE = True

class CameraProcessing:

    def __init__(self):
        """Initialize ros publisher, ros subscriber"""

        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
        self.first_image_captured = False

        #subscribed Topic
        self.subscriber = rospy.Subscriber("/bebop/camera1/image_raw/compressed",
                                           CompressedImage, self.image_cb)
        self.laser_subscriber = rospy.Subscriber("/laser/scan", LaserScan, self.laser_cb)
        if VERBOSE:
            print("Subscribed to /bebop/camera1/image_raw/compressed")

        RATE = 50

    def laser_cb(self, laser_msg):
        self.range = laser_msg.ranges[0]

    def image_cb(self, ros_data):
        """ Callback function of subscribed topic 
        Here images get converted and features detected
        :param ros_data: 
        :return: 
        """

        self.first_image_captured = True

        if VERBOSE:
            print("Received image of type: %s" %ros_data.format)

        ### direct conversion to CV2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv2.imshow('cv_img', image_np)
        #cv2.waitKey(1)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

    def run(self):
        rospy.Rate(20)

        while not self.first_image_captured:
            rospy.sleep(2)

        rospy.spin()
        print(self.range)

if __name__=='__main__':
    rospy.init_node('camera_launch', anonymous=True)
    try:
        CP = CameraProcessing()
        CP.run()
    except rospy.ROSInterruptException:
        #cv2.destroyAllWindows()
        pass


