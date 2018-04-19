#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


class CameraProcessing:
    """
    Class used for detecting turbine rotor plane normal vector.
    """

    def __init__(self):
        """
        Initialize ros publisher, ros subscriber
        """

        # Output image publishers
        self.image_pub = rospy.Publisher(
            "/output/image_raw/compressed",
            CompressedImage,
            queue_size=1)
        self.first_image_captured = False

        # Error publisher
        self.error_pub = rospy.Publisher(
            "/bebop/windmill_error",
            Float64,
            queue_size=1)

        # Subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/bebop/camera1/image_raw/compressed",
            CompressedImage,
            self.compressed_image_cb)

        # Flight control publisher
        self.fc_pub = rospy.Publisher(
            "bebop/flight_control",
            Vector3,
            queue_size=10)
        self.fc_msg = Vector3()
        self.fc_msg.x = 1
        self.fc_msg.y = 1
        self.fc_msg.z = 0

        # Image processing rate
        self.cam_rate = 50
        self.rate = rospy.Rate(self.cam_rate)

        # Image array being processed
        self.img_array = []
        self.avg_theta = 1000

    def laser_cb(self, laser_msg):
        self.range = laser_msg.ranges[0]

    def compressed_image_cb(self, ros_data):
        """
        Callback function of subscribed topic
        Here images get converted and features detected

        :param ros_data:
        """
        self.first_image_captured = True

        # direct conversion to CV2
        self.img_array = np.fromstring(ros_data.data, np.uint8)

    def run(self):
        """
        Run image processing loop.
        """

        counter = 0

        # Wait until first image is published

        while not self.first_image_captured:
            rospy.sleep(2)

        print("CameraProcessing.run() - Flight control - Start")
        self.fc_pub.publish(self.fc_msg)
        print("CameraProcessing.run()")
        rospy.sleep(5)

        while not rospy.is_shutdown():

            # little sleepy boy
            self.rate.sleep()

            # image processing
            decoded_image = cv2.imdecode(self.img_array, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(decoded_image, 130, 200, apertureSize=3)
            # TODO: Try different parameters
            lines = cv2.HoughLines(edges, 2, np.pi / 180, 200)

            # If no lines are found punish and continue
            if lines is None:
                print("CameraProcessing.run() - no lines found")
                self.avg_theta = 500
                self.error_pub.publish(self.avg_theta)
                continue

            if self.avg_theta < 10e-6:

                print("CameraProcessing.run() - Flight control - Stop")
                # Signal to flight control to stop
                self.fc_msg.y = 0
                self.fc_pub.publish(self.fc_msg)

            #print("CameraProcessing.run() - found lines {}".format(lines.shape[0]))
            img = self.draw_hough_lines(lines, decoded_image)

            # Create published image
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

            # Publish new image
            self.image_pub.publish(msg)

    def draw_hough_lines(self, lines, img):
        """
        Draw Hough lines on the given image

        :param lines: Line array.
        :param img: Given image.

        :return: Image with drawn lines
        """

        self.avg_theta = 0
        for line in lines:

            # Extract line info
            rho = line[0][0]
            theta = line[0][1]

            theta_temp = abs(theta - math.pi/2)
            self.avg_theta += (theta_temp - math.pi/2)**4

            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 2000*(-b))
            y1 = int(y0 + 2000*(a))
            x2 = int(x0 - 2000*(-b))
            y2 = int(y0 - 2000*(a))

            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)

        self.avg_theta /= len(lines)
        print("Current error: ", self.avg_theta)

        self.error_pub.publish(self.avg_theta)

        return img


if __name__=='__main__':
    rospy.init_node('camera_launch', anonymous=True)
    try:
        CP = CameraProcessing()
        CP.run()
    except rospy.ROSInterruptException:
        pass


