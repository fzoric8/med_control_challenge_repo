#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import cv2
from nav_msgs.msg import Odometry
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
        self.normal = Vector3(0., 0., 0.)

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

        # Plane of rotation normal publisher
        self.normal_pub = rospy.Publisher(
            "bebop/plane_of_rotation_normal",
            Vector3,
            queue_size=10)

        # Odometry subscriber
        self.odom_subscriber = rospy.Subscriber(
            "bebop/odometry",
            Odometry,
            self.odometry_callback)

        self.angle_sub = rospy.Subscriber(
            "bebop/angle_ref",
            Vector3,
            self.angle_cb)
        self.img_saved = False

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

    def angle_cb(self, data):
        self.ref_yaw = data.z

    def odometry_callback(self, data):
        """Callback function for odometry subscriber"""

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

    def get_current_yaw(self):
        """
        Get current bebop position
        """
        # Get current position
        _, _, self.curr_yaw, _, _, _ = self.quaternion2euler(
            self.qx, self.qy, self.qz, self.qw)

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

        # Wait until first image is published
        while not self.first_image_captured:
            rospy.sleep(2)

        print("CameraProcessing.run() - Flight control - Start")
        self.fc_pub.publish(self.fc_msg)
        print("CameraProcessing.run()")
        rospy.sleep(5)
        normal_found = False
        detect_count = 0
        while not rospy.is_shutdown() and not normal_found:

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

            img = self.draw_hough_lines(lines, decoded_image)

            if self.avg_theta > 1e-1:
                self.fc_msg.z = 0
            else:
                self.fc_msg.z = 0.5
            self.fc_pub.publish(self.fc_msg)

            if self.avg_theta < 1e-6:
                detect_count += 1
                print("DETECT COUNT {}".format(detect_count))

            if detect_count >= 8:
                print("CameraProcessing.run() - Flight control - Stop")
                # Signal to flight control to stop
                self.fc_msg.y = 0
                self.fc_pub.publish(self.fc_msg)
                rospy.sleep(5)  # To stabilize yaw

                avg_yaw = 0
                for i in range(100):
                    self.get_current_yaw()
                    #print(self.curr_yaw)
                    avg_yaw += self.curr_yaw
                    rospy.sleep(0.01)

                avg_yaw /= 100
                self.curr_yaw = avg_yaw
                r_count, l_count = 0, 0
                voting_count = 3
                if not self.img_saved:
                    #self.get_current_yaw()
                    self.img_saved = True
                    for i in range(voting_count):
                        # capture different picture for voting process (wrong blade position misperception)
                        decoded_image = cv2.imdecode(self.img_array, cv2.COLOR_BGR2GRAY)
                        edges = cv2.Canny(decoded_image, 130, 200, apertureSize=3)
                        lines = cv2.HoughLines(edges, 2, np.pi / 180, 200)
                        img = self.draw_hough_lines(lines, decoded_image)
                        side = self.get_normal(decoded_image)
                        if side == "r":
                            r_count +=1
                        else:
                            l_count +=1
                    if r_count > l_count:
                        self.calculate_normal("r")
                    else:
                        self.calculate_normal("l")
                    print("Side: {}\nNormal: {}\n".format(side, self.normal))
                    self.normal_pub.publish(self.normal)
                    normal_found = True

            # print("CameraProcessing.run() - found lines {}".format(lines.shape[0]))

            # Create published image
            #msg = CompressedImage()
            #msg.header.stamp = rospy.Time.now()
            #msg.format = "jpeg"
            #msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

            # Publish new image
            #self.image_pub.publish(msg)

    def get_normal(self, img):

        pix_top = 0
        pix_mid = 0
        side = ''

        # changing RGB to BW
        # Adding median blur to photo
        blur = cv2.medianBlur(img, 9)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # Applying threshold to images
        th = cv2.bitwise_not(cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2))

        # searching for horizon y pixel and top windmill x pixel
        for i in range(200, 800):
            if not pix_top:
                if th[i, :].any():
                    for j in range(500, 1500):
                        if th[i, j]:
                            pix_top = j

            if th[i, 0:30].any():
                pix_mid = i - 30
                break

        for i in range(pix_top - 200, pix_top):
            i_2 = 2 * pix_top - i
            if th[(pix_mid - 100):pix_mid, i].any():
                side = 'r'
                break
            elif th[(pix_mid - 100):pix_mid, i_2].any():
                side = 'l'
                break
        #print("PIX MID: {}\nPIX TOP: {}\nSIDE: {}\nNORMAL: {}".format(
            #pix_mid, pix_top, side, self.normal))

        return side

    def calculate_normal(self, side):

        windmill_yaw_correction = math.pi/2
        if side == "r":
            windmill_yaw_correction *= -1

        windmill_yaw = self.curr_yaw + windmill_yaw_correction
        self.normal.x = np.cos(windmill_yaw)
        self.normal.y = np.sin(windmill_yaw)

        print("DRONE YAW: {}".format(self.curr_yaw))

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

            #a = np.cos(theta)
            #b = np.sin(theta)
            #x0 = a*rho
            #y0 = b*rho
            #x1 = int(x0 + 2000*(-b))
            #y1 = int(y0 + 2000*(a))
            #x2 = int(x0 - 2000*(-b))
            #y2 = int(y0 - 2000*(a))

            #cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)

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


