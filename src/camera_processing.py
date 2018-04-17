import rospy, roslib
import sys, time
import numpy as np
import math
import cv2
from launch_bebop import LaunchBebop
from sensor_msgs.msg import CompressedImage, LaserScan

VERBOSE = False


class CameraProcessing:

    def __init__(self):
        """Initialize ros publisher, ros subscriber"""

        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage,
                                         queue_size=1)
        self.first_image_captured = False

        # Subscribed Topic
        self.subscriber = rospy.Subscriber("/bebop/camera1/image_raw/compressed",
                                           CompressedImage,
                                           self.image_cb)

        self.laser_subscriber = rospy.Subscriber("/laser/scan",
                                                 LaserScan,
                                                 self.laser_cb)
        if VERBOSE:
            print("Subscribed to /bebop/camera1/image_raw/compressed")

        self.cam_rate = 20
        self.rate = rospy.Rate(self.cam_rate)

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
        self.np_arr = np.fromstring(ros_data.data, np.uint8)


    def draw_HoughImage(self, lines, img):
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 2000*(-b))
            y1 = int(y0 + 2000*(a))
            x2 = int(x0 - 2000*(-b))
            y2 = int(y0 - 2000*(a))

            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
        return(img)



    def run(self):


        while not self.first_image_captured:
            rospy.sleep(2)

        while not rospy.is_shutdown():
            # little sleepy boy
            self.rate.sleep()
            print(self.np_arr.shape)

            # image procssing
            image_np = cv2.imdecode(self.np_arr, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(image_np, 130, 200, apertureSize=3)
            lines = cv2.HoughLines(edges, 1, 1, 20)
            img = self.draw_HoughImage(lines, image_np)

            # Create published image
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

            # Publish new image
            self.image_pub.publish(msg)



if __name__=='__main__':
    rospy.init_node('camera_launch', anonymous=True)
    try:
        CP = CameraProcessing()
        CP.run()
    except rospy.ROSInterruptException:
        #cv2.destroyAllWindows()
        pass


