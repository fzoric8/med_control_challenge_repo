import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, LaserScan

VERBOSE = False
SAVE_DIR = "/home/lmark/Desktop/windmill_photos"


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

        # Subscribed Topic
        self.subscriber = rospy.Subscriber(
            "/bebop/camera1/image_raw/compressed",
            CompressedImage,
            self.compressed_image_cb)

        self.laser_subscriber = rospy.Subscriber(
            "/laser/scan",
            LaserScan,
            self.laser_cb)

        if VERBOSE:
            print("Subscribed to /bebop/camera1/image_raw/compressed")

        # Image processing rate
        self.cam_rate = 20
        self.rate = rospy.Rate(self.cam_rate)

        # Image array being processed
        self.img_array = []

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

        while not rospy.is_shutdown():

            # little sleepy boy
            self.rate.sleep()
            #print(self.np_arr.shape)
            counter += 1

            # image processing
            image_np = cv2.imdecode(self.img_array, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(image_np, 130, 200, apertureSize=3)
            gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            gray = cv2.medianBlur(gray, 5)

            # HOUGH LINES
            # lines = cv2.HoughLines(edges, 2, np.pi/180, 150)
            # img = self.draw_HoughImage(lines, image_np)
            # print(lines.shape)

            # HOUGH CIRCLES -> doesen't detect them
            # rows = gray.shape[0]
            # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows/8,
            #                           param1=100, param2=30,
            #                            minRadius=1, maxRadius=50)

            # CORNER DETECTION -> few corners and maybe try to connect them 
            corners = cv2.goodFeaturesToTrack(gray, 3, 0.01, 10)
            corners = np.int0(corners)

            for i in corners:
                x, y = i.ravel()
                cv2.circle(image_np, (x, y), 3, 255, -1)
            cv2.imwrite("{0}/photo_{1}.png".format(SAVE_DIR, counter), gray)

            # img = self.drawHoughCircle(circles, gray)
            cv2.imwrite("{0}/hough_{1}.png".format(SAVE_DIR, counter), image_np)

            # Create published image
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

            # Publish new image
            self.image_pub.publish(msg)


def draw_hough_image(lines, img):
    """
    Draw Hough lines on the given image

    :param lines: Line array.
    :param img: Given image.

    :return: Image with drawn lines
    """
    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 2000*(-b))
        y1 = int(y0 + 2000*(a))
        x2 = int(x0 - 2000*(-b))
        y2 = int(y0 - 2000*(a))

        cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)

    return img


def draw_hough_circles(circles, img):
    #print(circles)
    if circles is not None:
        circles = np.uint(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(img, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(img, radius, (255, 0, 255), 3)
    return img


if __name__=='__main__':
    rospy.init_node('camera_launch', anonymous=True)
    try:
        CP = CameraProcessing()
        CP.run()
    except rospy.ROSInterruptException:
        pass


