import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import seaborn as sns
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class KinectSubscriber:
    def __init__(self, show = False):
        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber("/camera/depth_undistorted", Image, self.depth_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.rgb_callback, queue_size=1)
        self.map_data = []
        self.rgb_data = []
        self.show = show
    
    def get_map_data(self):
        return self.map_data
    
    def get_rgb_data(self):
        return self.rgb_data

    def depth_callback(self, data):
        encoding = "passthrough"
        # encoding = "16UC1"
        # print("[KinectSubscriber] New depth message. Encoding used:" + encoding)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, encoding)
            self.map_data = cv_image.copy()
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(3)
    
    def rgb_callback(self, data):
        encoding = "bgr8"
        # print("[KinectSubscriber] New rgb message. Showing image with encoding: " + encoding)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, encoding)
            self.rgb_data = cv_image.copy()
        except CvBridgeError as e:
            print(e)

        if self.show:
            cv2.imshow("RGB Image Window", cv_image)
            cv2.waitKey(3)

if __name__ == '__main__':
    ic = KinectSubscriber(show=False)
    rospy.init_node('kinect_subscriber', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

