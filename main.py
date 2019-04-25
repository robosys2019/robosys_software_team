import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import seaborn as sns
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import signal

from listener import KinectSubscriber
from map_maker import MapMaker
from marker_detector import MarkerDetector

import time

run = True

class RoboSys:
    def __init__(self):
        self.listener = KinectSubscriber()
        self.map_maker = MapMaker()
        self.marker_detector = MarkerDetector()
        self.map_data = []
        self.rgb_data = []
    
    def signal_handler(self, signal, frame):
        global run
        run = False
        print("Shutting down")
        cv2.destroyAllWindows()
    
    def run(self):
        rospy.init_node('KinectSubscriber', anonymous=True)
        signal.signal(signal.SIGINT, self.signal_handler)
        i = 0

        while(run):
            self.map_data = self.listener.get_map_data()
            self.rgb_data = self.listener.get_rgb_data()

            # Depth Data
            # bar = np.zeros((424, 121))
            # img = np.hstack((bar, self.map_data, bar))
            # print(img.shape)
            # plt.imshow(img)
            # plt.show()

            # # center = cv_image.shape[1]/2
            # # cv_image = cv_image[:, center-1304/2:center+1304/2]
            # self.rgb_data = cv_image
            # print(self.rgb_data.shape)
            # # plt.imshow(self.rgb_data)
            # # plt.show()
            

            # self.map_maker.set_map(self.map_data)
            # self.map_maker.plot_all()
            # if self.rgb_data != []:
            #     sns.heatmap(self.map_data, cmap="YlGnBu", vmin=0, vmax=2000)
            #     plt.show()         
                # self.marker_detector.detect_markers(self.rgb_data)

            cv2.imshow("RGB Image Window", self.rgb_data)
            cv2.waitKey(3)

if __name__ == '__main__':
    rb = RoboSys()
    rb.run()