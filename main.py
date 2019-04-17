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
from map_builder import MapBuilder
from marker_detector import MarkerDetector

run = True

class RoboSys:
    def __init__(self):
        self.listener = KinectSubscriber()
        self.map_builder = MapBuilder()
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

        while(run):
            self.map_data = self.listener.get_map_data()
            self.rgb_data = self.listener.get_rgb_data()
            
            self.marker_detector.detect_markers(self.rgb_data)

            # cv2.imshow("RGB Image Window", detected)
            # cv2.waitKey(3)

if __name__ == '__main__':
    rb = RoboSys()
    rb.run()