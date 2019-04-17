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
<<<<<<< HEAD
from map_maker import MapMaker
=======
from marker_detector import MarkerDetector
>>>>>>> c0e68cd915d8dd173bc31d00acc51458501d754a

run = True

class RoboSys:
    def __init__(self):
        self.listener = KinectSubscriber()
<<<<<<< HEAD
        #self.map_builder = MapBuilder()
        self.map_maker = MapMaker()

    def listen(self):
        try:
            rospy.init_node('KinectSubscriber', anonymous=True)
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()
=======
        self.map_builder = MapBuilder()
        self.marker_detector = MarkerDetector()
        self.map_data = []
        self.rgb_data = []
    
    def signal_handler(self, signal, frame):
        global run
        run = False
        print("Shutting down")
        cv2.destroyAllWindows()
>>>>>>> c0e68cd915d8dd173bc31d00acc51458501d754a
    
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