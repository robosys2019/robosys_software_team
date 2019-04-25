"THIS IS CHARLIE'S COPY FOR TESTING PURPOSES"

import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import seaborn as sns
import numpy as np

#import rospy
#from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image

import signal

#from listener import KinectSubscriber
from map_maker import MapMaker
from marker_detector import MarkerDetector
from path_planner import PathPlanner, Node

run = True

class RoboSys:
    def __init__(self):
        #self.listener = KinectSubscriber()
        self.map_maker = MapMaker()
        #self.map_maker.set_map()
        #self.map_data = self.map_maker.get_lowres_map()
        self.path_planner = PathPlanner()
        #self.marker_detector = MarkerDetector()
        #self.rgb_data = []
    
    def signal_handler(self, signal, frame):
        global run
        run = False
        print("Shutting down")
        cv2.destroyAllWindows()
    
    def run(self):
        #rospy.init_node('KinectSubscriber', anonymous=True)
        #signal.signal(signal.SIGINT, self.signal_handler)

        #while(run):
            #self.map_data = self.listener.get_map_data()
            #self.rgb_data = self.listener.get_rgb_data()
            
            #self.marker_detector.detect_markers(self.rgb_data)

            # cv2.imshow("RGB Image Window", detected)
            # cv2.waitKey(3)

        self.map_maker.set_map() # setting to default map in map_maker, can pass other map live
        lowres_map = self.map_maker.get_lowres_map()

        self.path_planner.set_map(lowres_map)
        self.path_planner.set_start_node() # setting to default
        self.path_planner.set_end_node() # setting to default

        self.path_planner.run()

        #sns.heatmap(self.map_data, cmap="YlGnBu")
        #plt.show()

if __name__ == '__main__':
    rb = RoboSys()
    rb.run()