import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import seaborn as sns
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from listener import KinectSubscriber
from map_builder import MapBuilder


class RoboSys:
    def __init__(self):
        self.listener = KinectSubscriber()
        self.map_builder = MapBuilder()

    def listen(self):
        try:
            rospy.init_node('KinectSubscriber', anonymous=True)
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()
    
    def run(self):
        self.listen()
        self.map_builder.plot_map(self.listener.get_map_data())
        

if __name__ == '__main__':
    rb = RoboSys()
    rb.run()