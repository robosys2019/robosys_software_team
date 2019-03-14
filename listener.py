#!/usr/bin/env python
import rospy

# from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class kinect_subscriber:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/depth_undistorted", Image, self.callback)

    def callback(self, data):
        # rospy.loginfo("[Subscriber] " + rospy.get_caller_id() + ": I heard %s", data)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    ic = kinect_subscriber()

    rospy.init_node('kinect_subscriber', anonymous=True)

    

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()