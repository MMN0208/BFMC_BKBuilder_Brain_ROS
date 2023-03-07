#!/usr/bin/env python3
import cv2
import math
        
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg    import String, Byte
from utils.msg       import *
from enum            import Enum

class cvshowNODE:
    def __init__(self) -> None:
        rospy.init_node('cvshowNODE', anonymous=False)
        self.show_image = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        cv2.namedWindow("Image Window", 1)

        
    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)
        
    # Define a callback for the Image message
    def image_callback(self, img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)

        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Show the converted image
        #self.show_image(cv_image)
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

        
    def run(self):
        global traffic_light_id
        global light_color
        # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    cvshow_NODE = cvshowNODE()
    cvshow_NODE.run()