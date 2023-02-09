#!/usr/bin/env python3

import sys
sys.path.append('.')
import rospy
import time
from utils.msg import lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class testPerceptionNODE():
    def __init__(self) -> None:
        rospy.init_node("testPerceptionNODE", anonymous = False)
        self.lane_subcriber = rospy.Subscriber("automobile/lane",lane,self._testmsg)
        self.image_subcriber = rospy.Subscriber("automobile/birdeyes_view",Image,self._testImage)
        self.bridge = CvBridge()
    def run(self):
        self._read()
    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("Hello from test perception")
                time.sleep(2)
            except UnicodeDecodeError:
                pass
    def _testmsg(self,msg):
        print(msg)        
    def _testImage(self,image):
        while not rospy.is_shutdown():
            cv2_img = self.bridge.imgmsg_to_cv2(image)
            print(type(cv2_img))
           # cv2.imshow('image',cv2_img)
            #cv2.waitKey(0)
            status = cv2.imwrite("/receive_image.jpg",cv2_img)
            print(status)
            time.sleep(2)
        
        

if __name__ == "__main__":
    testNODE = testPerceptionNODE()
    testNODE.run()