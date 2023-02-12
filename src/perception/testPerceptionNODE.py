#!/usr/bin/env python3

import sys
sys.path.append('.')
import rospy
import time
from sensor_msgs.msg import Image
from utils.msg import lane, perception
import time
class testPerceptionNODE():
    """Don't explicitly define the tet value
        because we are not going to do that further.
    """
    def __init__(self) -> None:
        rospy.init_node("testPerceptionNODE", anonymous = False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", perception, self.testTopicLane)
        self.lane_info_subcriber = rospy.Subscriber("/automobile/lane_info",lane, self.testTopicLaneInfo)
        self.BEV_subscriber = rospy.Subscriber("/automobile/bev", Image, self.testBEV)
        self.lane_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._lane)
    def run(self):       
        rospy.loginfo("Hello from testPerceptionNODE")
        rospy.spin()  
    def testTopicLane(self, msg):
        #   Use to test the lane_publisher publisher
        #   lane_publisher is located at src/perception/perceptionNODE.py
        #while not rospy.is_shutdown():
        print(type(msg))
        time.sleep(1)
    def testTopicLaneInfo(self, msg):
        #   Use to test the lane_info_publisher publisher
        #   lane_info_publisher is located at src/perception/perceptionNODE.py
        print(type(msg))
        time.sleep(1)
    def testBEV(self,msg):
        print(type(msg))
        time.sleep(1)
    def _lane(self,msg):
        print(type(msg))
        time.sleep(1)
        
        

if __name__ == "__main__":
    testNODE = testPerceptionNODE()
    testNODE.run()