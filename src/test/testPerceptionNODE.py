#!/usr/bin/env python3

import sys
sys.path.append('.')
import rospy
import time
from utils.msg import lane

class testPerceptionNODE():
    """Don't explicitly define the tet value
        because we are not going to do that further.
    """
    def __init__(self) -> None:
        rospy.init_node("testPerceptionNODE", anonymous = False)
    
    def testTopicLane(self, msg):
        #   Use to test the lane_publisher publisher
        #   lane_publisher is located at src/perception/perceptionNODE.py
        pass

    def testTopicLaneInfo(self, msg):
        #   Use to test the lane_info_publisher publisher
        #   lane_info_publisher is located at src/perception/perceptionNODE.py
        pass
        

if __name__ == "__main__":
    testNODE = testPerceptionNODE()
    testNODE.run()