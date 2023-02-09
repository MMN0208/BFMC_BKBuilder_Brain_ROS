#!/usr/bin/env python3

import sys
sys.path.append('.')
import rospy
import time
from utils.msg import lane

class testPerceptionNODE():
    def __init__(self) -> None:
        rospy.init_node("testPerceptionNODE", anonymous = False)
        self.lane_subcriber = rospy.Subscriber("automobile/lane",lane,self._test)
    def run(self):
        self._read()
    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("Hello from test perception")
                time.sleep(2)
            except UnicodeDecodeError:
                pass
    def _test(self,msg):
        print(msg)        

if __name__ == "__main__":
    testNODE = testPerceptionNODE()
    testNODE.run()