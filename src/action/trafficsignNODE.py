#!/usr/bin/env python3

import cv2
import sys
sys.path.append('.') 
import rospy
from enum import Enum
import time
from utils.msg import TrafficSign

class trafficsignNODE():
    def traffic_sign_processing(self, msg):
        traffic_sign = msg.traffic_sign_type
        if traffic_sign == 6: #detech a STOP sign
            print("Stop at least 3 seconds")
        elif traffic_sign == 3: #packing place
            print("pass")
        elif traffic_sign == 0: #crosswalk
            print("Slow down")
        elif traffic_sign == 4: #priority walk
            print("Do not stop at intesection")
        elif traffic_sign ==  2: #one way road
            print("go straight")
        elif traffic_sign == 1:
            print("turn other way")

    def __init__(self): 
        rospy.init_node('trafficsignNODE', anonymous = False)
        self.trafficSign_subscriber = rospy.Subscriber("/automobile/traffic_sign", TrafficSign, self.traffic_sign_processing)
        self.unlock = 1
        self.lock = 0

    def run(self):
        rospy.loginfo('starting trafficsignNODE')
        self._read()
        
    def _read(self):
        while not rospy.is_shutdown():
            try: 
                print("trafficSign Hello")
                time.sleep(2)
            except UnicodeDecodeError:
                pass

    def lock_state():
        if self.unlock:
            self.lock = 1
            self.unlock = 0
            self.time_lock_start = rospy.get_time()

    def unlock_state(self, state, time):
        if self.lock:
            if (rospy.get_time() - self.time_lock_start) < time:
                self.run_state = state
            else: 
                self.unlock = 1
                self.lock = 0


if __name__ == "__main__":
    perNod = trafficsignNODE()
    perNod.run()
            
            