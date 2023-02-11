#!/usr/bin/env python3

import cv2
import sys
sys.path.append('../') 
import rospy
from enum import Enum
import time
from utils.msg import TrafficSign
from control.controlNODE import controlNODE

class trafficsignNODE():
    def traffic_sign_processing(self, msg):
        traffic_sign = msg.traffic_sign_type

        if traffic_sign == 6: #detech a STOP sign
            print("Stop at least 3 seconds") 
            self.control.brake(0) # ???
            start_time = time.time()
            while((time.time() - start_time) > 3):
                pass   

        elif traffic_sign == 3: #packing place
            print("pass")
            #call parking funtion

        elif traffic_sign == 0: #crosswalk
            print("Slow down")
            #controlNODE().brake(0)
            #sleep(1)
            self.control.setSpeed(0.1)
            #after pass the crosswalk -> then what(go straight)
            self.control.setSpeed(0.3, 0.3)

        elif traffic_sign == 4: #priority road
            print("Do not stop at intesection")
            #intersction -> cross walk -> 1: move forward, 2: turn right/left
            self.control.moveForward(0.3, 0.3)


        elif traffic_sign ==  2: #one way road
            print("go straight")
            self.control.setSpeed(0.3)

        elif traffic_sign == 1: #no entry road
            print("turn other way")
            self.control.setSteer(23)
            self.control.moveForward(0.3, 0.3)
            self.control.setSteer(0)
            self.control.setSpeed(0.3)

        elif traffic_sign == 7 #roundabout
            self.control.setSteer(23)
            self.control.moveForward(0.2,0.1)
            self.control.setSteer(-23)
            self.control.moveForward(0.2,0.1)
            self.control.setSteer(10)
            self.control.moveForward(0.2,0.1)
            self.control.setSteer(0)
            self.control.setSpeed(0.3)

        

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
            
            