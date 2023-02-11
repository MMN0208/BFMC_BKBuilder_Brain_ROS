#!/usr/bin/env python3

import cv2
import sys
sys.path.append('../') 
import rospy
from enum import Enum
import time
from utils.msg import Parking
from control.controlNODE import controlNODE

class parkingNODE():
    def traffic_sign_parking_processing(self, msg):
        traffic_sign_type = msg.traffic_sign_type
        slot1 = msg.slot1
        slot2 = msg.slot2
        parking_type = msg.parking_type
        if traffic_sign_type == 3 and (slot1 == 1 or slot2 == 1):
            if parking_type == 0:
                if slot1 == 1 and slot2 == 0:
                    print("start parking parallel at slot 1")
                    self.parking_parallel(slot1, slot2)
                elif slot1 == 0 and slot2 == 1:
                    print("start parking parallet at slot 2")
                    self.parking_parallel(slot1, slot2)
            elif parking_type == 1:
                if slot1 == 1 and slot2 == 0:
                    print("start parking perpendicular at slot 1")
                    self.parking_perpendicular(slot1,slot2)
                elif slot1 == 0 and slot2 == 1:
                    print("start parking perpendicular at slot 2")
                    self.parking_perpendicular(slot1,slot2)
        else:
            print("pass")

    def __init__(self):
        rospy.init_node('parkingNODE', anonymous=False)
        self.parking_subscriber = rospy.Subscriber("/automobile/parking", Parking, self.traffic_sign_parking_processing)
        self.unlock = 1
        self.lock = 0
        self.control = controlNODE()

    def run(self):
        rospy.loginfo('starting parkingNODE')
        self._read()
    
    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("Hello from parking")
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
    def parking_perpendicular(self, slot1, slot2):
        if slot1 == 1 and slot2 == 0:
            self.control.setSteer(23)
            self.control.moveForward(0.5, 0.3)
        elif slot1 == 0 and slot2 == 1:
            self.control.setSteer(23)
            self.control.moveForward(-0.5, 0.3)

    def parking_parallel(self, slot1, slot2):
        if slot1 == 1 and slot2 == 0:
            control.setSteer(-23)
            control.moveForward(0.5, 0.3)
        elif slot1 == 0 and slot2 == 1:
            control.setSteer(-23)
            control.moveForward(-0.5, 0.3)

if __name__ == "__main__":
    parkNODE = parkingNODE()
    parkNODE.run()