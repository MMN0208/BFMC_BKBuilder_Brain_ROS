#!/usr/bin/env python3

import cv2
import sys
sys.path.append('.') 

import rospy
from enum import Enum
import time
from utils.msg import trafficlight

class trafficLightNODE():
    def __init__(self):
        rospy.init_node('trafficLight', anonymous = False)
        self.trafficLight_subcriber = rospy.Subscriber("/automobile/trafficLight", trafficlight, self.traffic_light_processing)

    def run(self):
        rospy.loginfo('starting trafficLightNode')
        self._read()
        
    def _read(self):
        while not rospy.is_shutdown():
            try:
                #print("trafficLight hello")
                time.sleep(2)
            except UnicodeDecodeError:
                pass
    
    def traffic_light_processing(self, msg):
        RED = 0
        YEL = 1
        GRN = 2
        #This function receive a signal of traffic light and change state
        if msg.in_sight == 1:
            if msg.light_color == RED:
                print("Red light!")
            elif msg.light_color == YEL:
                print("Yellow light!")
            elif msg.light_color == GRN:
                print("Green light!")
            else:
                print("Not a valid color!")
        else:
            print("No traffic light in sight!")

if __name__ == "__main__":
    traLightNODE = trafficLightNODE()
    traLightNODE.run()