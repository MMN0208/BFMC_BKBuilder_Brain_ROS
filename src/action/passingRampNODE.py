#!/usr/bin/env python3


import cv2
import sys
sys.path.append('.') 
import rospy
from utils.msg import IMU
import math
from enum import Enum
import time

#from control.controlNODE import *

class passingRampNODE():
    def __init__(self):
        rospy.init_node('passingRampNODE', anonymous = False)
        self.passingRamp_subcriber = rospy.Subscriber("/automobile/imu", IMU, self._handle)
    def run(self):
        rospy.loginfo('starting passingRampNODE')
        self._read()
    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("Hello from passing ramp")
                time.sleep(2)
            except UnicodeDecodeError:
                pass
    def _handle(self,msg):
        pitch = msg.pitch
        """
        This function receive a pitch - a value illustrate how car is tilted ( measure in radian)
        and control the car to accelerate/decelerate
        """
        RAMP_HEIGHT = 15
        RAMP_WIDTH = 100 # not know the exact value
        RAMP_ANGLE = math.atan(RAMP_HEIGHT/RAMP_WIDTH)
        """
        assume that if the go up the ramp, the value of pitch will be positive, vice versa
        """
        RAITO_OF_ANGLE = 0.5
        BASE_SPEED = 0.1 # call function get the speed
        ACCELERATE_RATE = 1.15
        DECELERATE_RATE = 0.85
        #control = controlNODE()
        print("pitch:{}, angle:{}".format(msg.pitch,RAMP_ANGLE))
        if (pitch > 0 and pitch <= RAITO_OF_ANGLE*RAMP_ANGLE): # going up the ramp
            #control.setSpeed(BASE_SPEED*ACCELERATE_RATE)
            print("accelerate")
        elif (pitch < 0 and pitch >= -1*RAITO_OF_ANGLE*RAMP_ANGLE): #going down the ramp
            #control.setSpeed(BASE_SPEED*DECELERATE_RATE)
            print("decelerate")
        time.sleep(1)

if __name__ == "__main__":
    passRampNode = passingRampNODE()
    passRampNode.run()



