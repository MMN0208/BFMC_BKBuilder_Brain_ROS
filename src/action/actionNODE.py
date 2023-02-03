
import socket
import struct
import time
import cv2
        
import rospy

from cv_bridge       import CvBridge
from sensor_msgs.msg import Image, String
from utils.msg import vehicles
from enum import Enum

from control import controlNODE

class State(Enum):
    NOT_RUNNING = 0
    RUNNING = 1
    #
    RED_LIGHT = 2
    YELLOW_LIGHT = 3
    GREEN_LIGHT = 4
    #
    STOP_SIGN = 5
    PARKING = 6
    CROSSWALK = 7
    #
    RAMP = 8

class RunStates(Enum):
    GO_STRAIGHT = 0
    SWITCH_LANE = 1
    HARD_TURN = 2

class actionNODE:
    def __init__(self) -> None:
        rospy.init_node('actionNODE', anonymous=False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", String, self.lane_check)
        self.object_subscriber = rospy.Subscriber("/automobile/object", String, self.check_state2)
        self.v2v_subscriber = rospy.Subscriber("/automobile/vehicles", vehicles, self.check_state3)
        self.traffic_light = rospy.Subscriber("/automobile/traffic_light", vehicles, self.check_state4)
        
        #MORE SUBSCRIBING IF AVAILABLE
        
        #INIT STATE
        self.start = 0
        self.state = State.NOT_RUNNING
        self.run_state = RunStates.GO_STRAIGHT
        self.flags = None
        self.control = controlNODE()
        
        while start:
            self.main_process()
        
    def check_trafficlight(self, msg):
        if GREEN_LIGHT:
            start = 1
            self.state = State.RUNNING
    def check_state2(self, msg):
        pass
    
    def lane_check(self, msg): # neu xe qua lech do voi duong dang di => return goc steer
        # xu ly xe di trong nay
        # gui toc do + goc lai
        # tinh toan gi do de self.angle and self.speed

        
    def auto_control(self): # chi de check state machine
        if self.run_state == RunStates.GO_STRAIGHT:
            self.control.setsomething

    
    def main_process(self):
            while not rospy.is_shutdown():
                while not start:
                    start = 1 #start da chay roi khong chay dc nua             
                if(self.state == State.NOT_RUNNING):
                    print("DOING NOTHING")
                elif(self.state == State.RUNNING):
                    auto_control()
                if(self.state == State.RED_LIGHT):
                    print("RED LIGHT")
                if(self.state == State.YELLOW_LIGHT):
                    pass
                if(self.state == State.RAMP):
                    pass
            