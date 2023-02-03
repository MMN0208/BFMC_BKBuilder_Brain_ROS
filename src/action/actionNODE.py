
import socket
import struct
import time
import cv2
        
import rospy

from cv_bridge       import CvBridge
from sensor_msgs.msg import Image, String
from utils.msg import vehicles
from enum import Enum

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
     

class actionNODE:
    def __init__(self) -> None:
        rospy.init_node('actionNODE', anonymous=False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", String, self.check_state)
        self.object_subscriber = rospy.Subscriber("/automobile/object", String, self.check_state)
        self.v2v_subscriber = rospy.Subscriber("/automobile/vehicles", vehicles, self.check_state)
        
        #MORE SUBSCRIBING IF AVAILABLE
        
        #INIT STATE
        self.state = State.NOT_RUNNING
        self.flags = None
        self.main_process()
        
    def check_state(self, msg):
        #TODOS: Define msg format
        if(msg._type == "vehicles"):
            #TODOS: Add steering angle calculation
            #TODOS: Change state logic
            pass
        elif(msg == "OBJECT"):
            pass
    
    def main_process(self):
        while not rospy.is_shutdown():
            if(self.state == State.NOT_RUNNING):
                print("DOING NOTHING")
            elif(self.state == State.RED_LIGHT):
                print("RED LIGHT")
            elif(self.state == State.YELLOW_LIGHT):
                pass
            elif(self.state == State.RAMP):
                pass
            