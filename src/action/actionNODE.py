
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

class RunStates(Enum):
    GO_STRAIGHT = 0
    SWITCH_LANE = 1
    HARD_TURN = 2
    WAIT_FOR_PEDESTRIANS = 3
    OVERTAKE = 4
    PARKING = 5
    OUT_OF_PARKING = 6 
    FOLLOWING_TRAFFIC = 7 
    ROUNDABOUND = 8
    # #
    # #
    # STOP_SIGN = 5
    # PARKING = 6
    # CROSSWALK = 7
    # #
    # RAMP = 8  
    # unknown state
class Traffic_Light_Rule(Enum):
    GREEN_LIGHT = 0
    RED_LIGHT = 1
    YELLO_LIGHT = 2
class actionNODE:
    def __init__(self) -> None:
        rospy.init_node('actionNODE', anonymous=False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", String, self.lane_check)
        self.object_subscriber = rospy.Subscriber("/automobile/object", String, self.check_state2)
        self.v2v_subscriber = rospy.Subscriber("/automobile/vehicles", vehicles, self.check_state3)
        self.traffic_light = rospy.Subscriber("/automobile/traffic_light", vehicles, self.traffic_check)
        self.imu = rospy.Subscriber("/automobile/IMU",IMU,self.imu_check)
        self.server = rospy.Subscriber("/automobile/server",Server,self.check_server)
        #MORE SUBSCRIBING IF AVAILABLE
        
        #INIT STATE
        self.init = 1
        self.start = 0
        self.state = State.NOT_RUNNING
        self.run_state = RunStates.GO_STRAIGHT
        self.traffic_light = Traffic_Light_Rule.GREEN_LIGHT
        self.flags = None
        self.control = controlNODE()
        
    def check_trafficlight(self, msg):
        if GREEN_LIGHT:
            if self.init == 1:
                self.start = 1 #bien start chi duoc dung mot lan
                self.init = 0
                self.state = State.RUNNING
    def check_state2(self, msg):
        pass
    
    def imu_check(self,msg):
        # neu gap doc thi doi state thanh len doc
        # neu xuong doc thi doi state thanh xuong doc
        
    def object_check(self,msg):
        
    def traffic_check(self,msg):
    
    def lane_check(self, msg): # neu xe qua lech do voi duong dang di => return goc steer
        # xu ly xe di trong nay
        # gui toc do + goc lai
        # tinh toan gi do de self.angle and self.speed
        # state machine se duoc xu ly o day
        # if state == len doc hoac xuong doc: xu ly toc do

    def check_server(self, msg):
        # xu ly tin hieu overtake static car
        # xu ly tin hieu dung xe
        # xu ly tin hieu den xanh den do
        
    def auto_control(self): # chi de check state machine
        if self.run_state == RunStates.GO_STRAIGHT:
            if gap nguoi di bo:
                chuyen trang thai => cho nguoi di bo
            if gap static car:
                chuyen sang trang thai overtake
            self.control.setsomething
        if self.run_state == RunStates.SWITCH_LANE:
            pass

        if self.run_state == RunStates.WAIT_FOR_PEDESTRIANS: #cho thay doi trang thai tu object_detection
            if(nguoi di bo di qua roi):
                self.run_state == RunStates.GO_STRAIGHT

        if self.run_state == RunStates.OVERTAKE: #cho thay doi trang thai tu server
            if(overtake xong roi()): # HARD => PLEASE INVESTIGATE MORE
                self.run_state == RunStates.GO_STRAIGHT

        if self.run_state == RunStates.PARKING: #cho thay doi trang thai tu server
            if(cho` parking xong roi):
                self.run_state == RunStates.OUT_OF_PARKING

        if self.run_state == RunStates.OUT_OF_PARKING:
            if(ra khoi parking):
                self.run_state == RunStates.GO_STRAIGHT

        if self.run_state == RunStates.FOLLOWING_TRAFFIC: #doi tin hieu tu traffic light
            
            if(self.traffic_light == Traffic_Light_Rule.GREEN_LIGHT):
                pass

            if(self.traffic_light == Traffic_Light_Rule.RED_LIGHT):
                pass

            if(self.traffic_light == Traffic_Light_Rule.YELLOW_LIGHT):

        if self.run_state == RunStates.ROUNDABOUND:
            dosomething until out of ROUNDABOUND
            self.run_state == RunStates.GO_STRAIGHT

    def main_process(self):
            while not rospy.is_shutdown() and start: 
                while not start: #cho den xanh de xuat phat, bien start chi duoc dung mot lan
                    print("Waiting for green")        
                if(self.state == State.NOT_RUNNING):
                    print("DOING NOTHING")
                elif(self.state == State.RUNNING):
                    auto_control()
            