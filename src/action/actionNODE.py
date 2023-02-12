
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

# =============================== CONFIG =================================================
testRUNING          =  True
testWAITING         =  True 

class State(Enum):
    NOT_RUNNING = 0
    RUNNING = 1

class RunStates(Enum):
    RUNNING = 0
    WAIT = 1
    HARD_TURN = 2
    SWITCH_LANE = 3
    ROUNDABOUT = 4
    HIGHWAY = 5
    CROSSWALK = 6
    RAMP = 7
    TAILING = 8
    PARKING = 9
    TRAFFIC_LIGHT = 10 
    PRIORITY = 11
    STOP = 12
    
class TrafficSign:
    #STOP, parking place, crosswalk, priority road, highway entrance, highway exit, roundabout, one-way road, and no-entry road. 
    STOP_SIGN = 0
    PARKING_SIGN = 1
    CROSS_WALK = 2
    PRIORITY_SIGN = 3
    HIGHWAY_ENTRANCE_SIGN = 4
    HIGHWAY_EXIT_SIGN = 5
    ROUNDABOUT_SIGN = 6
    ONE_WAY_SIGN = 7
    NO_ENTRY_SIGN = 8
    NO_SIGN = 9
    
class TrafficLightRule(Enum):
    GREEN_LIGHT = 0
    RED_LIGHT = 1
    YELLOW_LIGHT = 2
    
class LanePosition(Enum):
    LEFT_LANE = 0
    RIGHT_LANE = 1
    
class SpeedMod(Enum):
    NORMAL = 1
    HIGH = 1.4
    LOW = 0.6
    
class actionNODE:
    def __init__(self) -> None:
        rospy.init_node('actionNODE', anonymous=False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", String, self.lane_check)
        self.object_subscriber = rospy.Subscriber("/automobile/object", String, self.check_state2)
        self.traffic_sign_subscriber = rospy.Subscriber("/automobile/traffic_sign", String, self.traffic_sign_check)
        self.v2v_subscriber = rospy.Subscriber("/automobile/vehicles", vehicles, self.check_state3)
        self.traffic_light = rospy.Subscriber("/automobile/traffic_light", vehicles, self.traffic_light_check)
        self.imu = rospy.Subscriber("/automobile/IMU",IMU,self.imu_check)
        self.server = rospy.Subscriber("/automobile/server",Server,self.check_server)
        #MORE SUBSCRIBING IF AVAILABLE
        
        #INIT STATE
        # self.init = 1
        # self.start_signal = 0
        # self.state = State.NOT_RUNNING
        # self.run_state = RunStates.GO_STRAIGHT
        # self.traffic_light = Traffic_Light_Rule.GREEN_LIGHT
        # self.flags = None
        # self.control = controlNODE()
        self.init = 1
        self.start_signal = 0
        self.traffic_sign = NO_SIGN
        self.lane = LanePosition.RIGHT_LANE
        self.speed_mod = SpeedMod.NORMAL
        self.lane_switchable = False
        self.base_speed = 0.1
        self.steer_angle = 0
        self.lock = 0
        self.unlock = 1
        self.control = controlNODE()
        self.main_process()
        
    def lock_state(self, state):
        if not isinstance(state, RunStates):
            raise TypeError('state must be an instance of RunStates Enum')

        if self.unlock:
            self.lock = 1
            self.unlock = 0
            self.lock_start_time = rospy.get_time()
            self.run_state = state
        
    def unlock_state(self, time):
        if self.lock:
            if (rospy.get_time() - self.lock_start_time) >= time:
                self.unlock = 1
                self.lock = 0
                
    def lane_check(sel, msg):
        if self.lane == LanePosition.RIGHT_LANE: #on the right side of the road, check left lane type for lane switching
            if msg.left_lane_type == 1: #dotted lane
                self.lane_switchable = True
            else:
                self.lane_switchable = False
                
        else: #on the left side of the road, check right lane type for lane switching
            if msg.right_lane_type == 1: #dotted lane
                self.lane_switchable = True
            else:
                self.lane_switchable = False
                
        self.steer_angle = msg.steer_angle
    
    def traffic_sign_check(self, msg):
        if msg.traffic_sign == 6: #"STOP_SIGN":
            self.traffic_sign = TrafficSign.STOP_SIGN
            if run_state==RunStates.RUNNING:
                self.sign_start_time = rospy.get_time()
                self.run_state = RunStates.WAIT
                
        elif msg.traffic_sign == 3: #"PARKING_SIGN":
            self.traffic_sign = TrafficSign.PARKING_SIGN
            if run_state==RunStates.RUNNING:
                self.run_state = RunStates.WAIT
                
        elif msg.traffic_sign == 0: #"CROSS_WALK":
            self.traffic_sign = TrafficSign.CROSS_WALK
            if run_state==RunStates.RUNNING:
                self.run_state = RunStates.WAIT
                
        elif msg.traffic_sign == 0: #"PRIORITY_SIGN":
            self.traffic_sign = TrafficSign.PRIORITY_SIGN
            if run_state==RunStates.RUNNING:
                self.run_state = RunStates.WAIT
                
        elif msg.traffic_sign == "HIGHWAY_ENTRANCE_SIGN"
            self.traffic_sign = TrafficSign.HIGHWAY_ENTRANCE_SIGN
            if run_state==RunStates.RUNNING:
                self.speed_mod = SpeedMod.HIGH
                self.run_state = RunStates.HIGHWAY
                
        elif msg.traffic_sign == "HIGHWAY_EXIT_SIGN"
            self.traffic_sign = TrafficSign.HIGHWAY_EXIT_SIGN
            if run_state==RunStates.HIGHWAY:
                self.speed_mod = SpeedMod.NORMAL
                self.run_state = RunStates.RUNNING
                
        elif msg.traffic_sign == 7: #"ROUNDABOUT_SIGN"
            self.traffic_sign = TrafficSign.ROUNDABOUT_SIGN
            if run_state==RunStates.RUNNING:
                self.run_state = RunStates.WAIT
                
        elif msg.traffic_sign == 2: #"ONE_WAY_SIGN"
            self.traffic_sign = TrafficSign.ONE_WAY_SIGN
            if run_state==RunStates.RUNNING:
                self.run_state = RunStates.WAIT
                
        elif msg.traffic_sign == 1: #"NO_ENTRY_SIGN"
            self.traffic_sign = TrafficSign.NO_ENTRY_SIGN
            if run_state==RUNNING:
                self.run_state = RunStates.WAIT
                
    def traffic_light_check(self, msg):
        print("Hello")
        
    def imu_check(self, msg):
        RAMP_HEIGHT = 15
        RAMP_WIDTH = 100 # not know the exact value
        RAMP_ANGLE = math.atan(RAMP_HEIGHT/RAMP_WIDTH)
        
        if msg.pitch > 0 and msg.pitch <= RAMP_ANGLE:
            if self.run_state == RunStates.RUNNING:
                self.speed_mod = SpeedMod.HIGH
                self.run_state = RunStates.RAMP
            
        elif msg.pitch < 0 and msg.pitch >= -RAMP_ANGLE:
            if self.run_state == RunStates.RAMP:
                self.speed_mod = SpeedMod.LOW
                
        else:
            if self.run_state == RunStates.RAMP and self.speed_mod = SpeedMod.LOW:
                self.run_state = RunStates.RUNNING
            self.speed_mod = NORMAL
                
    def running_action(self):
        self.control.setSteer(self.steer_angle)
        OFFSET_ANGLE = 20
        MOD = 10
        offset_speed = abs((abs(self.steer_angle) - OFFSET_ANGLE)) // MOD
        if offset_speed > 0:
            self.control.setSpeed(self.base_speed - offset_speed)
        else:
            self.speed_action()
        
    def wait_action(self):
        if  (    
            (traffic_light and light_color == TrafficLightColor.GREEN_LIGHT) or 
            (stop_sign     and rospy.get_time() - self.sign_start_time >= 3) or
            (pedestrian    and pedestrian_detection == PedestrianPosition.LEFT)
        ):
        self.lock_state(RunStates.RUNNING)
            
    def speed_action(self): # called in state == RAMP and HIGHWAY
        self.control.setSpeed(self.base_speed * self.speed_mod)
            
    def auto_control(self):
        if testRUNING:
            if self.run_state == RunStates.RUNNING:
        if testWAITING:
            if self.run_state == RunStates.WAIT:
                self.wait_action()
        
    def run(self):
        while not rospy.is_shutdown() and start: 
            while not start_signal: #cho den xanh de xuat phat, bien start chi duoc dung mot lan
                print("Waiting for start signal")
            if(self.state == State.NOT_RUNNING):
                print("DOING NOTHING")
            elif(self.state == State.RUNNING):
                auto_control()
                
if __name__ == "__main__":
    action_node = actionNODE()
    action_node.run()
    #         while not rospy.is_shutdown() and start: 
    #             while not start: #cho den xanh de xuat phat, bien start chi duoc dung mot lan
    #                 print("Waiting for green")        
    #             if(self.state == State.NOT_RUNNING):
    #                 print("DOING NOTHING")
    #             elif(self.state == State.RUNNING):
    #                 auto_control()       
    
    # def check_trafficlight(self, msg):
    #     if GREEN_LIGHT:
    #         if self.init == 1:
    #             self.start = 1 #bien start chi duoc dung mot lan
    #             self.init = 0
    #             self.state = State.RUNNING
    # def check_state2(self, msg):
    #     pass
    
    # def imu_check(self,msg):
    #     # neu gap doc thi doi state thanh len doc
    #     # neu xuong doc thi doi state thanh xuong doc
        
    # def object_check(self,msg):
        
    # def traffic_check(self,msg):
    
    # def lane_check(self, msg): # neu xe qua lech do voi duong dang di => return goc steer
    #     # xu ly xe di trong nay
    #     # gui toc do + goc lai
    #     # tinh toan gi do de self.angle and self.speed
    #     # state machine se duoc xu ly o day
    #     # if state == len doc hoac xuong doc: xu ly toc do

    # def check_server(self, msg):
    #     # xu ly tin hieu overtake static car
    #     # xu ly tin hieu dung xe
    #     # xu ly tin hieu den xanh den do
        
    # def auto_control(self): # chi de check state machine
    #     if self.run_state == RunStates.GO_STRAIGHT:
    #         if gap nguoi di bo:
    #             chuyen trang thai => cho nguoi di bo
    #         if gap static car:
    #             chuyen sang trang thai overtake
    #         self.control.setsomething
    #     if self.run_state == RunStates.SWITCH_LANE:
    #         pass

    #     if self.run_state == RunStates.WAIT_FOR_PEDESTRIANS: #cho thay doi trang thai tu object_detection
    #         if(nguoi di bo di qua roi):
    #             self.run_state == RunStates.GO_STRAIGHT

    #     if self.run_state == RunStates.OVERTAKE: #cho thay doi trang thai tu server
    #         if(overtake xong roi()): # HARD => PLEASE INVESTIGATE MORE
    #             self.run_state == RunStates.GO_STRAIGHT

    #     if self.run_state == RunStates.PARKING: #cho thay doi trang thai tu server
    #         if(cho` parking xong roi):
    #             self.run_state == RunStates.OUT_OF_PARKING

    #     if self.run_state == RunStates.OUT_OF_PARKING:
    #         if(ra khoi parking):
    #             self.run_state == RunStates.GO_STRAIGHT

    #     if self.run_state == RunStates.FOLLOWING_TRAFFIC: #doi tin hieu tu traffic light
            
    #         if(self.traffic_light == Traffic_Light_Rule.GREEN_LIGHT):
    #             pass

    #         if(self.traffic_light == Traffic_Light_Rule.RED_LIGHT):
    #             pass

    #         if(self.traffic_light == Traffic_Light_Rule.YELLOW_LIGHT):

    #     if self.run_state == RunStates.ROUNDABOUT:
    #         dosomething until out of ROUNDABOUT
    #         self.run_state == RunStates.GO_STRAIGHT

    # def main_process(self):
    #         while not rospy.is_shutdown() and start: 
    #             while not start: #cho den xanh de xuat phat, bien start chi duoc dung mot lan
    #                 print("Waiting for green")        
    #             if(self.state == State.NOT_RUNNING):
    #                 print("DOING NOTHING")
    #             elif(self.state == State.RUNNING):
    #                 auto_control()
            