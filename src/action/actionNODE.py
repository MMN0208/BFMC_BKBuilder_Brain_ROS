#!/usr/bin/env python3
import socket
import struct
import time
import cv2

import sys
sys.path.append('../') 
        
import rospy

from cv_bridge       import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg    import String
from utils.msg       import vehicles, traffic_sign
from enum import Enum

from control.controlNODE import controlNODE


class SystemStates(Enum):
    OFFLINE = 0
    ONLINE = 1

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
    
class TrafficSign(Enum):
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
    RED_LIGHT = 0
    YELLOW_LIGHT = 1
    GREEN_LIGHT = 2
    
class LanePosition(Enum):
    LEFT_LANE = 0
    RIGHT_LANE = 1
    
class SpeedMod(Enum):
    NORMAL = 0
    HIGH = 1
    LOW = 2

# =============================== CONFIG =================================================
testRUNNING         =  True
testWAITING         =  True
testPARKING         =  True
testSPEED           =  True
testTRAFFICLIGHT    =  False
testCROSSWALK       =  True

# =============================== BUFFER =================================================
#imu buffer
imu_buffer0 = 0
imu_buffer1 = 0
imu_buffer2 = 0

# =============================== GLOBAL VARIABLES =======================================
#traffic_sign
traffic_sign_type = TrafficSign.NO_SIGN

#traffic_light
traffic_light = False
light_color = TrafficLightRule.GREEN_LIGHT

#pedestrian
wait_for_pedestrian = False
pedestrian = False
    
class actionNODE:
    def __init__(self) -> None:
        rospy.init_node('actionNODE', anonymous=False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", String, self.lane_check)
        self.pedestrian_subscriber = rospy.Subscriber("/automobile/pedestrian", String, self.pedestrian_check)
        self.traffic_sign_subscriber = rospy.Subscriber("/automobile/traffic_sign", traffic_sign, self.traffic_sign_check)
        # self.v2v_subscriber = rospy.Subscriber("/automobile/vehicles", vehicles, self.check_state3)
        self.traffic_light_subscriber = rospy.Subscriber("/automobile/traffic_light", vehicles, self.traffic_light_check)
        # self.imu_subscriber = rospy.Subscriber("/automobile/IMU",IMU,self.imu_check)
        # self.server_subscriber = rospy.Subscriber("/automobile/server",Server,self.check_server)
        #MORE SUBSCRIBING IF AVAILABLE
        
        #INIT STATE
        # self.init = 1
        # self.start_signal = 0
        # self.state = State.OFFLINE
        self.run_state = RunStates.RUNNING
        # self.traffic_light = TrafficLightRule.GREEN_LIGHT
        # self.flags = None
        # self.control = controlNODE()
        self.init = 1
        self.start_signal = 0
        self.lane = LanePosition.RIGHT_LANE
        self.speed_mod = SpeedMod.NORMAL
        self.lane_switchable = False
        self.sign_start_time = 0
        self.base_speed = 0.1
        self.steer_angle = 0
        self.lock = 0
        self.unlock = 1
        self.control = controlNODE()
        
    def lock_state(self, state):
        if not isinstance(state, RunStates):
            raise TypeError('state must be an instance of RunStates Enum')

        if self.unlock:
            self.lock = 1
            self.unlock = 0
            self.lock_start_time = time.time()
            self.run_state = state
        
    def unlock_state(self, time):
        if self.lock:
            if (time.time() - self.lock_start_time) >= time:
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
        
    def pedestrian_check(self, msg):
        pedestrian = msg.pedestrian
    
    def traffic_sign_check(self, msg):
        global traffic_sign_type
        traffic_sign_type = msg.traffic_sign_type
        
        if DEBUG:
            print("traffic_sign callback, sign: ", traffic_sign_type, "runstate: ", self.run_state)
        
        if traffic_sign_type == TrafficSign.STOP_SIGN.value:
            # traffic_sign_type = TrafficSign.STOP_SIGN
            
            if self.run_state == RunStates.RUNNING:
                self.sign_start_time = time.time()
                if DEBUG:
                    print("STOP SIGN")
                self.run_state = RunStates.WAIT
                
        elif traffic_sign_type == TrafficSign.PARKING_SIGN:
            # traffic_sign_type = TrafficSign.PARKING_SIGN
            if self.run_state == RunStates.RUNNING:
                self.run_state = RunStates.PARKING
                
        elif traffic_sign_type == TrafficSign.CROSS_WALK:
            # traffic_sign_type = TrafficSign.CROSS_WALK
            if self.run_state == RunStates.RUNNING:
                self.speed_mod = SpeedMod.LOW
                self.run_state = RunStates.CROSS_WALK
        
        elif traffic_sign_type == TrafficSign.PRIORITY_SIGN:
            # traffic_sign_type = TrafficSign.PRIORITY_SIGN
            if self.run_state == RunStates.RUNNING:
                self.run_state = RunStates.WAIT
                
        elif traffic_sign_type == TrafficSign.HIGHWAY_ENTRANCE_SIGN.value:
            # traffic_sign_type = TrafficSign.HIGHWAY_ENTRANCE_SIGN
            if self.run_state == RunStates.RUNNING:
                self.speed_mod = SpeedMod.HIGH
                self.run_state = RunStates.HIGHWAY
                
        elif traffic_sign_type == TrafficSign.HIGHWAY_EXIT_SIGN.value:
            # traffic_sign_type = TrafficSign.HIGHWAY_EXIT_SIGN
            if self.run_state==RunStates.HIGHWAY:
                self.speed_mod = SpeedMod.NORMAL
                self.run_state = RunStates.RUNNING
                
        elif traffic_sign_type == TrafficSign.ROUNDABOUT_SIGN:
            # traffic_sign_type = TrafficSign.ROUNDABOUT_SIGN
            if self.run_state == RunStates.RUNNING:
                self.run_state = RunStates.ROUNDABOUT
                
        elif traffic_sign_type == TrafficSign.ONE_WAY_SIGN:
            # traffic_sign_type = TrafficSign.ONE_WAY_SIGN
            if self.run_state == RunStates.RUNNING:
                print("One way road")
                
        elif traffic_sign_type == TrafficSign.NO_ENTRY_SIGN:
            # traffic_sign_type = TrafficSign.NO_ENTRY_SIGN
            if self.run_state == RunStates.RUNNING:
                print("Can not go this way")
        
        elif traffic_sign_type == TrafficSign.NO_SIGN:
            if self.run_state != RunStates.RUNNING:
                self.run_state = RunStates.RUNNING
                
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
            if self.run_state == RunStates.RAMP and self.speed_mod == SpeedMod.LOW:
                self.run_state = RunStates.RUNNING
            self.speed_mod = NORMAL
                
    def running_action(self):
        # self.control.setSteer(self.steer_angle)
        # OFFSET_ANGLE = 20
        # MOD = 10
        # offset_speed = abs((abs(self.steer_angle) - OFFSET_ANGLE)) // MOD
        # if offset_speed > 0:
        #     self.control.setSpeed(self.base_speed - offset_speed)
        # else:
        #     self.speed_action()
        self.speed_action()
        
    def wait_action(self):
        print(traffic_sign_type)
        if  (    
            #(traffic_light == True                       and light_color == TrafficLightColor.GREEN_LIGHT) or 
            (traffic_sign_type == TrafficSign.STOP_SIGN.value and (time.time() - self.sign_start_time) >= 3) #or
            #(wait_for_pedestrian == True           and pedestrian == False)
        ):
        # if (time.time() - self.sign_start_time) >= 3:
            #wait_for_pedestrian = False
            self.speed_mod = SpeedMod.NORMAL
            self.lock_state(RunStates.RUNNING)
        else:
            if DEBUG:
                print(time.time() - self.sign_start_time)
            self.control.brake(0)
            
    def speed_action(self): # called in state == RAMP and HIGHWAY
        if self.speed_mod == SpeedMod.NORMAL:
            speed_mod = 1
        elif self.speed_mod == SpeedMod.HIGH:
            speed_mod = 1.4
        elif self.speed_mod == SpeedMod.LOW:
            speed_mod = 0.6
            
        self.control.setSpeed(self.base_speed * speed_mod)
        
    def cross_walk_action(self):
        if pedestrian == False:
            self.speed_action()
        else:
            wait_for_pedestrian = True
            self.run_state = RunStates.WAIT
            
    def auto_control(self):
        if testRUNNING:
            if self.run_state == RunStates.RUNNING:
                self.running_action()
        if testWAITING:
            if self.run_state == RunStates.WAIT:
                if DEBUG: 
                    print("WAIT")
                self.wait_action()
        if testSPEED:
            if (    self.run_state == RunStates.RAMP    or
                    self.run_state == RunStates.HIGHWAY
                ):
                if DEBUG: 
                    print("SPEED")
                self.speed_action()
        if testCROSSWALK:
            if self.run_state == RunStates.CROSS_WALK:
                if DEBUG:
                    print("CROSS_WALK")
                self.cross_walk_action()
        
    def run(self):
        while not rospy.is_shutdown(): 
            # while not start_signal: #cho den xanh de xuat phat, bien start chi duoc dung mot lan
            #     print("Waiting for start signal")
            # if(self.state == State.OFFLINE):
            #     print("DOING NOTHING")
            # elif(self.state == State.ONLINE):
            self.auto_control()
            rospy.sleep(0.1)
                
if __name__ == "__main__":
    action_node = actionNODE()
    action_node.run()
            
