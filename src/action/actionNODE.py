#!/usr/bin/env python3
import socket
import struct
import time
import cv2
import math

import sys
sys.path.append('../') 
        
import rospy

from cv_bridge       import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg    import String, Byte
from utils.msg       import *
from enum            import Enum

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
    CROSS_WALK_SIGN = 0
    NO_ENTRY_SIGN = 1
    ONE_WAY_SIGN = 2
    PARKING_SIGN = 3
    PRIORITY_SIGN = 4
    ROUNDABOUT_SIGN = 5
    STOP_SIGN = 6
    HIGHWAY_ENTRANCE_SIGN = 8
    HIGHWAY_EXIT_SIGN = 7
    NO_SIGN = 9
    
class TrafficLightRule(Enum):
    RED_LIGHT = 0
    YELLOW_LIGHT = 1
    GREEN_LIGHT = 2
    
class LanePosition(Enum):
    LEFT_LANE = 0
    RIGHT_LANE = 1
    
class LaneVisibility(Enum):
    BOTH = 0
    LEFT = 1
    RIGHT = 2
    NONE = 3
    
class SpeedMod(Enum):
    NORMAL = 0
    HIGH = 1
    LOW = 2
    
# ============================ CAR CONSTRAINTS ===========================================
MAX_SPEED = 0.2
MAX_STEER = 30.0

# =============================== CONFIG =================================================
testRUNNING         =  True
testWAITING         =  True
testPARKING         =  True
testSPEED           =  True
testTRAFFICLIGHT    =  True
testTRAFFICSIGN     =  False
testCROSSWALK       =  True
DEBUG               =  True
DEBUG_ANGLE         =  True
DEBUG_MOD_SPEED     =  False

# =============================== BUFFER =================================================
#imu buffer
imu_buffer0 = 0
imu_buffer1 = 0
imu_buffer2 = 0

# =============================== GLOBAL VARIABLES =======================================
#traffic_sign
traffic_sign_type = TrafficSign.NO_SIGN

#traffic_light
NUM_OF_TRAFFIC_LIGHTS = 4
traffic_light_id = 0
light_color = [0 for i in range(NUM_OF_TRAFFIC_LIGHTS)]

#pedestrian
wait_for_pedestrian = False
pedestrian = False
    
class actionNODE:
    def __init__(self) -> None:
        rospy.init_node('actionNODE', anonymous=False)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", perception, self.lane_check)
        self.pedestrian_subscriber = rospy.Subscriber("/automobile/pedestrian_test", pedestrian_output, self.pedestrian_check)
        self.traffic_sign_subscriber = rospy.Subscriber("/automobile/traffic_sign", traffic_sign, self.traffic_sign_check)
        # self.v2v_subscriber = rospy.Subscriber("/automobile/vehicles", vehicles, self.check_state3)
        self.imu_subscriber = rospy.Subscriber("/automobile/IMU", IMU, self.imu_check)
        # self.server_subscriber = rospy.Subscriber("/automobile/server",Server,self.check_server)
        
        # TRAFFIC_LIGHT SUBSCRIBERS
        self.traffic_light_subscriber = rospy.Subscriber("/automobile/traffic_light", traffic_light, self.traffic_light_check)
        self.Semaphoremaster_subscriber = rospy.Subscriber("/automobile/semaphore/master", Byte, self.semaphore_master_update)
        self.Semaphoreslave_subscriber = rospy.Subscriber("/automobile/semaphore/slave", Byte, self.semaphore_slave_update)
        self.Semaphoreantimaster_subscriber = rospy.Subscriber("/automobile/semaphore/antimaster", Byte, self.semaphore_antimaster_update)
        self.Semaphorestart_subscriber = rospy.Subscriber("/automobile/semaphore/start", Byte, self.semaphore_start_update)
        #MORE SUBSCRIBING IF AVAILABLE
        
        # STEERING PID 
        self.previous_err = 0
        self.integral_err = 0
        self.offset_steer = 0.05
        
        #INIT STATE
        # self.sys_state = SystemStates.OFFLINE
        self.sys_state = SystemStates.ONLINE   #Test

        self.run_state = RunStates.RUNNING
        self.cur_x = 0
        self.cur_y = 0
        self.init = 1
        self.start_signal = 0
        self.lane = LanePosition.RIGHT_LANE
        self.speed_mod = SpeedMod.NORMAL
        self.lane_switchable = False
        self.sign_start_time = 0
        self.base_speed = 0.2
        
        # steering
        self.desired_steer_angle = 0
        self.curr_steer_angle = 0
        self.steer_end = 0
        self.see_left_lane = True
        self.see_right_lane = True

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
        
    def unlock_state(self, duration):
        if self.lock:
            if (time.time() - self.lock_start_time) >= duration:
                self.unlock = 1
                self.lock = 0
                
    # STEERING PID function
    def steering_pid(self, input):
        Ki = 0.09
        Kp = 0.4
        Kd = 0.04998
        
        processed = input
        setpoint = float(math.floor(input))
        error = math.fabs(setpoint - processed)
        
        integral_err = self.integral_err + error
        derivative_err = error - self.previous_err
        
        # calculate da shitz
        output = Kp * error + Ki * integral_err + Kd * derivative_err
        
        # save for next iteration
        self.previous_err = error
        self.integral_err = integral_err
        print("Desired angle = {}. Actual angle = {}".format(setpoint, processed))  
        return output
    
    # parking functions
    def parking_perpendicular(self, slot1, slot2):
        if slot1 == 1 and slot2 == 0:
            self.control.setSteer(-23)
            self.control.moveForward(0.5, 0.1)
            time.sleep(5)
            self.control.setSteer(20)
            self.control.moveForward(0.9, -0.1)
        elif slot1 == 0 and slot2 == 1:
            self.control.setSteer(23)
            self.control.moveForward(1.5, 0.1)

    def parking_parallel(self, slot1, slot2):
        if slot1 == 1 and slot2 == 0:
            self.control.setSteer(23)
            #self.control.moveForward(0.5, 0.3)
            self.control.setSpeed(-0.1)
            time.sleep(5)
            self.control.setSpeed(0)
            self.control.setSteer(-23)
            time.sleep(1)
            self.control.setSpeed(-0.1)
            time.sleep(5)
            self.control.setSpeed(0)
            self.control.setSteer(5)
            time.sleep(1)
            self.control.moveForward(0.4, 0.1)
        elif slot1 == 0 and slot2 == 1:
            self.control.setSteer(23)
            #self.control.moveForward(0.5, 0.3)
            self.control.setSpeed(-0.1)
            time.sleep(5)
            self.control.setSpeed(0)
            self.control.setSteer(-23)
            time.sleep(1)
            self.control.setSpeed(-0.1)
            time.sleep(4)
            self.control.setSpeed(0)
            self.control.setSteer(5)
            time.sleep(1)
            #self.control.moveForward(0.5, 0.1)
                
    def lane_check(self, msg):
        global MAX_STEER
        
        OFFSET_ANGLE = 3.0
        OFFSET_TURN = 8.0
        
        # TURN_ANGLE = MAX_STEER - 5.0

        if self.sys_state == SystemStates.ONLINE:
            # if self.lane == LanePosition.RIGHT_LANE: #on the right side of the road, check left lane type for lane switching
            #     if msg.left_lane_type == 1: #dotted lane
            #         self.lane_switchable = True
            #     else:
            #         self.lane_switchable = False
                    
            # else: #on the left side of the road, check right lane type for lane switching
            #     if msg.right_lane_type == 1: #dotted lane
            #         self.lane_switchable = True
            #     else:
            #         self.lane_switchable = False
                    
            if DEBUG_ANGLE:
                print("radius of curvature:", msg.radius_of_curvature)
                print("steer angle: ", msg.steer_angle)
                print("angle curvature:", msg.angle_curvature)
                print("lane_visibility: ", msg.one_lane)
                # 0: both
                # 1: left    
                # 2: right
                # 3: none
                       
            #processed_steer_angle = self.steering_pid(msg.steer_angle)
            if msg.one_lane == LaneVisibility.BOTH.value: # lane following
                if self.run_state == RunStates.RUNNING:
                    processed_steer_angle = msg.steer_angle
                    if abs(processed_steer_angle) > MAX_STEER:
                        if processed_steer_angle > 0:
                            self.desired_steer_angle = MAX_STEER
                        else:
                            self.desired_steer_angle = -MAX_STEER
                            
                    elif abs(processed_steer_angle - self.curr_steer_angle) > OFFSET_ANGLE:
                    # else:
                        self.desired_steer_angle = processed_steer_angle

                elif self.run_state == RunStates.HARD_TURN:
                    self.run_state = RunStates.RUNNING
            
            elif msg.one_lane == LaneVisibility.LEFT.value: # turn right
                if math.fabs(msg.steer_angle) > OFFSET_TURN and self.run_state == RunStates.RUNNING:
                    
                    # self.desired_steer_angle = TURN_ANGLE
                    
                    self.desired_steer_angle = self.curr_steer_angle # keep current steer angle
                    self.run_state = RunStates.HARD_TURN
            
            elif msg.one_lane == LaneVisibility.RIGHT.value: # turn left
                if math.fabs(msg.steer_angle) > OFFSET_TURN and self.run_state == RunStates.RUNNING:
                    
                    # self.curr_steer_angle = -TURN_ANGLE
                    
                    self.desired_steer_angle = self.curr_steer_angle # keep current steer angle
                    self.run_state = RunStates.HARD_TURN
                    
            elif msg.one_lane == LaneVisibility.NONE.value: # no lanes -> go straight forward
                if self.run_state == RunStates.RUNNING:
                    self.desired_steer_angle = 0
                elif self.run_state == RunStates.HARD_TURN:
                    self.run_state = RunStates.RUNNING
        
    def pedestrian_check(self, msg):
        if self.sys_state == SystemStates.ONLINE:
            global pedestrian
            pedestrian = msg.pedestrian
    
    def traffic_sign_check(self, msg):
        if self.sys_state == SystemStates.ONLINE:
            global traffic_sign_type
            traffic_sign_type = msg.traffic_sign_type
            self.unlock_state(3)
            
            if DEBUG:
                print("traffic_sign callback, sign: ", traffic_sign_type, ", run_state: ", self.run_state)
            
            if self.unlock:
                if traffic_sign_type == TrafficSign.STOP_SIGN.value:            
                    if self.run_state == RunStates.RUNNING:
                        self.sign_start_time = time.time()
                        if DEBUG:
                            print("STOP SIGN")
                        self.run_state = RunStates.WAIT
                        
                elif traffic_sign_type == TrafficSign.PARKING_SIGN.value:
                    if self.run_state == RunStates.RUNNING:
                        self.run_state = RunStates.PARKING
                        
                elif traffic_sign_type == TrafficSign.CROSS_WALK_SIGN.value:
                    # traffic_sign_type = TrafficSign.CROSS_WALK
                    if self.run_state == RunStates.RUNNING:
                        if DEBUG:
                            print("Running slow towards CROSSWALK")
                        self.speed_mod = SpeedMod.LOW
                        self.run_state = RunStates.CROSSWALK
                        self.sign_start_time = time.time()
                
                elif traffic_sign_type == TrafficSign.PRIORITY_SIGN.value:
                    # traffic_sign_type = TrafficSign.PRIORITY_SIGN
                    if self.run_state == RunStates.RUNNING:
                        self.run_state = RunStates.WAIT
                        
                elif traffic_sign_type == TrafficSign.HIGHWAY_ENTRANCE_SIGN.value:
                    if self.run_state == RunStates.RUNNING:
                        self.speed_mod = SpeedMod.HIGH
                        self.run_state = RunStates.HIGHWAY
                        
                elif traffic_sign_type == TrafficSign.HIGHWAY_EXIT_SIGN.value:
                    if self.run_state==RunStates.HIGHWAY:
                        self.speed_mod = SpeedMod.NORMAL
                        self.run_state = RunStates.RUNNING
                        
                elif traffic_sign_type == TrafficSign.ROUNDABOUT_SIGN.value:
                    # traffic_sign_type = TrafficSign.ROUNDABOUT_SIGN
                    if self.run_state == RunStates.RUNNING:
                        self.run_state = RunStates.ROUNDABOUT
                        
                elif traffic_sign_type == TrafficSign.ONE_WAY_SIGN.value:
                    # traffic_sign_type = TrafficSign.ONE_WAY_SIGN
                    if self.run_state == RunStates.RUNNING:
                        print("One way road")
                        
                elif traffic_sign_type == TrafficSign.NO_ENTRY_SIGN.value:
                    # traffic_sign_type = TrafficSign.NO_ENTRY_SIGN
                    if self.run_state == RunStates.RUNNING:
                        print("Can not go this way")
                
    def traffic_light_check(self, msg):
        global traffic_light_id
        global light_color
        traffic_light_id = msg.traffic_light_id
        
        if self.sys_state == SystemStates.ONLINE:
            self.unlock_state(3)
            
            if DEBUG:
                print("traffic_light callback, id: ", traffic_light_id)
            
            if self.unlock:
                if traffic_light_id:
                    if self.run_state == RunStates.RUNNING:
                        self.run_state = RunStates.TRAFFIC_LIGHT
                
    def semaphore_master_update(self, msg):
        global light_color
        light_color[0] = msg.data
        
        if DEBUG:
            print(light_color)
        
    def semaphore_slave_update(self, msg):
        global light_color
        light_color[1] = msg.data
        
    def semaphore_antimaster_update(self, msg):
        global light_color
        light_color[2] = msg.data
        
    def semaphore_start_update(self, msg):
        global light_color
        light_color[3] = msg.data
        
    def imu_check(self, msg):
        if self.sys_state == SystemStates.ONLINE:
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
        global MAX_SPEED
        global MAX_STEER
        
        EPSILON = 0.45
        # MOD = 10
        #offset_speed = abs(self.desired_steer_angle/100 - OFFSET_ANGLE)
        # if DEBUG_MOD_SPEED:
        #     print("offset speed: ",offset_speed)
        # if (self.base_speed - offset_speed) > 0:
        #     self.control.setSpeed(self.base_speed - offset_speed)
        # else:
        #     self.speed_action()
        
        if self.steer_end < time.time():
            self.steer_end = time.time() + 0.1
            self.curr_steer_angle = self.desired_steer_angle
        
        speed = MAX_SPEED - (math.fabs(self.curr_steer_angle) / MAX_STEER * EPSILON * MAX_SPEED)
        #self.control.setSteer(self.curr_steer_angle)
        self.control.setSteer(self.curr_steer_angle)
        print("angle: {}".format(self.curr_steer_angle))
        #self.control.setSpeed(speed)
        self.speed_action()
        print("speed: {}".format(speed))
        
    def turn_action(self):
        EPSILON = 0.3
        
        self.setSteer(self.curr_steer_angle)
        
        speed = MAX_SPEED - (math.fabs(self.curr_steer_angle) / MAX_STEER * EPSILON * MAX_SPEED)
        # self.setSpeed(speed)
        
        self.speed_action()

    def wait_action(self):
        global traffic_sign_type
        global traffic_light_id
        global wait_for_pedestrian
        
        if  (    
            (traffic_light_id                                 and light_color[traffic_light_id - 1] == TrafficLightRule.GREEN_LIGHT.value) or 
            (traffic_sign_type == TrafficSign.STOP_SIGN.value and (time.time() - self.sign_start_time) >= 3) or
            (wait_for_pedestrian == True                      and pedestrian == False)
        ):
            wait_for_pedestrian = False
            self.speed_mod = SpeedMod.NORMAL
            self.lock_state(RunStates.RUNNING)
        else:
            self.control.brake(0)
            
    def speed_action(self): # called in state == RAMP and HIGHWAY
        if self.speed_mod == SpeedMod.NORMAL:
            speed_mod = 1
        elif self.speed_mod == SpeedMod.HIGH:
            speed_mod = 1.4
        elif self.speed_mod == SpeedMod.LOW:
            speed_mod = 0.4
            
        self.control.setSpeed(self.base_speed * speed_mod)
        
    def cross_walk_action(self):
        global wait_for_pedestrian
        global pedestrian
        
        if pedestrian == False:
            if (time.time() - self.sign_start_time) >= 3:
                self.lock_state(RunStates.RUNNING)
            self.speed_action()
        else:
            wait_for_pedestrian = True
            self.run_state = RunStates.WAIT
            
    def traffic_light_action(self):
        global traffic_light_id
        global light_color
        
        if DEBUG:
            print(light_color[traffic_light_id - 1])
        
        if light_color[traffic_light_id - 1] == TrafficLightRule.RED_LIGHT.value or light_color[traffic_light_id - 1] == TrafficLightRule.YELLOW_LIGHT.value:
            self.run_state = RunStates.WAIT
        else:
            self.speed_mod = SpeedMod.NORMAL
            self.lock_state(RunStates.RUNNING)
            
    def move_to_destination(self, des_x, des_y):
        MAX_STEER = 30.00
        while abs(des_x - self.cur_x) >= 0.3 and abs(des_y - self.cur_y) >= 0.3:
            diff_x = des_x - self.cur_x
            diff_y = des_y - self.cur_y
            
            ang_rad = math.atan(diff_x / diff_y)
            ang_deg = ang_rad * (180 / math.pi)
            
            if abs(ang_deg) > MAX_STEER:
                if ang_deg > 0:
                    ang_deg = MAX_STEER
                else:
                    ang_deg = -MAX_STEER
            self.control.setSteer(ang_deg)
            self.control.setSpeed(0.15)
            rospy.sleep(0.5)
            self.control.brake(0)
            
    def auto_control(self):
        if self.sys_state == SystemStates.ONLINE:
            if testRUNNING:
                if self.run_state == RunStates.RUNNING:
                    if DEBUG: 
                        print("RUNNING")                
                    self.running_action()
                    
                elif self.run_state == RunStates.HARD_TURN:
                    if DEBUG:
                        print("HARD TURN")
                    self.turn_action()
            
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
                if self.run_state == RunStates.CROSSWALK:
                    if DEBUG:
                        print("CROSS_WALK")
                    self.cross_walk_action()
            if testTRAFFICLIGHT:
                if self.run_state == RunStates.TRAFFIC_LIGHT:
                    if DEBUG:
                        print("TRAFFIC_LIGHT")
                    self.traffic_light_action()
        
    def run(self):
        global traffic_light_id
        global light_color
        while not rospy.is_shutdown(): 
            # while not start_signal: #cho den xanh de xuat phat, bien start chi duoc dung mot lan
            if self.sys_state == SystemStates.OFFLINE:
                if DEBUG: 
                    print("WAITING FOR START SIGNAL")
                if traffic_light_id:
                    if light_color[traffic_light_id - 1] == TrafficLightRule.GREEN_LIGHT.value:
                        self.sys_state = SystemStates.ONLINE
                            
            elif self.sys_state == SystemStates.ONLINE:
                self.auto_control()
            
            rospy.sleep(0.1)
            
    def terminate(self):
        self.sys_state = SystemStates.OFFLINE
        self.control.brake(0)
            
def main():
    action_node = actionNODE()
    rospy.loginfo("START actionNODE")
    try:
        action_node.run()
    except:
        action_node.terminate()
        print("Your code is bugged lil bro")
    finally:
        action_node.terminate()
        rospy.loginfo("STOP actionNODE")
                
if __name__ == "__main__":
    main()