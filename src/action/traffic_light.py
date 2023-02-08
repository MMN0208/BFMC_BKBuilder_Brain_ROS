import cv2
import sys
sys.path.append('.') 
import rospy
from enum import Enum
import time
from std_msgs.msg import String

from controlNODE import *

class trafficLightNODE():
    def __init__(self):
        rospy.init_node('trafficLight', anonymous = False)
        self.trafficLight_subcriber = rospy.Subscriber("/automobile/trafficLight", int, traffic_light_processing)
        self.unlock=1
        self.lock=0
    def run(self):
        rospy.loginfo('starting trafficLightNode')
        self._read()
    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("trafficLight hello")
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
    
    def traffic_light_processing(self, msg):
         #This function receive a signal of traffic light and change state
        traffic_light=msg.traffic_light
        light_color=msg.light_color
        stop_line=msg.stop_line
        if traffic_light==1 and self.run_state==RUNNING: #in state running and meet the traffic light
            self.unlock_state(RunStates.RUNNING,1)
            if self.unlock:#avoid backing to trafffic light state right after runnung state
                self.run_state== TRAFFIC_LIGHT

          
       
    def traffic_light_state(self):
        # this function control the car when meet the traffic light
        #light color green=0, red=1, yellow=2
    if traffic_light==1:
	    
	    if light_color==1 or (light_color==2 and stop_line==1):
		   self.run_state = WAIT
        else 
           self.run_state = RUNNING
		   self.lock_state()


    if run_state==WAIT:
	    if traffic_light==1:
		    if light_color==0:
			   self.run_state = RUNNING
			   self.lock_state()


if __name__ == "__main__":
    trafficLightNODE = trafficLightNODE()
    trafficLightNODE.run()