#!/usr/bin/env python3

#import serial
import json
import time
import rospy

from std_msgs.msg      import String
from utils.srv        import subscribing, subscribingResponse

class controlNODE():
    def __init__(self):
        
       #rospy.init_node('controlNODE', anonymous=False)
        
        self.command_publisher = rospy.Publisher("/automobile/command", String, queue_size=1)
        
        time.sleep(1)
        self.activatePID(True)
            
    def activatePID(self, activate=True):
        _activate = {
                "action": "4",
                "activate": activate
            }
        self.command_publisher.publish(json.dumps(_activate))
        
    def setSpeed(self, speed):
        _speed = {
                "action": "1",
                "speed": float(speed)
            }
        self.command_publisher.publish(json.dumps(_speed))
        
    def moveForward(self, distance, speed):
        _move = {
                "action": "7",
                "distance": float(distance),
                "speed": float(speed)
            }
        self.command_publisher.publish(json.dumps(_move))
        
    def setSteer(self, steerAngle):
        _steer = {
                "action": "2",
                "steerAngle": float(steerAngle)
            }
        self.command_publisher.publish(json.dumps(_steer))
        
    def brake(self, steerAngle):
        _brake = {
                "action": "3",
                "brake (steerAngle)": float(steerAngle)
            }
        self.command_publisher.publish(json.dumps(_brake))
        
     # ===================================== RUN ==========================================
    def run(self):
        rospy.loginfo("starting controlNODE")
        while not rospy.is_shutdown():
            self.activatePID()
            time.sleep(1)
            self.moveForward(1, 0.1)
            time.sleep(1)
            self.brake(0)
            time.sleep(2)
            self.setSpeed(0.2)
            time.sleep(1)
            self.setSteer(15)
            time.sleep(1)
        
if __name__ == "__main__":
    ctrlNod = controlNODE()
    ctrlNod.run()
