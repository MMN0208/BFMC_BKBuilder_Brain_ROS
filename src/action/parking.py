from control.controlNODE import *
import json
import time
import rospy
import sys

from std_msgs.msg      import String
from utils.srv        import subscribing, subscribingResponse

class parkingNODE():
    def __init__(self):
        rospy.init_node('parkingNODE', anonymous=False)
        self.parking_sublisher = rospy.Subscriber("/automobile/object", String, self._handle)

    def run(self):
        rospy.loginfo('starting parkingNODE')
        self._read()
    
    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("Hello from parking")
                time.sleep(2)
                parking_in_process = 1
                parking_type
                slot1, slot2
                if traffic_sign == parking and empty_spot == 1:
                    while parking_in_process:
                        parking_in_process = 0
                        if parking_type == 0:
                            self.parking_perpendicular(slot1, slot2)
                        elif parking_type == 1:
                            self.parking_parallel(slot1, slot2)

            except UnicodeDecodeError:
                pass

# To test:
    def parking_perpendicular(self, slot1, slot2):
        if slot1 == 1 and slot2 == 0:
            controlNODE.setSteer(23)
            controlNODE.moveForward(0.5, 0.3)
        elif slot1 == 0 and slot2 == 1:
            controlNODE.setSteer(23)
            controlNODE.moveForward(-0.5, 0.3)

    def parking_parallel(self, slot1, slot2):
        if slot1 == 1 and slot2 == 0:
            controlNODE.setSteer(23)
            controlNODE.moveForward(0.5, 0.3)
        elif slot1 == 0 and slot2 == 1:
            controlNODE.setSteer(23)
            controlNODE.moveForward(-0.5, 0.3)

    def _handle(self,msg):
        print(msg.yaw)


if __name__ == "__main__":
    parkNODE = parkingNODE()
    parkNODE.run()
    
        
