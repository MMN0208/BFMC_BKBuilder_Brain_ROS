import serial
import json
import time
import rospy 

from std_msgs.msg           import String, Byte, Int8
from utils.srv           import subscribing, subscribingResponse 

class parking_sub():
    def __init__(self):
        rospy.init_note('parking_sub', anonymous = False)
        self.parking_subscriber = rospy.Subscriber("automobile/object", String, self._write)
    
    def run(self):
        rospy.loginfo("starting serialNODE")
        self._read()

    def _read(self):
        while not rospy.is_shutdown():
            try:
                print("hello from parking")
                time.sleep(2)

            except UnicodeDecodeError:
                pass

    def _write(self,msg):
        command = msg.data
        print(command)

if __name__ == "__main__":
    park_subNODE = parking_sub()
    park_subNODE.run()