#! /usr/bin/env python3

import sys
sys.path.append('.')
import os.path
import time
import math
import rospy
from utils.msg import IMU
import random

class imuNODE():
    def __init__(self):
        rospy.init_node('imuNODE', anonymous=False)
        self.BNO_publisher = rospy.Publisher("/automobile/imu", IMU, queue_size=1)

    def run(self):
        rospy.loginfo("starting imuNODE")
        self._send()
    def _send(self):
        imudata = IMU()
        imudata.roll = 1
        imudata.yaw = 2
        imudata.pitch = 1
        while not rospy.is_shutdown():
            imudata = IMU()
            imudata.roll = imudata.roll + 0.1
            imudata.yaw = imudata.yaw + 0.2
            imudata.pitch = imudata.pitch + random.gauss(0,2)
            imudata.accelx = 0
            imudata.accely = 0
            imudata.accelz = 0
            self.BNO_publisher.publish(imudata)

if __name__ == "__main__":
    imuNod = imuNODE()
    imuNod.run()