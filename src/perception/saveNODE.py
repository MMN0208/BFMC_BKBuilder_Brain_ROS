#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

#import serial
import json
import time
import rospy
import matplotlib.pyplot as plt
import numpy as np
import cv2

from std_msgs.msg      import String
from sensor_msgs.msg import Image
from utils.msg import traffic_sign
from utils.srv        import subscribing, subscribingResponse
from cv_bridge       import CvBridge
import os

path = '/home/pi/new_test/'
os.chdir(path)

class saveNODE():
    def __init__(self):
        rospy.init_node('saveNODE', anonymous=False)
        self.bridge = CvBridge()

        self.object_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._object)
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self._depth)

        self.i = 0
        self.j = 0

    def run(self): 
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting saveNODE")
        rospy.spin()  
    
    def _depth(self, msg):
        print("come here")
        image = self.bridge.imgmsg_to_cv2(msg)
        filename = f"depth_cn:{self.i}.png"
        print(f"SAVING:" + filename)
        self.i = self.i+1
        cv2.imwrite(filename, image)
        
    
    def _object(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        filename = f"image_cn:{self.j}.png"
        print(f"SAVING:" + filename)
        self.j = self.j+1
        cv2.imwrite(filename, image)

if __name__ == "__main__":
    perNod = saveNODE()
    perNod.run()
