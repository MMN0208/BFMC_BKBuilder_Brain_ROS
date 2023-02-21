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
import math
import json
import time
import rospy
from std_msgs.msg      import String
from sensor_msgs.msg import Image
from utils.srv        import subscribing, subscribingResponse
from cv_bridge       import CvBridge
from utils.msg      import perception, lane
#import object detection
import numpy as np
import cv2
from lane_detection.core.utils import TESTNODES, IMG_SIZE
from lane_detection.core.camera import Camera
#import lane detection
class perceptionNODE():
    def __init__(self):
        
        """perceptionNODE is used to publish messages produced by
        the perception service including lane detection and object detection

        Please checkout the ROS documentation for more information about the customized
        messages because there are some topics's messages are not standardized.
        
        To define nodes and their behaviors, we need to:
            1. Define publishers in src/perception/perceptionNODE.py
            2. Define msgs in src/utils/msg/lane.msg
            3. Define launch information in src/utils/launch

        * Definitions for several topics:

            automobile/birdeyes_view: just send only one message type Image
                msg_type: Image
                msg_var_name: BEV_threshold : this is variable name used in the function self.send_BEV(msg)
                publisher_name: BEV_publisher
                callback_function_name: send_BEV()
                example: self.BEV_publisher = rospy...

            automobile/lane: send multiples messages with multiple types
                msg_type: float32,  int8
                msg_var_names:  these variables name below are used in the function self._lane() 
                    float32: steer_angle
                    float32: radius_of_curvature
                    float32: off_centre
                    int8: left_lane_type
                    int8: right_lane_type
                    int8: midpoint
                publisher_name: lane_publisher
                callback_function_name: _lane()
                example: self.lane_publisher = rospy...
            
        *   IMPORTANT NOTE: 
            We need to test two publishers that publish two topics "birdeyes_view" and "lane" by 
            debugging when we try to send pseudo messages. Pseudo messages were defined in TESTNODES
            that was included in the source file (please visit src/perception/lane_detection/core/utils.py for more details of TESTNODES).

        """
              
        rospy.init_node('perceptionNODE', anonymous=False)
        
        #======CAMERA======
        self.bridge = CvBridge()
        #self.object_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._object)
        self.lane_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._lane)
        #=======LANE DETECTION======="""s
        self.camera = Camera()
        self.camera.load_calibrate_info('/home/pi/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save/calibration.pkl')
        self._image = None
        self.lane_publisher = rospy.Publisher("/automobile/lane", perception, queue_size =1)
        self.bev_publisher = rospy.Publisher("/automobile/bev", Image, queue_size = 1)
        self.lane_info_publisher = rospy.Publisher("/automobile/lane_info",lane, queue_size = 1)
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        
        rospy.loginfo("starting perceptionNODE")
        #rospy.spin() 
        while not rospy.is_shutdown():
            try:
                if self._image is not None:
                    #self.send_BEV()
                    self.send_perceptionInfo(self._image)
                    self.send_laneInfo(self._image)
                    print('hello world')
            except Exception as e:
                print(e)
    # ===================================== LANE DETECT ========================================
    
    def _lane(self, msg):
        """Lane detection callback
        """
        self._image = self.bridge.imgmsg_to_cv2(msg)

    def send_perceptionInfo(self, scene):
        
        """
        Send message via topic "automobile/lane"

        Message format:
            float32 steer_angle
            float32 radius_of_curvature
            float32 off_centre
            int8 left_lane_type
            int8 right_lane_type
            int32 midpoint

            left_lane_type, right_lane_type, radius, steer_angle
        """
        calibrate_scence = self.camera.undistort(scene)
        lane_detection_result = self.camera._runDetectLane(calibrate_scence)
        msg = perception()
        if lane_detection_result is not None:
            msg.steer_angle             = lane_detection_result['steer_angle']
            msg.radius_of_curvature     = lane_detection_result['radius'] 
            msg.left_lane_type          =   lane_detection_result['left_lane_type'] 
            msg.left_lane_type          =    lane_detection_result['right_lane_type']
        else:
            
            msg.steer_angle             = 0
            msg.radius_of_curvature     = -1
            msg.left_lane_type          = 0
            msg.left_lane_type          = 0
        self.lane_publisher.publish(msg)

    def send_laneInfo(self, scene):
        """
        Send message via topic "automobile/lane_info"
        
        Message format
            float32: width_topleft
            float32: height_topleft
            float32: width_topright
            float32: heigth_topright
            float32: width_bottomleft
            float32: height_bottomleft
            float32: width_bottomright
            float32: heigth_bottomright

        """
        msg = lane()
        msg.width_topleft =TESTNODES['WTL'] 
        msg.height_topleft = TESTNODES['HTL'] 
        msg.width_topright= TESTNODES['WTR'] 
        msg.height_topright=TESTNODES['HTR'] 
        msg.width_bottomright=TESTNODES['WBL'] 
        msg.height_bottomleft=TESTNODES['HBL'] 
        msg.width_bottomright=TESTNODES['WBR'] 
        msg.height_bottomright=TESTNODES['HBR']
        self.lane_info_publisher.publish(msg)

    def send_BEV(self):
        """Birdeye view callback
        Send only one message type Image (please visit ROs documentation for more information about custom message type)
        
        Note: at this time just use pseudo messages defined in TESTNODES to test the publish() command
        """
        image = TESTNODES['BEV']
        #print("Image to be send: {}".format(type(image)))
        send_image = self.bridge.cv2_to_imgmsg(image, '64FC3')
        self.bev_publisher.publish(send_image)

    # ===================================== READ ==========================================
    def _read(self):
        """ It's represent the reading activity on the the serial.
        """
        while not rospy.is_shutdown():
            try:
                print("hello from perception")
                time.sleep(2)
                 
            except UnicodeDecodeError:
                pass     
    # ===================================== WRITE ==========================================
    def _write(self, msg):
        """ Represents the writing activity on the the serial.
        """
        command = json.loads(msg.data)
        #command = msg.data
        print(command)

    
if __name__ == "__main__":
    perNod = perceptionNODE()
    perNod.run()