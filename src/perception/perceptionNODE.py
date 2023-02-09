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

from std_msgs.msg      import String
from sensor_msgs.msg import Image
from utils.srv        import subscribing, subscribingResponse
from cv_bridge       import CvBridge
from utils.msg      import lane
#import object detection
import numpy as np
import cv2
from src.perception.object_detection.network.edgetpumodel import EdgeTPUModel
from src.perception.object_detection.network.utils import plot_one_box, Colors, get_image_tensor
from src.perception.lane_detection.core.utils import TESTNODES
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
        # self.object_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._object)
        self.lane_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._lane)

        #======OBJECT DETECTION======
        self.model_path = "src/perception/object_detection/weights/traffic.tflite"
        self.names = "src/perception/object_detection/data.yaml"
        self.conf_thresh = 0.5
        self.iou_thresh = 0.65
        self.device = 0
        
        self.model = EdgeTPUModel(self.model_path, self.names, conf_thresh=self.conf_thresh, iou_thresh=self.iou_thresh)
        
        #self.colors = Colors()
        
        #=======LANE DETECTION======="""s
        self.lane_publisher = rospy.Publisher("/automobile/lane", String, queue_size =1)
        self.bev_publisher = rospy.Publisher("/automobile/bev", Image, queue_size = 1)
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting perceptionNODE")
        #self._read() 
        rospy.spin()   
        #self._testNODE()        
    # ===================================== OBJECT DETECT ========================================
    # def _object(self, msg):
    #     """Object detection callback
    #     """
    #     image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     output_image = image
    #     full_image, net_image, pad = get_image_tensor(image, 640) #Transform the image into tensors
    #     pred = self.model.forward(net_image) #Pass the tensor to the model to get a prediction
    #     #print(f"DetectionProcess{net_image.shape}")
    #     det = self.model.process_predictions(pred[0], full_image, pad) #Post process prediction
                
                
    #     for *xyxy, conf, cls in reversed(det): #Process prediction loop
    #         '''
    #         xyxy (List): bounding box
    #         conf (int): prediction percentage
    #         cls (int): class index of prediction
    #         '''
    #         c = int(cls)  # integer class
    #         label = f'{self.names[c]} {conf:.2f}' #Set label to the class detected
    #         command = f'DETECT:{label}:{xyxy}'
    #         self.command_publisher.publish(command)
    #         #output_image = plot_one_box(xyxy, output_image, label=label, color=self.colors(c, True)) #Plot bounding box onto output_image
                    
    #     tinference, tnms = self.model.get_last_inference_time()
    #     print("Frame done in {}".format(tinference+tnms))
     
    # ===================================== LANE DETECT ========================================
    
    def _lane(self, msg):
        """Lane detection callback
        Send multiple messages with multiple types, please visit head of source file for 
        more details.

        Note: at this time just use pseudo messages defined in TESTNODES to test the publish() command
        """
        msg = lane()
        msg.steer_angle =   TESTNODES['STEER_ANGLE']
        msg.radius_of_curvature = TESTNODES['RADIUS']
        msg.off_centre = TESTNODES['OFF_CENTRE']
        msg.left_lane_type = TESTNODES['LLT']
        msg.left_lane_type = TESTNODES['RLT']
        msg.midpoint = TESTNODES['MIDPOINT']
        
        while not rospy.is_shutdown():
            self.lane_publisher.publish(msg)
        


    def send_BEV(self, msg):
        """Birdeye view callback
        Send only one message type Image (please visit ROs documentation for more information about custom message type)
        
        Note: at this time just use pseudo messages defined in TESTNODES to test the publish() command
        """
        image = TESTNODES['BEV']
        print("Image to be send: {}".format(image))
        send_image = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        while not rospy.is_shutdown():
            self.bev_publisher.publish(send_image)

    # ===================================== READ ==========================================
    def _read(self):
        """ It's represent the reading activity on the the serial.
        """
        while not rospy.is_shutdown():
            try:
                #print("hello from perception")
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
    def _testNODE(self):
        msg = lane()
        msg.steer_angle = 30.2
        msg.radius_of_curvature = 12.2
        msg.off_centre = 23.2
        msg.left_lane_type = 0
        msg.left_lane_type = 1
        msg.midpoint = 10
        while not rospy.is_shutdown():
            self.lane_publisher.publish(msg)
if __name__ == "__main__":
    perNod = perceptionNODE()
    perNod.run()