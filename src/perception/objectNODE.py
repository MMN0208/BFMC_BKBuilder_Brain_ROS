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

import numpy as np
import cv2
from object_detection.network.edgetpumodel import EdgeTPUModel
from object_detection.network.utils import plot_one_box, Colors, get_image_tensor

from utils.msg import pedestrian

class objectNODE():
    def __init__(self):
        
        """perceptionNODE is used to publish messages produced by
        the perception service including lane detection and object detection
        """
              
        rospy.init_node('objectNODE', anonymous=False)
        
        #======CAMERA======
        self.bridge = CvBridge()
        self.object_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._object)
        #======PUBLISHER======
        self.pedestrian_publisher = rospy.Publisher("/automobile/pedestrian", pedestrian, queue_size=1)
        #self.car_pubisher
        #self.traffic_light_publisher
        #======OBJECT DETECTION======
        self.model_path = "object_detection/weights/traffic.tflite"
        self.names = "object_detection/data.yaml"
        self.conf_thresh = 0.5
        self.iou_thresh = 0.65
        self.device = 0
        
        self.model = EdgeTPUModel(self.model_path, self.names, conf_thresh=self.conf_thresh, iou_thresh=self.iou_thresh)
        #self.model = None 
        
        self.colors = Colors()
        #.
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting objectNODE")
        rospy.spin()   
    
    # ===================================== OBJECT DETECT ========================================
    def _object(self, msg):
        """Object detection callback
        """
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        output_image = image
        full_image, net_image, pad = get_image_tensor(image, 640) #Transform the image into tensors
        pred = self.model.forward(net_image) #Pass the tensor to the model to get a prediction
        #print(f"DetectionProcess{net_image.shape}")
        det = self.model.process_predictions(pred[0], full_image, pad) #Post process prediction
                
                
        for *xyxy, conf, cls in reversed(det): #Process prediction loop
            '''
            xyxy (List): bounding box
            conf (int): prediction percentage
            cls (int): class index of prediction
            '''
            c = int(cls)  # integer class
            label = f'{self.model.names[c]} {conf:.2f}' #Set label to the class detected
            command = f'DETECT:{label}:{xyxy}'
            print(command)
            output_image = plot_one_box(xyxy, output_image, label=label, color=self.colors(c, True)) #Plot bounding box onto output_image
        
        #cv2.imshow("test", output_image)       
        #cv2.waitKey(1)    
        imageObject = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
        out = pedestrian()
        out.image = imageObject
        out.valid, out.is_pedestrian_infront, out.x, out.y, out.h, out.w = 0,0,0,0,0,0
        self.pedestrian_publisher.publish(out)  
        tinference, tnms = self.model.get_last_inference_time()
        print("Frame done in {}".format(tinference+tnms))
     
            
if __name__ == "__main__":
    perNod = objectNODE()
    perNod.run()
