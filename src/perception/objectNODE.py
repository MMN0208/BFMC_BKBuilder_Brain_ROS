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


from object_detection.network.edgetpumodel import EdgeTPUModel
from object_detection.network.utils import plot_one_box, Colors, get_image_tensor, xyxy2xywh

from utils.msg import pedestrian

def crop_center(img,cropx,cropy):
    y,x = img.shape
    startx = x//2-(cropx//2)
    starty = y//2-(cropy//2)    
    return img[starty:starty+cropy,startx:startx+cropx]

class objectNODE():
    def __init__(self):
        
        """perceptionNODE is used to publish messages produced by
        the perception service including lane detection and object detection
        """
              
        rospy.init_node('objectNODE', anonymous=False)
        
        #======CAMERA======
        self.bridge = CvBridge()
        # self.object_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._object)

        self.object_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._object)
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self._depth)
        #======PUBLISHER======
        self.debug_publisher = rospy.Publisher("/automobile/object_debug", Image, queue_size=1)
        self.depth_debug_publisher = rospy.Publisher("/automobile/depth_debug", Image, queue_size=1)

        self.pedestrian_publisher = rospy.Publisher("/automobile/pedestrian", pedestrian, queue_size=1)
        self.traffic_light_publisher = rospy.Publisher("/automobile/traffic_sign", traffic_sign, queue_size=1)
        #self.car_pubisher
        #self.traffic_light_publisher
        #======OBJECT DETECTION======
        self.model_path = "object_detection/weights/traffic_3.tflite"
        self.names = "object_detection/data.yaml"
        self.conf_thresh = 0.5
        self.iou_thresh = 0.65
        self.device = 0
        
        self.model = EdgeTPUModel(self.model_path, self.names, conf_thresh=self.conf_thresh, iou_thresh=self.iou_thresh)
        #self.model = None 
        
        self.colors = Colors()
        self.colormap = plt.get_cmap('inferno')
        #.
        self.depth = np.random.rand(1280, 720)
    # ===================================== DEPTH ==========================================
    def _depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)
        print(f"DEPTH:{self.depth.shape}")
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
        #print("down here")
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        output_image = image
        #heatmap = self.depth
        #heatmap = cv2.applyColorMap(heatmap.astype(np.uint8), cv2.COLORMAP_HOT)

        #print(f"IMAGE:{image.shape}")
        full_image, net_image, pad = get_image_tensor(image, 640) #Transform the image into tensors
        pred = self.model.forward(net_image) #Pass the tensor to the model to get a prediction
        #print(f"DetectionProcess{net_image.shape}")
        #det = self.model.process_predictions(pred[0], output_image, pad) #Post process prediction
        det = pred[0]
        
        
        #Process predictions
        if len(det):
            # Rescale boxes from img_size to im0 size
            # x1, y1, x2, y2=
            det[:, :4] = self.model.get_scaled_coords(det[:,:4], output_image, pad)
            output = {}
            
            s = ""
            
            # Print results
            for c in np.unique(det[:, -1]):
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
            
            if s != "":
                s = s.strip()
                s = s[:-1]
            
            #logger.info("Detected: {}".format(s))
            
            # Write results
            for *xyxy, conf, cls in reversed(det):
                #c = int(cls)
                #print(f'{self.names[c]} {conf:.2f} {xyxy}')
                # Add bbox to image
                c = int(cls)  # integer class
                
                
                #d = self.depth[int(xyxy[0]):int(xyxy[2]), int(xyxy[1]):int(xyxy[3])]
                
                #print(f"Depth:{d.shape}")
                d_ = self.depth[int(xyxy[1]):int(xyxy[3]), int(xyxy[0]):int(xyxy[2])]
                d_ = crop_center(d_, 2, 2)
                if(np.median(d_) > 0 and np.median(d_) <= 600):
                    traffic = traffic_sign()
                    traffic.traffic_sign_type = c
                    self.traffic_light_publisher.publish(traffic)
                #print(f"size d_:{d_.shape}")
                print(f"Depth of {self.model.names[c]}:{d_}")
                print(f"size bbox={int(xyxy[0])}:{int(xyxy[1])}:{int(xyxy[2])}:{int(xyxy[3])}")
                
                label = f"Class:{self.model.names[c]}:{xyxy}"  
                output_image = plot_one_box(xyxy, output_image, label=label, color=self.colors(c, True))        
        #PROCESS
        #self.depth_debug_publisher.publish(self.bridge.cv2_to_imgmsg(heatmap, "rgb8"))
        
        imageObject = self.bridge.cv2_to_imgmsg(output_image, "rgb8")

        out = pedestrian()
        out.image = imageObject
        out.valid, out.is_pedestrian_infront, out.x, out.y, out.h, out.w = 0,0,0,0,0,0
        
        self.debug_publisher.publish(out.image)
        self.pedestrian_publisher.publish(out)  
        tinference, tnms = self.model.get_last_inference_time()
        #print("Frame done in {}".format(tinference+tnms))


if __name__ == "__main__":
    perNod = objectNODE()
    perNod.run()
