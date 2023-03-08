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

import time
from object_detection.network.edgetpumodel import EdgeTPUModel
from object_detection.network.utils import plot_one_box, Colors, get_image_tensor, xyxy2xywh

from utils.msg import pedestrian
import threading

object_lock = threading.Lock()
depth_lock = threading.Lock()

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

        self.object_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._objectcallback)
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self._depth)
        #======PUBLISHER======
        #self.debug_publisher = rospy.Publisher("/automobile/object_debug", Image, queue_size=1)
        #self.depth_debug_publisher = rospy.Publisher("/automobile/depth_debug", Image, queue_size=1)

        #self.pedestrian_publisher = rospy.Publisher("/automobile/pedestrian", pedestrian, queue_size=1)
        self.traffic_light_publisher = rospy.Publisher("/automobile/traffic_sign", traffic_sign, queue_size=1)
        #self.car_pubisher
        #self.traffic_light_publisher
        #======OBJECT DETECTION======
        self.model_path = "object_detection/weights/traffic_3.tflite"
        self.names = "object_detection/data.yaml"
        self.conf_thresh = 0.5
        self.iou_thresh = 0.60
        self.device = 0
        
        self.model = EdgeTPUModel(self.model_path, self.names, conf_thresh=self.conf_thresh, iou_thresh=self.iou_thresh)
        #self.model = None 
        print(self.model)
        #Thread init
        #self.detect_thread = threading.Thread(target=self._object, args=(arr,))

        #
        self.colors = Colors()
        self.colormap = plt.get_cmap('inferno')
        #.
        self.depth = np.random.rand(1280, 720)
        self.image = np.random.rand(1280, 720, 3)

        self.img_flag = 0
        self.depth_flag = 0
        #QUEUE
        self.queue = []
        self.d_queue = []
         
    # ===================================== DEPTH ==========================================
    def _depth(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg)
        #self.depth = cv2.applyColorMap(self.depth, cv2.COLORMAP_JET)
        depth_lock.acquire()
        # if(len(self.d_queue) < 100):
        #     self.d_queue.append(depth)
        self.depth = depth
        depth_lock.release()
        self.depth_flag = 1
        #print(f"DEPTH:{self.depth.shape}")
        #print(f"DEPTH ANALYSIS:{np.min(self.depth)}:{np.max(self.depth)}")
    # ===================================== RUN ==========================================
      
    
    # ===================================== OBJECT DETECT ========================================
    def _objectcallback(self, msg):
        """Queue object callback
        """
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        #print(image.shape)
        #time.sleep(1)
        object_lock.acquire()
        self.image = image
        # if(len(self.queue) < 100):
        #     self.queue.append(image)
        object_lock.release()
        self.img_flag = 1

    def _object(self):
        """Object detection callback
        """
        a = time.time()
        #print("down here")
        # image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        # if (len(self.queue) > 0 and len(self.d_queue) > 0):
        if(self.img_flag and self.depth_flag):
            self.img_flag, self.depth_flag = 0, 0

            object_lock.acquire()
            output_image = self.image
            # output_image = self.queue.pop(0)
            object_lock.release()

            depth_lock.acquire()
            depth = self.depth
            # depth = self.d_queue.pop(0)
            depth_lock.release()

            #PASS TO MODEL
            #image = output_image
            full_image, net_image, pad = get_image_tensor(output_image, 640) #Transform the image into tensors
            pred = self.model.forward(net_image) #Pass the tensor to the model to get a prediction
            det = pred[0]

            #Process predictions
            if len(det):
                # Rescale boxes from img_size to im0 size
                # x1, y1, x2, y2=
                det[:, :4] = self.model.get_scaled_coords(det[:,:4], output_image, pad)
                #logger.info("Detected: {}".format(s))
                
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    #c = int(cls)
                    #print(f'{self.names[c]} {conf:.2f} {xyxy}')
                    # Add bbox to image
                    c = int(cls)  # integer class                    
                    #print(f"Depth:{d.shape}")
                    d_ = depth[int(xyxy[1]):int(xyxy[3]), int(xyxy[0]):int(xyxy[2])]
                    d_c = crop_center(d_, 4, 4)
                    if(np.median(d_c) > 0 and np.median(d_c) <= 1000):
                        traffic = traffic_sign()
                        traffic.traffic_sign_type = c
                        self.traffic_light_publisher.publish(traffic)
                    #print(f"size d_:{d_.shape}")
                    #print(f"Depth of {self.model.names[c]}:{d_c}")
                    #print(f"size bbox={int(xyxy[0])}:{int(xyxy[1])}:{int(xyxy[2])}:{int(xyxy[3])}")
                    
                    label = f"Class:{self.model.names[c]}:{xyxy}"  
                    output_image = plot_one_box(xyxy, output_image, label=label, color=self.colors(c, True))
                    #visualize = cv2.hconcat([output_image[int(xyxy[1]):int(xyxy[3]), int(xyxy[0]):int(xyxy[2])], d_])
                    #d_ = cv2.applyColorMap(d_.astype(np.uint8), cv2.COLORMAP_JET)
                    cv2.imshow('Object', output_image[int(xyxy[1]):int(xyxy[3]), int(xyxy[0]):int(xyxy[2])])       
                    #cv2.imshow('Depth', d_)

                #mageObject = self.bridge.cv2_to_imgmsg(output_image, "rgb8")

                # out = pedestrian()
                # out.image = imageObject
                # out.valid, out.is_pedestrian_infront, out.x, out.y, out.h, out.w = 0,0,0,0,0,0

            cv2.imshow('Random', output_image) 
            cv2.waitKey(3)
            #tinference, tnms = self.model.get_last_inference_time()
            #print("Frame done in {}".format(tinference+tnms))
            print(f"Frame done in {time.time() - a}")

        #heatmap = self.depth
        #heatmap = cv2.applyColorMap(heatmap.astype(np.uint8), cv2.COLORMAP_HOT)
        #print(f"IMAGE:{image.shape}")        
        #print(f"DetectionProcess{net_image.shape}")
        #det = self.model.process_predictions(pred[0], output_image, pad) #Post process prediction
        #PROCESS
        #self.depth_debug_publisher.publish(self.bridge.cv2_to_imgmsg(heatmap, "rgb8"))
    
    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting objectNODE")
        #rospy.spin()
        while not rospy.is_shutdown():
            self._object() 



if __name__ == "__main__":
    perNod = objectNODE()
    perNod.run()
