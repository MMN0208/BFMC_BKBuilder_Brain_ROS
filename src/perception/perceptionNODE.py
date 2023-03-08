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
from lane_detection.core.camera import Camera
from lane_detection.core.utils import Trackbars

from object_detection.network.edgetpumodel import EdgeTPUModel
from object_detection.network.utils import plot_one_box, Colors, get_image_tensor

# tracker = Trackbars()
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
        
        # cv2.namedWindow("Perception", 1)
        # cv2.namedWindow("BEV", 1)
        # self.command_subscriber = rospy.Subscriber("/automobile/perception", String, self._write)      
        #self.command_publisher = rospy.Publisher("/automobile/perception", String)
        self.bridge = CvBridge()
        #======CAMERA======
        self.camera = Camera()
        self.camera.load_calibrate_info('/home/pi/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save/calibration.pkl')
        self._image = None
        self.lane_publisher = rospy.Publisher("/automobile/lane", perception, queue_size =1)
        self.bev_publisher = rospy.Publisher("/automobile/bev", Image, queue_size = 1)
        self.bev_thresh_publisher = rospy.Publisher("/automobile/thresh", Image, queue_size = 1)
        self.lane_info_publisher = rospy.Publisher("/automobile/lane_info",lane, queue_size = 1)
        
        self.lane_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._lane)
        self.tuning = False                     # Tune the bird eye view and other params 
        self.tune_BEV = False
        #======OBJECT DETECTION======
        # self.object_subscriber = rospy.Subscriber("/automobile/image_raw", Image, self._object)
        # self.model_path = "object_detection/weights/traffic.tflite"
        # self.names = "object_detection/data.yaml"
        # self.conf_thresh = 0.5
        # self.iou_thresh = 0.65
        # self.device = 0
        # self.model = EdgeTPUModel(self.model_path, self.names, conf_thresh=self.conf_thresh, iou_thresh=self.iou_thresh)
        # #self.model = None 
        
        # self.colors = Colors()
        # #.
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        
        rospy.loginfo("starting perceptionNODE")
        count = 1
        #rospy.spin() 
        while not rospy.is_shutdown():
            try:
                if self._image is not None:
                    
                    if not self.tuning:
                        #self.send_BEV()
                        # cv2.imshow("Perception", self._image)
                        self.send_perceptionInfo(self._image, count)
                        count += 1
            #                self.send_laneInfo(self._image)
                    else:
                        
                        calibrate_img = self.camera.undistort(self._image)
                        if self.tune_BEV:        
                            process_results = self.camera.laneDetector.processor.process(calibrate_img)

                            BEV = process_results['birdeye_img']
                            # cv2.imshow("BEV", BEV)
                            # cv2.waitKey(3)
                        else:
                            lines, _ = trackers.getSrcView()
                            lines = lines.reshape(-1, 1, 2)
                            # image = cv2.polylines(image, [lines], True, (0,255,0), 2)
                            # cv2.imshow("BEV", image)
                            # cv2.waitKey(3)
            except Exception as e:
                print(e)


            #time.sleep(0.1)        #self._read() 
        rospy.spin()   
    
    # ===================================== OBJECT DETECT ========================================
    # def _object(self, msg):
    #     """Object detection callback
    #     """
    #     image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
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
    #         label = f'{self.model.names[c]} {conf:.2f}' #Set label to the class detected
    #         command = f'DETECT:{label}:{xyxy}'
    #         #self.command_publisher.publish(command)
    #         output_image = plot_one_box(xyxy, output_image, label=label, color=self.colors(c, True)) #Plot bounding box onto output_image
        
    #     cv2.imshow("test", output_image)       
    #     cv2.waitKey(1)      
    #     tinference, tnms = self.model.get_last_inference_time()
    #     print("Frame done in {}".format(tinference+tnms))
     
    # ===================================== LANE DETECT ========================================
    
    def _lane(self, msg):
        """Lane detection callback
        """
        self._image = self.bridge.imgmsg_to_cv2(msg)

    def send_perceptionInfo(self, scene, count):
        
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
        # calibrate_scence = self.camera.undistort(scene)
        calibrate_scence = scene
        lane_detection_result = self.camera._runDetectLane(calibrate_scence)
        bev_img = lane_detection_result['BEV']
        thresh_show = lane_detection_result['thresh']
        # cv2.imshow("Perception", thresh)
        # cv.waitKey(3)
        msg = perception()
        if lane_detection_result is not None:
            # msg.steer_angle             = lane_detection_result['steer_angle']
            msg.radius_of_curvature     = lane_detection_result['radius'] 
            msg.left_lane_type          = lane_detection_result['left_lane_type'] 
            msg.left_lane_type          = lane_detection_result['right_lane_type']
            msg.one_lane                =  lane_detection_result['one_lane']
        else:
            
            # msg.steer_angle             = 0
            msg.one_lane                = 3
            msg.radius_of_curvature     = -1
            msg.left_lane_type          = 0
            msg.left_lane_type          = 0

        # cv2.imwrite(IMG_DIR + '/' + 'ckpt_' + str(count) + '.png', thresh)        
        msg.steer_angle = lane_detection_result['steer_angle']
        print("Sent angle = {}".format(msg.steer_angle))
        # msg.angle_curvature = lane_detection_result['angle_curvature']
        self.lane_publisher.publish(msg)
        # self.send_BEV(bev_img)
    

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

    def send_BEV(self, img):
        """Birdeye view callback
        Send only one message type Image (please visit ROs documentation for more information about custom message type)
        
        Note: at this time just use pseudo messages defined in TESTNODES to test the publish() command
        """
        
        #print("Image to be send: {}".format(type(image)))
        send_image = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.bev_publisher.publish(send_image)
        
    def send_BEV_thresh(self, img):
        send_image_thresh = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.bev_thresh_publisher.publish(send_image_thresh)
    # ===================================== READ ==========================================


if __name__ == "__main__":
    perNod = perceptionNODE()
    perNod.run()
