#!/usr/bin/env python3
import json
import time
import rospy

from std_msgs.msg      import String
from utils.srv         import subscribing, subscribingResponse
from utils.msg         import pedestrian, pedestrian_output
from sensor_msgs.msg import Image

#from yolov8detect      import *
from cv_bridge         import CvBridge
import cv2
import numpy as np

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


"""
Logs: 12/2/23 (hai): Added code for pedestrian detection
     - When a pedestrian is detected within a predefined area (self.points), the code will publish a pedestrian_output msg to pedestrian_test with
     values of 1 (otherwise values will be 0)
     - To run the code:
        + pip install -r requirements.txt
        + add following to your launch file
        <node pkg="perception" type="objectNODE.py" 	name="objectNODE"  		    output="screen" cwd="node"/>
        <node pkg="action" type="pedestrianNODE.py" 	name="pedestrianNODE"  		output="screen" cwd="node"/>
     - Notes:
        + Adaptive area with respect to lane is not implemented (area is always fixed)
        + Pedestrian class is not defined (currently the model treats pedestrian as a cross-walk sign)
"""
def crop_center(img,cropx,cropy):
    #print(img.shape)
    y = img.shape[0]
    x = img.shape[1]
    startx = x//2-(cropx//2)
    starty = y//2-(cropy//2)    
    return img[starty:starty+cropy,startx:startx+cropx]

class pedestrianNODE():
    def __init__(self) -> None:
        rospy.init_node('pedestrianNODE', anonymous=False)
        #self.pedestrian_subscriber = rospy.Subscriber("/automobile/pedestrian", pedestrian, self.check_pedestrian)
        #self.lane_subscriber = rospy.Subscriber("/automobile/lane_info", String, self.check_lane)
        #self.pedestrian_publisher = rospy.Publisher("/automobile/pedestrian_test", pedestrian_output, queue_size=1)
        self.bridge = CvBridge()

        self.object_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self._object)
        self.depth_subscriber = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self._depth)
        
        #PEDESTRIAN DETECTION
        ih, iw = 480, 640
        self.points = [[(iw-500)//2, (ih-500)//2], #LEFT TOP
          [(iw+500)//2,(ih-500)//2], 
          [(iw+500)//2, (ih+700)//2], 
          [(iw-500)//2, (ih+700)//2],
          [(iw-500)//2, (ih-500)//2]]
        
        self.image = np.random.rand(480, 640, 3)
        self.depth = np.random.rand(480, 640)

    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting pedestrianNODE")
        #self._read() 
        rospy.spin()     

    def _object(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        score = np.square(np.subtract(crop_center(self.image, 100, 100),crop_center(image, 100, 100))).mean()
        #print(f"SCORE SSIM:{score}")
        #print(f"SIZE:{image.shape}")
        if(score > 40):
            print(f"SCORE SSIM:{score}")
        self.image = image
        x = 320 - 50
        y = 240 - 200
        x_plus_w = 320 + 50
        y_plus_h = 240 + 50
        color = (0, 255, 0)
        cv2.rectangle(image, (int(x), int(y)), (int(x_plus_w), int(y_plus_h)), color, 2)
        #cv2.drawContours(image, self.points,-1, (0,255,0), 3)
        # show the output image
        cv2.imshow("IMAGE", image)
        #cv2.imshow('CURRENT', crop_center(image, 100, 100))
        #cv2.imshow('IMAGE', image)
        cv2.waitKey(3)        

    def _depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg)
        self.depth = crop_center(self.depth, 100, 100)
        #cv2.imshow('DEPTH_PED', self.depth)
        print(f"DEPTH_VALUE:{np.median(self.depth)}")
        cv2.waitKey(3)

    def isInside(self, points, centroid):
        polygon = Polygon(points)
        centroid = Point(centroid)
        #print(polygon.contains(centroid))
        return polygon.contains(centroid)   
     
    def draw_prediction(self, img, x, y, x_plus_w, y_plus_h, points):
        color = (0, 255, 0)
        #print(f"TYPE:{type(x), type(y)}")
        cv2.rectangle(img, (int(x), int(y)), (int(x_plus_w), int(y_plus_h)), color, 2)
        cv2.putText(img, "", (int(x - 10), int(y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Tinh toan centroid
        centroid = ((int(x) + int(x_plus_w)) // 2, (int(y) + int(y_plus_h)) // 2)
        #cv2.circle(img, centroid, 5, (color), -1)

        return self.isInside(points, centroid)
        
    def check_pedestrian(self, msg):
        """Determined if a pedestrian is currently walking or not
        """
        image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        #print(f"IMAGE FROM PEDESTRIAN:{image}")
        output = pedestrian_output()
        output.pedestrian = 0
        if(self.draw_prediction(image, msg.x, msg.y, msg.h, msg.w, self.points)):
            output.pedestrian = 1
        self.pedestrian_publisher.publish(output)
            
    def check_lane(self, msg):
        """Update lane values accordingly
        """
        """
        float32: width_topleft
        float32: height_topleft
        float32: width_topright
        float32: heigth_topright
        float32: width_bottomleft
        float32: height_bottomleft
        float32: width_bottomright
        float32: heigth_bottomright
        """
        #TODOS: Update values with lane type
        # self.boundaryLines = [
        #     boundaryLine([msg.width_topleft, msg.height_topleft, msg.width_bottomleft, msg.height_bottomleft]),
        #     boundaryLine([msg.width_topright, msg.heigth_topright, msg.width_bottomright, msg.heigth_bottomright])
        # ]

    def check_object(self):
        # np.square(np.subtract(Y_true,Y_pred)).mean()
        pass

if __name__ == "__main__":
    print("Hello")
    perNod = pedestrianNODE()
    print(perNod)
    perNod.run()