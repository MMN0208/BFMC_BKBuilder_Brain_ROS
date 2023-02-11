#!/usr/bin/env python3
import json
import time
import rospy

from std_msgs.msg      import String
from utils.srv         import subscribing, subscribingResponse
from utils.msg         import pedestrian, pedestrian_output
from line_boundary_check import *
from cv_bridge       import CvBridge
import cv2
class pedestrianNODE():
    def __init__(self) -> None:
        self.pedestrian_subscriber = rospy.Subscriber("/automobile/pedestrian", pedestrian, self.check_pedestrian)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane_info", String, self.check_lane)
        self.pedestrian_publisher = rospy.Publisher("/automobile/pedestrian_test", pedestrian_output, queue_size=1)
        #
        self.boundaryLines = [
            boundaryLine([ 300,  40,  20, 400 ]),
            boundaryLine([ 440,  40, 700, 400 ])
        ]  
        
        self.areas = [
            area([ [200,200], [500,180], [600,400], [300,300], [100,360] ])
        ]
        #
        self.tracker = objectTracker()
        
    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting pedestrianNODE")
        #self._read() 
        rospy.spin()     
        
    def check_pedestrian(self, msg):
        """Determined if a pedestrian is currently walking or not
        """
        image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
        print(image)
        #if msg.is_pedestrian_infront:
        px, py, ph, pw = msg.x, msg.y, msg.h, msg.w
        #TODOS: Process and return is_pedestrian (bool)
        #       Add pedestrian tracking
        output = pedestrian_output()
        output.pedestrian = 0
        
        objects = None
        self.tracker.trackObjects(objects)
        self.tracker.evictTimeoutObjectFromDB()
        self.tracker.drawTrajectory(outimg, objects)

        checkLineCrosses(self.boundaryLines, objects)
        drawBoundaryLines(image, boundaryLines)

        checkAreaIntrusion(self.areas, objects)
        for area in self.areas:
            if area.count > 0:
                output.pedestrian = 1
        drawAreas(image, areas)
        cv2.imshow("test", image)
        cv2.waitKey(1)

        #self.pedestrian_publisher.publish(output)
            
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
        self.boundaryLines = [
            boundaryLine([msg.width_topleft, msg.height_topleft, msg.width_bottomleft, msg.height_bottomleft]),
            boundaryLine([msg.width_topright, msg.heigth_topright, msg.width_bottomright, msg.heigth_bottomright])
        ]

if __name__ == "__main__":
    print("Hello")
    perNod = pedestrianNODE()
    print(perNod)
    perNod.run()