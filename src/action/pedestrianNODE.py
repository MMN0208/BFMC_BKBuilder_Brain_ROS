import json
import time
import rospy

from std_msgs.msg      import String
from utils.srv         import subscribing, subscribingResponse
from utils.msg         import pedestrian

class pedestrianNODE():
    def __init__(self) -> None:
        self.pedestrian_subscriber = rospy.Subscriber("/automobile/pedestrian", pedestrian, self.check_pedestrian)
        self.lane_subscriber = rospy.Subscriber("/automobile/lane", String, self.check_lane)
        self.mid_point = 0 #TODOS: calibrate this value
        self.off_center = 0 #TODOS: calibrate this value
        self.right_lane_type = None
        self.left_lane_type = None
        self.radius_of_curvature = None
        # Float32: steer_angle
        # Float32: radius_of_curvature
        # Float32: off_centre
        # Int8: left_lane_type
        # Int8: right_lane_type
        # Int8: mid_point
        
    def check_pedestrian(self, msg):
        """Determined if a pedestrian is currently walking or not
        """
        px, py, ph, pw = msg.x, msg.y, msg.h, msg.w
        #self.midpoint = center of road
        a, b, c, d = 0, 0, 0, 0 #
        
        #TODOS: Process and return is_pedestrian (bool)
        is_pedestrian = 0
        pedestrian_infront = 0
        if (pedestrian_infront)
            while (px - pw > a and px + pw < d) is_pedestrian = 1
        
        pass
    
    def check_lane(self, msg):
        """Update lane values accordingly
        """
        self.mid_point = msg.mid_point
        self.off_center = msg.off_center #TODOS: calibrate this value
        self.right_lane_type = msg.right_lane_type
        self.left_lane_type = msg.left_lane_type
        self.radius_of_curvature = msg.radius_of_curvature
        