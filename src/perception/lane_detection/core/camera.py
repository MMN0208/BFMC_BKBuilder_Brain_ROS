import math
import cv2 as cv
import numpy as np
# import matplotlib.pyplot as plt
from .const import SAVE_DIR
from .utils import load_pkl, save_pkl
from .laneDetect import LaneDetection
from collections import deque
import random

####CONFIG DEBUG
DEBUG_VISUAL = False
cv.namedWindow("Thresh", 1)
cv.namedWindow("Human", 1)

class Lane:

    def __init__(self, maxSamples=4):
        """
        Attributes:
        
        Methods:
            
        """
        self.maxSamples = maxSamples 
        # x values of the last n fits of the line
        self.recent_xfitted = deque(maxlen=self.maxSamples)
        # Polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]  
        # Polynomial coefficients averaged over the last n iterations
        self.best_fit = None  
        # Average x values of the fitted line over the last n iterations
        self.bestx = None
        # Was the line detected in the last iteration?
        self.detected = False 
        # Radius of curvature of the line in some units
        self.radius_of_curvature = 0 
        # Distance in meters of vehicle center from the line
        self.line_base_pos = None 
         
    def update_lane(self, ally, allx):
        # Updates lanes on every new frame
        # Mean x value 
        self.bestx = np.mean(allx, axis=0)
        # Fit 2nd order polynomial
        new_fit = np.polyfit(ally, allx, 2)
        # Update current fit
        self.current_fit = new_fit
        # Add the new fit to the queue
        self.recent_xfitted.append(self.current_fit)
        # Use the queue mean as the best fit
        self.best_fit = np.mean(self.recent_xfitted, axis=0)
        # meters per pixel in y dimension
        self.ym_per_pix = 1.0
        # meters per pixel in x dimension
        self.xm_per_pix = 1.0 
        # Calculate radius of curvature
        fit_cr = np.polyfit(ally*self.ym_per_pix, allx*self.xm_per_pix, 2)
        y_eval = np.max(ally)

        print("Update lane...")
        self.radius_of_curvature = ((1 + (2*fit_cr[0]*y_eval*self.ym_per_pix + fit_cr[1])**2)**1.5) / np.absolute(2*fit_cr[0])
        print("Update radius = {}".format(self.radius_of_curvature))

class Camera():    
    def __init__(self):
        # Stores the source 
        self.ret = None
        self.mtx = None
        self.dist = None
        self.rvecs = None
        self.tvecs = None
        
        self.objpoints = [] # 3D points in real space
        self.imgpoints = [] # 2D points in img space
    
        self.laneDetector =  LaneDetection()
        self.left_lane = Lane()
        self.right_lane = Lane()
        self.ym_per_pix = 0.05
        self.xm_per_pix = 0.03
        self.max_steer = 30
        self.min_steer = -30

        #   Left-right lane check
        self.visible_range_points = 200

    def calibrate_camera(self, imgList, nx = 9, ny = 6):
        if self.mtx == None:
            objp = np.zeros((ny*nx, 3), np.float32)
            objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

            for idx, img_file in enumerate(imgList):
                img = cv.imread(img_file, 1)
                gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                ret, corners = cv.findChessboardCorners(gray, (nx, ny), flags=cv.CALIB_CB_FAST_CHECK)
                
                if ret == True:
                    self.objpoints.append(objp)
                    self.imgpoints.append(corners)
            # print(self.objpoints, self.imgpoints)
            results = dict()
            results['ret'], results['mtx'], results['dist'], results['rvecs'], results['tvecs'] = \
                                cv.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)
            self.ret = results['ret']
            self.mtx = results['mtx']
            self.dist = results['dist']
            self.rvecs = results['rvecs']
            self.tvecs = results['tvecs']

            print("Saving calibration...")
            save_pkl(results, SAVE_DIR+'/calibration.pkl')
            print("Saved!")
            return self.mtx, self.dist

    def load_calibrate_info(self, state_dict: str):
        try:
            results = load_pkl(state_dict)
            self.ret = results['ret']
            self.mtx = results['mtx']
            self.dist = results['dist']
            self.rvecs = results['rvecs']
            self.tvecs = results['tvecs']

            assert self.ret != None, "Failed to load calibrate information"
        except Exception as e:
            print(e)

    def undistort(self, img):
        return cv.undistort(img,self.mtx,self.dist,None,self.mtx)
    
    def validate_lane_update(self, img, left_lane_inds, right_lane_inds):
        try:
            # Checks if detected lanes are good enough before updating
            img_size = (img.shape[1], img.shape[0])
            
            nonzero = img.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            
            # Extract left and right line pixel positions
            left_line_allx = nonzerox[left_lane_inds]
            left_line_ally = nonzeroy[left_lane_inds] 
            right_line_allx = nonzerox[right_lane_inds]
            right_line_ally = nonzeroy[right_lane_inds]

            # print("Left and right all x: {} {}".format(left_line_allx, right_line_allx)) 
            # Discard lane detections that have very little points, 
            # as they tend to have unstable results in most cases
            print("Left line allx: {}\nRight line allx : {}".format(len(left_line_allx), len(right_line_allx)))
            if len(left_line_allx) <= 200 and len(right_line_allx) <= 200:          # Both lanes are not detected
                self.left_lane.detected = False
                self.right_lane.detected = False
            
            if len(left_line_allx) > 200 and len(right_line_allx) > 200:            # Both lanes are detected
                self.left_lane.detected = True
                self.right_lane.detected = True
                # self.right_lane.update_lane(right_line_ally, right_line_allx)       #   Update right lane
                # self.left_lane.update_lane(left_line_ally, right_line_allx)         #   Update left lane
    
            if len(left_line_allx) > 200 and len(right_line_allx) <= 200:           #   Just left lane is detected
                self.left_lane.detected = True
                self.right_lane.detected = False
                # self.left_lane.update_lane(left_line_ally, left_line_allx)          #   Update left lane

            if len(left_line_allx) <= 200 and len(right_line_allx) > 200:            #   Just right lane is detected
                self.left_lane.detected = False
                self.right_lane.detected = True
                # self.right_lane.update_lane(right_line_ally, right_line_allx)       #   Update right lane

            left_x_mean = np.mean(left_line_allx, axis=0)
            right_x_mean = np.mean(right_line_allx, axis=0)
            lane_width = np.subtract(right_x_mean, left_x_mean)

             
            # Discard the detections if lanes are not in their repective half of their screens
            if left_x_mean > 400 or right_x_mean < 400:
                self.left_lane.detected = False
                self.right_lane.detected = False
                return
            
            # Discard the detections if the lane width is too large or too small
            if  lane_width < 300 or lane_width > 640:
                self.left_lane.detected = False
                self.right_lane.detected = False
                return 
            
            # If this is the first detection or 
            # the detection is within the margin of the averaged n last lines 
            if self.left_lane.bestx is None or np.abs(np.subtract(self.left_lane.bestx, np.mean(left_line_allx, axis=0))) < 100:
                print("Update left lane")
                self.left_lane.update_lane(left_line_ally, left_line_allx)
                self.left_lane.detected = True
            else:
                self.left_lane.detected = False

            if self.right_lane.bestx is  None or np.abs(np.subtract(self.right_lane.bestx, np.mean(right_line_allx, axis=0))) < 100:
                print("Update right lane")
                self.right_lane.update_lane(right_line_ally, right_line_allx)
                self.right_lane.detected = True
            else:
                self.right_lane.detected = False    
        
            # Calculate vehicle-lane offset
            xm_per_pix = 1.0 
            car_position = img_size[0]/2
            l_fit = self.left_lane.current_fit
            r_fit = self.right_lane.current_fit
            left_lane_base_pos = l_fit[0]*img_size[1]**2 + l_fit[1]*img_size[1] + l_fit[2]
            right_lane_base_pos = r_fit[0]*img_size[1]**2 + r_fit[1]*img_size[1] + r_fit[2]
            lane_center_position = (left_lane_base_pos + right_lane_base_pos) /2
            self.left_lane.line_base_pos = (car_position - lane_center_position) * xm_per_pix +0.2
            self.right_lane.line_base_pos = self.left_lane.line_base_pos
            self.left_lane.update_lane(left_line_ally, left_line_allx)
            self.right_lane.update_lane(right_line_ally, right_line_allx)

        except Exception as e:
            print(e)

    def compute_radius(self, ally, allx):
        try:
            allx = allx[::-1]           # Reverse 
            fit_cr = np.polyfit(ally*self.ym_per_pix, allx*self.xm_per_pix, 2)
            y_eval = np.max(ally)
            radius = ((1 + (2*fit_cr[0]*y_eval*self.ym_per_pix + fit_cr[1])**2)**1.5) / np.absolute(2*fit_cr[0])
            return radius
        except Exception as e:
            print(e)
            return 0

    def find_lanes(self, img):

        results = dict()
        out_img = None
        left_lane_inds = []
        right_lane_inds = []
        left_lane_coor = None
        right_lane_coor = None
        results['angle_change'] = False

        if self.left_lane.detected and self.right_lane.detected:  # Perform margin search if exists prior success.
        
            # Margin Search
            left_fit = self.left_lane.current_fit
            right_fit = self.right_lane.current_fit       
            margin_search_result = self.laneDetector.margin_search(img, left_fit, right_fit)
            
            if margin_search_result is not None:
                left_lane_inds = margin_search_result['left_lane_inds']
                right_lane_inds = margin_search_result['right_lane_inds']
                out_img = margin_search_result['out_img']

                left_lane_coor = margin_search_result['left']
                right_lane_coor = margin_search_result['right'] 
                # Update the lane detections
                self.validate_lane_update(img, left_lane_inds, right_lane_inds)
                results['angle_change'] = True
        else:  
            # Perform a full window search if no prior successful detections.
            # Window Search
            window_search_result = self.laneDetector.slide_window_search(img)
            
            if window_search_result is not None:
                left_lane_inds = window_search_result['left_lane_inds']
                right_lane_inds = window_search_result['right_lane_inds']
                out_img = window_search_result['out_img']

                left_lane_coor = window_search_result['left']
                right_lane_coor = window_search_result['right']
                # Update the lane detections
                self.validate_lane_update(img, left_lane_inds, right_lane_inds)
                
                results['angle_change'] = True
        

        #   Calculate the radius of each side
        img_size = (img.shape[1], img.shape[0])
        if img_size[0] != 640:
            raise ValueError("Width must be 1280")
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        # Extract left and right line pixel positions
        left_line_allx = nonzerox[left_lane_inds]
        left_line_ally = nonzeroy[left_lane_inds] 
        right_line_allx = nonzerox[right_lane_inds]
        right_line_ally = nonzeroy[right_lane_inds]
        #   Left lane
        left_lane_radius = self.compute_radius(left_line_ally, left_line_allx)
        right_lane_radius = self.compute_radius(right_line_ally, right_line_allx)
        #   Right lane

        print("Left line all x = \nRight line all x = {}".format(len(left_line_allx), len(right_line_allx))) 
        

        results['out_img'] = out_img
        results['left_lane_inds'] = left_lane_inds
        results['right_lane_inds'] = right_lane_inds
        results['left_line_allx'] = left_line_allx
        results['right_line_allx'] = right_line_allx
        results['left_lane_type'] = 1
        results['right_lane_type'] = 0
        results['radius'] = (left_lane_radius + right_lane_radius) / 2
        
        # print("Left lane coordinates: {}".format(left_lane_coor))
        # print("Right lane coordinates: {}".format(right_lane_coor))
        return results 

    def get_radiusCurvature(self):
        if self.left_lane.radius_of_curvature is not None and self.right_lane.radius_of_curvature is not None:
            radius_of_curvature = (self.left_lane.radius_of_curvature + self.right_lane.radius_of_curvature)/2.0
            print("Both lane detected")
            print("Left radius = {}\nRight radius = {}".format(self.left_lane.radius_of_curvature, self.right_lane.radius_of_curvature)) 
            return radius_of_curvature  
        
        elif self.left_lane.radius_of_curvature is None:
            print("Just right lane")
            radius = self.right_lane.radius_of_curvature
            print("Right lane radius = {}".format(self.right_lane.radius_of_curvature))
            return radius

        elif self.right_lane.radius_of_curvature is None:
            print("Just left lane")
            radius = self.left_lane.radius_of_curvature
            print("Left lane radius = {}".format(self.left_lane.radius_of_curvature))
            print(radius)
            return radius

        return 0

    def write_stats(self, img):
        
        font = cv.FONT_HERSHEY_PLAIN
        size = 3
        weight = 2
        color = (255,255,255)
        
        radius_of_curvature = self.get_radiusCurvature()    
        cv.putText(img,'Lane Curvature Radius: '+ '{0:.2f}'.format(radius_of_curvature)+'m',(30,60), font, size, color, weight)

        if (self.left_lane.line_base_pos >=0):
            cv.putText(img,'Vehicle is '+ '{0:.2f}'.format(self.right_lane.line_base_pos*100)+'cm'+ ' Right of Center',(30,100), font, size, color, weight)
        else:
            cv.putText(img,'Vehicle is '+ '{0:.2f}'.format(abs(self.left_lane.line_base_pos)*100)+'cm' + ' Left of Center',(30,100), font, size, color, weight)
    

    def draw_lane(self, undist, img, Minv):
        # Generate x and y values for plotting
        ploty = np.linspace(0, undist.shape[0] - 1, undist.shape[0])
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(img).astype(np.uint8)
        color_warp = np.stack((warp_zero, warp_zero, warp_zero), axis=-1)

        left_fit = self.left_lane.best_fit
        right_fit = self.right_lane.best_fit
        
        # print("Left fit and right fit when draw lines: {} {}".format(left_fit, right_fit))

        if left_fit is not None and right_fit is not None:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]

            # Recast the x and y points into usable format for cv.fillPoly()
            pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
            pts = np.hstack((pts_left, pts_right))
            
            # print("Pts in draw lane = {}".format(pts))
            # Draw the lane onto the warped blank image
            cv.fillPoly(color_warp, np.int_([pts]), (64, 224, 208))
            
            # Warp the blank back to original image space using inverse perspective matrix (Minv)
            newwarp = cv.warpPerspective(color_warp, Minv, (img.shape[1], img.shape[0])) 
            
            # Combine the result with the original image
            result = cv.addWeighted(undist, 1, newwarp, 1.3, 0)
            self.write_stats(result)

            return result

        return undist

    def generate_output(self, warped, threshold_img, polynomial_img, lane_img):

        
        fontScale=1
        thickness=2
        fontFace = cv.FONT_HERSHEY_PLAIN
        out_img = np.zeros((720, 1280,3), np.uint8)
        out_img[:360, :640, :] = lane_img
        cv.putText(out_img, "Detection", (50, 100), fontFace, fontScale, (0,0,255), thickness, lineType = cv.LINE_AA)
        
        
        # Perspective transform image
        out_img[360:, :640:,:] = cv.resize(warped,(640, 360))
        boxsize, _ = cv.getTextSize("Transformed", fontFace, fontScale, thickness)
        cv.putText(out_img, "Transformed", (400, 400), fontFace, fontScale,(0,0,255), thickness,  lineType = cv.LINE_AA)
    
        # Threshold image
        resized = cv.resize(threshold_img,(640, 360))
        resized=np.uint8(resized)
        gray_image = cv.cvtColor(resized*255, cv.COLOR_GRAY2RGB)
        out_img[:360, 640:, :] = cv.resize(gray_image,(640, 360))
        boxsize, _ = cv.getTextSize("Filtered", fontFace, fontScale, thickness)
        cv.putText(out_img, "Filtered", (100, 700), fontFace, fontScale,(255,255,255), thickness,  lineType = cv.LINE_AA)
    
        # Polynomial lines
        out_img[360:, 640:, :] = cv.resize(polynomial_img,(640, 360))
        boxsize, _ = cv.getTextSize("Detected Lanes", fontFace, fontScale, thickness)
        cv.putText(out_img, "Detected Lanes", (int(1494-boxsize[0]/2),521), fontFace, fontScale,(255,255,255), thickness,  lineType = cv.LINE_AA)
        
        return out_img 
    
    def angleCalculator(self, img_angle):

        angleDegree = 0
        offset_x =  320
        offset_y =  480
        # ratio_x = 1
        # ratio_y = 1 

        # img_angle = cv.resize(img_angle, (144, 144))
        
        center_x, center_y = self.computeCenter(img_angle)
        # center_x = center_x * ratio_x
        # center_y = center_y * ratio_y 
        
        centers = dict()
        slope = 0
        
        if center_x != 0 or center_y != 0:
            slope = (center_x - offset_x) / float (center_y - offset_y) # (72, 144) is center of (144, 144) image
            angleRadian = np.arctan(slope)
            angleDegree = float(angleRadian * 180.0 / math.pi)

        centers['x'] = center_x
        centers['y'] = center_y
        centers['slope'] = slope
        return angleDegree, centers
    
    def computeCenter(self, roadImg):

        threshold = 240 
        center_x = 0
        center_y = 0
        
        roadImg = roadImg.astype(np.int32)
        roadImg = roadImg * 255 

        # count = 0
        #     
        # for i in range(0, 144):
        #     for j in range(0, 144):
        #         if roadImg[i][j] >= 240:
        #             count += 1
        #             center_x += j
        #             center_y += i

        # if center_x != 0 or center_y != 0 or count != 0:
        #     center_x = center_x / count
        #     center_y = center_y / count

        pixels_indices = np.argwhere(roadImg >= threshold)
        stats_pixels = pixels_indices.sum(axis =0 )                 # Return matrix of nx2
        count = stats_pixels.shape[0]
        center_x = stats_pixels[1] / count
        center_y = stats_pixels[0] / count

        if  center_x != 0 or  center_y != 0 or count != 0:
            center_x = center_x / count
            center_y = center_y / count    

        return center_x, center_y
    
    def steerAngleCurvature(self, desired_radius, img):
       
        steering_angle = 0
        max_steering_angle = self.max_steer 
        min_steering_angle = self.min_steer
        eps = 1e-6
        offset_x = 100
        offset_y = 195

        angleDegree, centers = self.angleCalculator(img)
        slope = centers['slope']
        # Calculate the steering angle based on the desired radius of the turn
        if abs(slope) > 0.1:
            print("Desired radius = {}".format(desired_radius)) 
            radius = abs(desired_radius / (2 * slope + eps))
            print("Radius = {}".format(radius))
            steering_angle = np.arctan(1 / (eps + radius)) * 180 / np.pi
            if slope < 0:
                steering_angle = -steering_angle
        else:
            steering_angle = 0
        print("Angle in camera = {}".format(steering_angle))
        # Limit the steering angle to the maximum and minimum values
        if steering_angle > max_steering_angle:
            steering_angle = max_steering_angle
        elif steering_angle < min_steering_angle:
            steering_angle = min_steering_angle
        print("Curvature angle = {}".format(steering_angle))
        return steering_angle * 0.5 + angleDegree * 0.5


    def _runDetectLane(self, img):

        preprocess_results= self.laneDetector.processor.process(img)
        warped = preprocess_results['birdeye_img']
        thresh = preprocess_results['thresh']
        
        inverse_transform = preprocess_results['inverse_transform']

        
        ####    Slide window search
        find_lane_result = self.find_lanes(thresh)
        find_lane_result['steer_angle'] = random.randint(-1, 1)
        
        ##  Check left lane and right lane
        left_line_allx = find_lane_result['left_line_allx']
        right_line_allx = find_lane_result['right_line_allx']
        find_lane_result['one_lane'] = 3
        
        if len(left_line_allx) >= self.visible_range_points and len(right_line_allx) >= self.visible_range_points:        #     Two lanes are detected
            find_lane_result['one_lane'] = 0
        
        if len(left_line_allx) >= self.visible_range_points and len(right_line_allx) < self.visible_range_points:          #    Just left lane
            find_lane_result['one_lane'] = 1

        if len(left_line_allx) <self.visible_range_points and len(right_line_allx) >= self.visible_range_points:          #    Just right lane
            find_lane_result['one_lane'] = 2 

        output_img = find_lane_result['out_img']

        find_lane_result['thresh'] = thresh
        find_lane_result['BEV'] = warped
        
        ###     Hough lines transform
        Hough_lines = cv.HoughLinesP(thresh, rho=1, theta=np.pi/180, threshold=20, minLineLength=20)
        desired_radius = find_lane_result['radius']
        steer_angle_curvature = self.steerAngleCurvature(desired_radius, thresh)
        
        if find_lane_result['angle_change']:
            steer_angle, centers  = self.angleCalculator(thresh)
            #steer_angle = self.computeAngleTest(thresh)
            find_lane_result['steer_angle'] = steer_angle

        center_x = centers['x']
        center_y = centers['y']

        center_x = int(center_x)
        center_y = int(center_y)

        string = "Left line all x = {}\nRight line all x = {}".format(len(left_line_allx), len(right_line_allx))
        print("Center x = {}\nCenter y = {}".format(center_x, center_y))

        start_point = (400, 480)
        end_point = (center_x, center_y)

        # thresh = cv.putText(thresh, string, (160, 160), cv.FONT_HERSHEY_PLAIN, 60, (0,255,0), 10, cv.LINE_AA)
        thresh = cv.line(thresh, start_point, end_point, color = (255, 255,0), thickness = 50)  
            # steer_angle = 0.5 * steer_angle + 0.5 * steer_angle_curvature
        # cv.imshow("Thresh", thresh)
        # cv.imshow("Human", warped)
        # cv.waitKey(3)
        find_lane_result['angle_curvature'] = int(math.ceil(steer_angle_curvature))
        ################## Visualization #################
        if DEBUG_VISUAL:
            lane_img = self.draw_lane(img, thresh, inverse_transform)
            finalImg = self.generate_output(warped=warped, threshold_img=thresh, polynomial_img=output_img, lane_img=lane_img)
        ##################################################

        """Testing"""
        if DEBUG_VISUAL:
            test_results = dict()
            test_results['out_img'] = output_img
            test_results['lane_img'] = lane_img
            test_results['finalImg'] = finalImg
            return test_results, find_lane_result       # Test results
        
        return find_lane_result
