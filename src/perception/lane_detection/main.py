import cv2
import numpy as np
from pathlib import Path
import glob
from image_process import Preprocessor

LIGHT_IMG_DIR  = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/png/light'
DARK_IMG_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/png/dark'

BEV_LIGHT = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/BEV/light'
BEV_DARK = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/BEV/dark' 
BEV_THRESH = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/BEV/thresh'
BEV_NONE_LANE = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/BEV/none'
BEV_MARKS = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/BEV/marks'
POLY_LIGHT  = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/polylines/light'
POLY_DARK = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/polylines/dark'

wTop = 957
hTop = 0
wBot = 262
hBot = 720
W = 1280
H = 720
srcPts = np.array([[10, 690], [1280, 690], [800, 317], [380, 317]], dtype = np.float32)
dstPts = np.array([[wBot, hBot], [W - wBot, hBot], [wTop, hTop], [W - wTop, hTop]], dtype = np.float32)



def projectBEV(img, srcPts, dstPts):

    H, W, C = img.shape
    project_matrix = cv2.getPerspectiveTransform(srcPts, dstPts)
    inverse_project_matrix = cv2.getPerspectiveTransform(dstPts, srcPts)

    frameBEV = cv2.warpPerspective(img, project_matrix, (W, H))
    inverseBEV = cv2.warpPerspective(img, inverse_project_matrix, (W,H))

    return frameBEV

def draw_polylines(img, points):

    points = srcPts.astype(np.int32)
    points = points.reshape(-1,1,2)

    img = cv2.polylines(img, [points], False, (0,255,0), thickness=2)

    return img

def ensemble(img):

    BEV = projectBEV(img, srcPts, dstPts)
    polylines = draw_polylines(img, srcPts)

    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return BEV, polylines

def curvatureRadius(img):

    pass

def get_lane_lines(img, lines):

    """
    Params: 
        lines: the list index get from the HoughLineP Transform
    Return:
        left_lines, right_lines

    """
    left_lines = []
    right_lines = []
    
    for line in lines:
        eps = 1e-5
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1 + eps)

        if slope < -0.5:
            left_lines.append(line)
        elif slope > 0.5:
            right_lines.append(line)
    
    return left_lines, right_lines
        

def steerAngleCenter(img, lines):
    if lines is not None:
        left_lines = []
        right_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            if slope < -0.5:
                left_lines.append(line)
            elif slope > 0.5:
                right_lines.append(line)
        
        left_slope = 0
        right_slope = 0
        for line in left_lines:
            x1, y1, x2, y2 = line[0]
            left_slope += (y2 - y1) / (x2 - x1)
        if len(left_lines) > 0:
            left_slope /= len(left_lines)
        
        for line in right_lines:
            x1, y1, x2, y2 = line[0]
            right_slope += (y2 - y1) / (x2 - x1)
        if len(right_lines) > 0:
            right_slope /= len(right_lines)
        
        avg_slope = (left_slope + right_slope) / 2
        steering_angle = avg_slope * 180 / np.pi
    else:
        steering_angle = 0
    return steering_angle

def steerAngleCurvature(img, lines, desired_radius):
    max_steering_angle = 30
    min_steering_angle = -30

    slope, intercept = [], []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope.append((y2 - y1) / (x2 - x1))
        intercept.append(y1 - slope[-1] * x1)
    avg_slope = np.mean(slope)
    avg_intercept = np.mean(intercept)

    # Calculate the steering angle based on the desired radius of the turn
    if abs(avg_slope) > 0.1:
        radius = abs(desired_radius / (2 * avg_slope))
        steering_angle = np.arctan(1 / radius) * 180 / np.pi
        if avg_slope < 0:
            steering_angle = -steering_angle
    else:
        steering_angle = 0

    # Limit the steering angle to the maximum and minimum values
    if steering_angle > max_steering_angle:
         steering_angle = max_steering_angle
    elif steering_angle < min_steering_angle:
        steering_angle = min_steering_angle

    return steering_angle 

def extract_bev_poly(LIGHT = True):
    processor = Preprocessor()
    LIGHT = True
    img_paths=  []
    count = 0
    if LIGHT:
        print(LIGHT_IMG_DIR)
        img_paths = [path for path in glob.glob(LIGHT_IMG_DIR + '/*.png')]
        dev
        if len(img_paths) <= 0:
            raise ValueError
        else:
            print("Found {} images".format(len(img_paths)))
        
        for idx, path in enumerate(img_paths):
            img = cv2.imread(path)

            BEV, polylines = ensemble(img)

            thresh_bev = processor.process(BEV)['thresh']
            BEV_filename = "BEV_"+str(idx+1)+'.png'
            Poly_filename =  "Poly_"+str(idx+1)+'.png'
            thresh_name = "Thresh_"+str(idx+1) + '.png'

            

            lines = cv2.HoughLinesP(thresh_bev, rho=1, theta=np.pi/180, threshold=20, minLineLength=20)
            angle = steerAngleCurvature(img, lines)

            print("Angle = {}".format(angle))
            cv2.putText(thresh_bev, "Angle = {}".format(angle), (300, 300), cv2.FONT_HERSHEY_PLAIN, 10, (255,255,255))
            cv2.imwrite(BEV_LIGHT + '/' + BEV_filename, BEV)
            cv2.imwrite(BEV_THRESH + '/' + thresh_name, thresh_bev)
            
        print("Count = {}".format(count))

def process():
    processor = Preprocessor()
    img_file = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/bfmc_frame/png/light/image_lanecurved_0040.png'

    frame = cv2.imread(img_file)
    output = processor.process(frame)

    cv2.imwrite("test.png", output['thresh'])

if __name__ == '__main__':


    extract_bev_poly()