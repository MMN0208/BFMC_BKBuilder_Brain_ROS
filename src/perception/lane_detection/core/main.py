import time
import cv2 as cv
from preprocess import Preprocessor
from laneDetect import LaneDetection
from camera import Camera
from utils import *
from const import *
import glob
#from camera import Camera
import matplotlib.pyplot as plt


NUM_FRAMES = 500 
OFFSET = 10

def draw_points(img, points):
    """ -- Support function
        This function is used to draw tracking points for better 
        choice of source and destination points 
        in performing warping transformation. 
    """    
    try:
        for idx in range(4):
            cv.circle(img, (
                (int(points[idx][0])), int(points[idx][1])
            ), 15, (0,0,255), cv.FILLED)    
        return img
    except Exception as e:
        print(e)

if __name__ == '__main__':

    # extract_frames(VIDEO_PATH, NUM_FRAMES, OFFSET)

    # preprocessor = Preprocessor()
    # lanedetector = LaneDetection()

    #   Load video for testing
    video = cv.VideoCapture(VIDEO_PATH)
    video.set(cv.CAP_PROP_FPS, 10)
    camera = Camera()
    camera.load_calibrate_info(CALIBRATE_PICKLE)
    

    while True:
        time.sleep(0.01)
        flag, frame = video.read()

        if flag:

            #   ====================    MAIN FLOW   =====================
            frame = cv.resize(frame, IMG_SIZE)
            calibrate_img = camera.undistort(frame)

#           plt.imshow(calibrate_img)
#            plt.show()
            # detection_img= camera._runDetectLane(calibrate_img)
            # detection_results = camera._runDetectLane(calibrate_img)    #   TEST
            # plottable = detection_results['lane_img']
            # print(plottable) 
            # cv.imshow("Detection", plottable)                           #TEST
            ######################### TESTING   #########################
            output = camera.laneDetector.processor.process(calibrate_img)
            thresh = output['thresh']
            
            # cv.imshow('points', points_img)
            cv.imshow('Thresh', thresh)
            # cv.imshow('Detection', detection_img)
            #############################################################
            # cv.imshow('Main', output['birdeye']['birdeye'])
            # cv.imshow("Test", out_test)
            # cv.imshow("Thresh", tmp)
            cv.waitKey(1)

    
   

