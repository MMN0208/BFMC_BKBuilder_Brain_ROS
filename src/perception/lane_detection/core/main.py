import time
import cv2 as cv
from preprocess import Preprocessor
from laneDetect import LaneDetection
from camera import Camera, DEBUG_VISUAL
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

    # [0, 350], [640, 350], [400, 50], [100, 50]
    
    while True:
        try:
            flag, frame = video.read()
            if flag:
                
                
    # #             #   ====================    MAIN FLOW   =====================
                # frame = cv.resize(frame, IMG_SIZE)
                frame = cv.imread(IMG_SRC)
                calibrate_img = camera.undistort(frame)
               
    # #             # # plt.imsave(SAVE_DIR + 'bev.jpeg', calibrate_img)
    # #             # # plt.imshow(calibrate_img)
    # #             # # plt.show()

    # #             # # if DEBUG_VISUAL:
    # #             # #     test_result, preprocess_results, detection_results = camera._runDetectLane(calibrate_img)    #   TEST
    # #             # # else:
    # #             # detection_results = camera._runDetectLane(calibrate_img)

    # #             # plottable = detection_results['thresh']

    # #             # steer = detection_results['steer_angle']
    # #             # print("Angle = {}".format(steer))
                
    # #             # if DEBUG_VISUAL:
    # #             #     test_img = np.zeros((720, 1280))
    # #             #     transform_mat = preprocess_results['transformed_view']
    # #             #     inverse_mat = preprocess_results['inverse_transform']
    # #             #     birdeye = preprocess_results['birdeye_img']
    # #             #     humanview = cv.warpPerspective(birdeye, inverse_mat, (640, 360))


    # #             #     # cv.imshow("BEV", birdeye)
    # #             #     # cv.imshow("Normal View", humanview)
    # #             # cv.imshow("Detection", plottable)                  #TEST
    # #             ######################### TESTING   #########################
                while True:
                    output = camera.laneDetector.processor.process(calibrate_img)
                    thresh = output['birdeye_img']
                    # thresh = output['thresh']  # test
                    cv.imshow('Thresh', thresh)
    #             # # # cv.imshow('points', points_img)
                
    #             # # # cv.imshow('Detection', detection_img)
    #             # #############################################################
    #             # # cv.imshow('Main', output['birdeye']['birdeye'])
    #             # # cv.imshow("Test", out_test)
    #             # # cv.imshow("Thresh", tmp)
                    cv.waitKey(1)

        except Exception as e:
            print(e)
    
    # frame = cv.imread(IMG_SRC)
    # frame = cv.resize(frame, IMG_SIZE)
    # calibrate_img = camera.undistort(frame)

    # # src, _ = trackers.getSrcView()


    # img_copy = calibrate_img.copy()
    # # lines = src.reshape(-1,1,2)
    # lines = np.array([[0, 315], [535,  318], [428, 213], [125, 207]], dtype=np.int32).reshape(-1,1,2)
    
    # while True:

    #     # lines,_ = trackers.getSrcView()
    #     points = lines.reshape(-1,1,2)
    #     solid = cv.polylines(calibrate_img, [points], True, (0,255,0), 2)
    #     calibrate_img = img_copy
        
    #     calibrate_img = img_copy
    #     cv.imshow("img", solid)
        # cv.waitKey(1)
    # cv.destroyAllWindows()
            
        


   

