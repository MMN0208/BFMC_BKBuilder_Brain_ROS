import cv2 as cv
import numpy as np
import pickle
# from numba import float32, uint16

##################  TEST CONFIG #################################
VIDEO_PATH = 'D:\Bosch-Future-Mobility-BKBUILDER\lane_detection\\video\\bosch_test_3.mp4' 
# VIDEO_PATH = 'D:\Bosch-Future-Mobility-BKBUILDER\lane_detection\\video\project_video.mp4'
IMG_DIR = 'D:\Bosch-Future-Mobility-BKBUILDER\lane_detection\test_images'
SAVE_DIR = 'D:\Bosch-Future-Mobility-BKBUILDER\lane_detection\core\save' 
CAL_IMG_DIR =  'D:\Bosch-Future-Mobility-BKBUILDER\lane_detection\calibrate_imgs'
CALIBRATE_PICKLE = 'D:\Bosch-Future-Mobility-BKBUILDER\lane_detection\src\save\calibration.pkl'
IMG_SIZE  = [640, 360]

global image_processing_results
#####################   END TEST CONFIG  ########################

##################  HELPER FUNCTIONS    ###########################
def extract_frames(video_path, NUM_FRAMES, OFFSET):

    video = cv.VideoCapture(video_path)
    current_frame = 0
    while True:
        flag, frame = video.read()

        if flag and current_frame < NUM_FRAMES:
            cv.imwrite(IMG_DIR+ '/frame_'+str(current_frame+1) + '.png', frame)
            current_frame += OFFSET 
        else:
            break

def save_pkl(pickle_file, path):
    try:
        with open(path, 'wb') as f:
            pickle.dump(pickle_file, f, pickle.HIGHEST_PROTOCOL)
        f.close()
    except Exception as e:
        print(e)

def load_pkl(pickle_file):
    
    results = {}
    try:
        with open(pickle_file, 'rb') as f:
            results = pickle.load(f)
        print("Pickle load: {}".format(results))
        return results
    except Exception as e:
        print(e)

#####################################################################

class Trackbars:

    def __init__(self):
        pass
        #self.initPointTrackings([100, 100, 100, 100])

    def initPointTrackings(self, initVals, width = IMG_SIZE[1], height = IMG_SIZE[0]):
        """
        :params: initVals = (Width Top, Height Top, Width Bottom, Height Bottom)
        """
        cv.namedWindow('ViewPerspective')
        cv.resizeWindow("ViewPerspective", 360, 240) 
        cv.createTrackbar("Width Top", "ViewPerspective", initVals[0], width , self.doNothing)
        cv.createTrackbar("Height Top", "ViewPerspective", initVals[1], height, self.doNothing)
        cv.createTrackbar("Width Bottom", "ViewPerspective", initVals[2], width , self.doNothing)
        cv.createTrackbar("Height Bottom", "ViewPerspective", initVals[3], height, self.doNothing)

    def getValPoints(self, width = IMG_SIZE[1], height = IMG_SIZE[0]):
        
        wTop = cv.getTrackbarPos("Width Top", "ViewPerspective")
        hTop = cv.getTrackbarPos("Height Top", "ViewPerspective")
        wBot = cv.getTrackbarPos("Width Bottom", "ViewPerspective")
        hBot = cv.getTrackbarPos("Height Bottom", "ViewPerspective")
        points = np.array([[wTop, hTop], [width - wTop, hTop], [wBot, hBot], [width - wBot, hBot]], dtype = np.float32)

        return points
