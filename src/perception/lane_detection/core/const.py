import numpy as np
from utils import Trackbars 
import sys
trackers = Trackbars()
params_processing = dict()
params_lane_detection = dict()

""" =======================GLOBAL CONFIGURATION==================================================="""

VIDEO_PATH = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/video/bfmc2020_online_1.avi' 
SAVE_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save' 
CAL_IMG_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/calibrate_imgs' 
CALIBRATE_PICKLE = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save/calibration.pkl'
IMG_SIZE  = (1280, 720)
IMG_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/frame_test'
IMG_SRC = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/frame_test/bfmc_bkhouse_2.jpg' 
W, H = IMG_SIZE
# wTop =  489 
# hTop =  0    
# wBot =  153
# hBot =  360

# wTop = 530
# hTop = 0
# wBot = 137
# hBot = 360


wTop = 957
hTop = 0
wBot = 405
hBot = 720
""" ======================= PROCESSING PARAMS ===================================="""
params_processing['gaussian_kernel_size'] = 5
params_processing['sobel_kernel_size'] = 3
params_processing['lower_white'] = np.array([0, 160, 10])
params_processing['upper_white'] = np.array([255, 255, 255])
params_processing['dst_points'] = np.array([[wBot, hBot], [W - wBot, hBot], [wTop, hTop], [W - wTop, hTop]], dtype = np.float32)
# params_processing['dst_points'], _ = trackers.getDstView()
params_processing['src_points'] = np.array([[320, 633], [1170,  633], [853, 440], [533, 440]], dtype=np.float32)
params_lane_detection['ym_per_pix'] = 20.0 / 360.0
params_lane_detection['xm_per_pix'] = 40.0 / 640.0 

