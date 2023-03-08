import numpy as np
#from .utils import Trackbars 
import sys
#trackers = Trackbars()
params_processing = dict()
params_lane_detection = dict()

""" =======================GLOBAL CONFIGURATION==================================================="""

# VIDEO_PATH = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/video/bfmc2020_online_1.avi' 
SAVE_DIR = '/home/pi/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save' 
CAL_IMG_DIR = '/home/pi/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/calibrate_imgs' 
CALIBRATE_PICKLE = '/home/pi/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save/calibration.pkl'
IMG_SIZE  = (640, 480)
# IMG_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/frame_test'
# IMG_SRC = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/frame_test/frame_1001.png' 
IMG_DIR = "/home/pi/BFMC_BKBuilder_Brain_ROS/src/perception/checkpoints"
W, H = IMG_SIZE
wTop =  570
hTop =  37
wBot =  127 
hBot =  480

# wTop = 426
# hTop = 0
# wBot = 169
# hBot = 360
""" ======================= PROCESSING PARAMS ===================================="""
params_processing = {
    'gaussian_kernel_size':     5,
    'sobel_kernel_size':        3,
    'lower_white':              np.array([0, 150, 10]),
    'upper_white':              np.array([250, 250, 255]),
    'dst_points':               np.array([[wBot, hBot], [W - wBot, hBot], [wTop, hTop], [W - wTop, hTop]], dtype = np.float32),
    'src_points':               np.array([[0, 315], [535, 318], [428, 213], [125, 207]], dtype=np.float32),

    'thd_highlight_L':                  255,                # mid L         # low-pass filter on H-chanel
    'thd_highlight_S':                  0,                  # low S         # low-pass filter on H-chanel
    'thd_shadow_L':                     30,                 # low L
    'thd_shadow_S':                     50,                 # high S       
    'thd_S_mag':                        25,                 # high S_mag 25
    'thd_S_arg':                        0,                  # high S_arg 25
    'thd_S_x':                          0,                  # high-pass filter on sobel of L-chanel in direction x, by defaut 35
    'thd_L_mag':                        20,                 # high L_mag 25   # high-pass filter on magnitude of sobel of L-chanel, by defaut 5
    'thd_L_arg':                        0,                      
    'thd_L_y':                          75    
}

""" ======================= LANE DETECTION  ======================================"""

params_lane_detection = {
    'ym_per_pix':               1.0,
    'xm_per_pix':               1.0,

}


