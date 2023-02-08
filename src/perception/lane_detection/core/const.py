import numpy as np

params_processing = dict()
params_lane_detection = dict()
""" =======================GLOBAL CONFIGURATION==================================================="""

VIDEO_PATH = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/video/bfmc2020_online_1.avi' 
SAVE_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save' 
CAL_IMG_DIR = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/calibrate_imgs' 
CALIBRATE_PICKLE = '/home/quangngcs/Desktop/Github/BFMC_BKBuilder_Brain_ROS/src/perception/lane_detection/core/save/calibration.pkl'
IMG_SIZE  = [640, 360]

""" ======================= PROCESSING PARAMS ===================================="""
params_processing['gaussian_kernel_size'] = 5
params_processing['sobel_kernel_size'] = 3
params_processing['lower_white'] = np.array([0, 160, 10])
params_processing['upper_white'] = np.array([255, 255, 255])
params_processing['src_points'] = np.array([ \
                                        [0, 340], [640, 340], [486, 145], [214, 121]  \
                                            ], dtype = np.float32)
params_processing['dst_points'] = np.array([\
                                             [0, 340], [640, 340], [640, 20], [0, 20] \
                                             ], dtype = np.float32)
""" ======================= LANE DETECTION  ======================================"""
params_lane_detection['ym_per_pix'] = 20.0 / 360.0
params_lane_detection['xm_per_pix'] = 40.0 / 640.0 
