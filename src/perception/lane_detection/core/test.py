import time
from const import *
from utils import Trackbars, order_points, four_point_transform, extract_frames
import cv2 as cv
import numpy as np
from preprocess import Preprocessor
tracker = Trackbars() 


if __name__ == '__main__':
    # while True:
    #     img = cv.imread(IMG_SRC)
    #     img = cv.resize(img, (640, 360))
    #     # src, _ = tracker.getSrcView()
        
    #     src = params_processing['src_points']
    #     # src = src.astype(np.int32)
    # #     polysrc = src.astype(np.int32)
    #     cv.polylines(img, [src], True, (255,255,0))
    #     warped = four_point_transform(img, src)
    #     # cv.imshow('a', warped)
    #     # cv.waitKey(1)
    #     cv.imshow('as', img)
    #     cv.waitKey(1)
    
    video = cv.VideoCapture(VIDEO_PATH)
    preprocessor = Preprocessor()
    while True:


        # frame = cv.imread(IMG_SRC)
        # src = params_processing['src_points']
        # dst = params_processing['dst_points']


        # transform_mat = cv.getPerspectiveTransform(src, dst)
        # inverse_mat = cv.getPerspectiveTransform(dst, src)
        # birdeye = cv.warpPerspective(frame, transform_mat, (640, 360))
        # humaneye = cv.warpPerspective(frame, inverse_mat, (640, 360))
        
        flag, frame = video.read()
        frame = cv.resize(frame, (640, 360))
        if flag:
            
            # while True:
            # src = params_processing['src_points']
            src = params_processing['src_points']
            # dst = tracker.getValPoints()
            # dst, _ = tracker.getDstView()
            dst = params_processing['dst_points']

            
            transform_mat = cv.getPerspectiveTransform(src, dst)
            inverse_mat = cv.getPerspectiveTransform(dst, src)
            birdeye = cv.warpPerspective(frame, transform_mat, (640, 360))
            humaneye = cv.warpPerspective(frame, inverse_mat, (640, 360))

            thresh = preprocessor.process(birdeye)['thresh']


            cv.imshow("Rec", frame)
            cv.imshow("View", birdeye)
            cv.imshow("Human", humaneye)
            cv.imshow("thresh", thresh)
            cv.waitKey(1)
