from const import *
from utils import Trackbars
import cv2 as cv
import numpy as np

tracker = Trackbars() 


if __name__ == '__main__':

    while True:

        img = cv.imread(IMG_SRC)
        #bot, top = tracker.getSrcPoints()
        #cv.rectangle(img, bot, top, (0,255,0), 1)
        
        src, srcPts = tracker.getSrcView()
        dst, dstPts = tracker.getDstView()
        
        print(srcPts)
        botSrc, topSrc = srcPts
        botDst, topDst = dstPts
        transform_mat = cv.getPerspectiveTransform(src, dst)
        birdeye = cv.warpPerspective(img, transform_mat, (640, 360))
        
        cv.rectangle(img, botSrc, topSrc, (0,255,0), 1)
        cv.rectangle(img, botDst, topDst, (255, 0, 0), 1)
        for idx in range(4):
            cv.circle(img, (int(src[idx][0], src[idx][1])), 15, (0,0,255), cv.FILLED)


        cv.imshow("Rec", img)
        cv.imshow("View", birdeye)
        cv.waitKey(1)
