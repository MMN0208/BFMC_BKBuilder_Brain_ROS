# from numba imprt njit
import time
import cv2 as cv
import numpy as np
import pickle
import sys
from const import *
# from numba import float32, uint16

##################  TEST CONFIG #################################
TESTNODES = dict()
TESTNODES['BEV'] = np.random.randn(640, 360,3)    #   Bird eye view image 
TESTNODES['STEER_ANGLE'] = 30.2                 #   Steering angle
TESTNODES['RADIUS'] = 12.2                      #   Radius of curvature
TESTNODES['OFF_CENTRE'] = 23.2                  #   Off center  
TESTNODES['LLT'] = 0                            #   Left lane type
TESTNODES['RLT'] = 1                            #   Right lane type
TESTNODES['MIDPOINT'] = 320                     #   Mid point

TESTNODES['WTL'] = 30                            #   width top left
TESTNODES['HTL'] = 50                            #   height top left
TESTNODES['WTR'] = 600                           #   width top right
TESTNODES['HTR'] = 50                           #   height top right
TESTNODES['WBL'] = 30                           #   width bottom left
TESTNODES['HBL'] = 350                          #   height bottom left
TESTNODES['WBR'] = 600                          #   width bottom right
TESTNODES['HBR'] = 50                            #   height bottom right
#####################   END TEST CONFIG  ########################

WIDTH = 640
HEIGHT = 480
##################  HELPER FUNCTIONS    ###########################
def extract_frames(video_path, NUM_FRAMES, OFFSET):

    video = cv.VideoCapture(video_path)
    current_frame = 0
    while True:
        flag, frame = video.read()

        if flag :
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
def order_points(pts):
	rect = np.zeros((4, 2), dtype = "float32")
	# the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
	# now, compute the difference between the points, the
	# top-right point will have the smallest difference,
	# whereas the bottom-left will have the largest difference
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]
	# return the ordered coordinates
	return rect

def four_point_transform(image, pts):
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
	dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")

	print("Destination: {}".format(dst))
	print("Source: {}".format(rect))
	# compute the perspective transform matrix and then apply it
	M = cv.getPerspectiveTransform(rect, dst)
	warped = cv.warpPerspective(image, M, (maxWidth, maxHeight))
	# return the warped image
	return warped, dst
class Trackbars:

    def __init__(self):
        self.initPointTrackings([0, 299, 0, 270])
		
        self.initSrcTracking()
        self.initSrcView()
        self.initDstView()

    def initSrcView(self, width = WIDTH, height = HEIGHT):
        cv.namedWindow('SrcView')
        cv.resizeWindow("SrcView", 360, 240)
        
        cv.createTrackbar("WTL", "SrcView", 249 ,width , self.doNothing)			# 	width top left
        cv.createTrackbar("HTL", "SrcView", 123, height, self.doNothing)		# 	height top left
        cv.createTrackbar("WBL", "SrcView", 0, width , self.doNothing)			#	width bottom leftj
        cv.createTrackbar("HBL", 'SrcView', 315, height, self.doNothing)		#	height bottom left
        cv.createTrackbar('WTR', 'SrcView', 428, width , self.doNothing)			#	width top right
        cv.createTrackbar('HTR', 'SrcView', 130, height , self.doNothing)			#	height top right
        cv.createTrackbar('WBR', 'SrcView', 626, width , self.doNothing)			#	width bottom right
        cv.createTrackbar('HBR', 'SrcView', 318, height , self.doNothing)			#	height bottom right
        
         
    def initDstView(self, width = WIDTH, height = HEIGHT):
        H = height
        W = width 
        cv.namedWindow('DstView')
        cv.resizeWindow("DstView", 360, 240)
        cv.createTrackbar("WTL", "DstView", width, width , self.doNothing)			# 	width top left
        cv.createTrackbar("HTL", "DstView", height, height, self.doNothing)		# 	height top left
        cv.createTrackbar("WBL", "DstView", 0, width , self.doNothing)			#	width bottom leftj
        cv.createTrackbar("HBL", 'DstView', 0, height, self.doNothing)		#	height bottom left
        cv.createTrackbar('WTR', 'DstView', 0, width , self.doNothing)			#	width top right
        cv.createTrackbar('HTR', 'DstView', H, height , self.doNothing)			#	height top right
        cv.createTrackbar('WBR', 'DstView', W, width , self.doNothing)			#	width bottom right
        cv.createTrackbar('HBR', 'DstView', 0, height , self.doNothing)			#	height bottom right
    
	
    def getSrcView(self):
        wtl = cv.getTrackbarPos('WTL', 'SrcView')
        htl = cv.getTrackbarPos('HTL', 'SrcView')
        wbl = cv.getTrackbarPos('WBL', 'SrcView')
        hbl = cv.getTrackbarPos('HBL', 'SrcView')
        wtr = cv.getTrackbarPos('WTR', 'SrcView')
        htr = cv.getTrackbarPos('HTR', 'SrcView')
        wbr = cv.getTrackbarPos('WBR', 'SrcView')
        hbr = cv.getTrackbarPos('HBR', 'SrcView')
        srcPts= np.array([[wbl, hbl], [wbr, hbr], [wtr, htr], [wtl, htl]], dtype=np.int32)
        # srcPts = np.array([[wtl, htl], [wtr, htr], [wbr, hbr], [wbl, hbl]], dtype=np.float32)
        return srcPts, [(wbl, hbl), (wtr, htr)]
	
    def getDstView(self):
        wtl = cv.getTrackbarPos('WTL', 'DstView')
        htl = cv.getTrackbarPos('HTL', 'DstView')
        wbl = cv.getTrackbarPos('WBL', 'DstView')
        hbl = cv.getTrackbarPos('HBL', 'DstView')
        wtr = cv.getTrackbarPos('WTR', 'DstView')
        htr = cv.getTrackbarPos('HTR', 'DstView')
        wbr = cv.getTrackbarPos('WBR', 'DstView')
        hbr = cv.getTrackbarPos('HBR', 'DstView')
        dstPts= np.array([[wbl, hbl], [wbr, hbr], [wtr, htr], [wtl, htl]], dtype=np.float32)
        return dstPts, [(wbl, hbl), (wtr, htr)]

    def initSrcTracking(self):
        cv.namedWindow("Src")
        cv.resizeWindow("Src", 300, 300)
        cv.createTrackbar("WidthSrcBottom", "Src", 50, 640, self.doNothing)
        cv.createTrackbar("HeightSrcBottom", "Src", 350, 360, self.doNothing)
        cv.createTrackbar("WidthSrcTop", "Src", 500, 640, self.doNothing)
        cv.createTrackbar("HeightSrcTop", "Src",  50, 360, self.doNothing)
    
    def getSrcPoints(self):
        wBot = cv.getTrackbarPos("WidthSrcBottom", "Src")
        hBot = cv.getTrackbarPos("HeightSrcBottom", "Src")
        wTop = cv.getTrackbarPos("WidthSrcTop", "Src")
        hTop = cv.getTrackbarPos("HeightSrcTop", "Src")
        return [(wBot, hBot), (wTop, hTop)]

    def initPointTrackings(self, initVals, width = WIDTH, height = HEIGHT):
        """
        :params: initVals = (Width Top, Height Top, Width Bottom, Height Bottom)
        """
        cv.namedWindow('ViewPerspective')
        cv.resizeWindow("ViewPerspective", 360, 240)
        cv.createTrackbar("Width Top", "ViewPerspective", initVals[0], width , self.doNothing)
        cv.createTrackbar("Height Top", "ViewPerspective", initVals[1], height, self.doNothing)
        cv.createTrackbar("Width Bottom", "ViewPerspective", initVals[2], width , self.doNothing)
        cv.createTrackbar("Height Bottom", "ViewPerspective", initVals[3], height, self.doNothing)
    
    
    def getValPoints(self, width = WIDTH, height = HEIGHT):
        wTop = cv.getTrackbarPos("Width Top", "ViewPerspective")
        hTop = cv.getTrackbarPos("Height Top", "ViewPerspective")
        wBot = cv.getTrackbarPos("Width Bottom", "ViewPerspective")
        hBot = cv.getTrackbarPos("Height Bottom", "ViewPerspective")
        points = np.array([[wBot, hBot], [width - wBot, hBot], [wTop, hTop], [width - wTop, hTop]], dtype = np.float32)
        return points
    
    def initLowerWhite(self):

        cv.namedWindow("LowerWhite")
        cv.resizeWindow("LowerWhite", 360, 240)
        cv.createTrackbar("Num1", "LowerWhite",0, 255, self.doNothing)
        cv.createTrackbar("Num2", "LowerWhite",0, 255, self.doNothing)
        cv.createTrackbar("Num3", "LowerWhite",0, 255, self.doNothing)
    
    def getLowerWhite(self):

        num1 = cv.getTrackbarPos("Num1", "LowerWhite")
        num2 = cv.getTrackbarPos("Num2", "LowerWhite")
        num3 = cv.getTrackbarPos("Num3", "LowerWhite")

        return np.array([num1, num2, num3])

    def initUpperWhite(self):

        cv.namedWindow("UpperWhite")
        cv.resizeWindow("UpperWhite", 360, 240)
        cv.createTrackbar("Num1", "UpperWhite",0, 255, self.doNothing)
        cv.createTrackbar("Num2", "UpperWhite",0, 255, self.doNothing)
        cv.createTrackbar("Num3", "UpperWhite",0, 255, self.doNothing)
    
    def  getUpperWhite(self):

        num1 = cv.getTrackbarPos("Num1", "UpperWhite")
        num2 = cv.getTrackbarPos("Num2", "UpperWhite")
        num3 = cv.getTrackbarPos("Num3", "UpperWhite")

        return np.array([num1, num2, num3])

    def doNothing(self):
        pass

def measure_time(func):

    def wrapper(*args, **kwargs):
        start = time.time() 
        func(*args) 
        end = time.time()

        print("Time elapsed in {} = {}" .format(func.__name__,  end - start))
        
    return wrapper
        
@measure_time
def brute_force(mat):
    
    H, W = mat.shape
    center_x = 0
    center_y = 0
    count = 0
    for i in range(H):
        for j in range(W):

            if mat[i][j] >= 240:
                center_x += j
                center_y += i
                count += 1

    center_x = center_x / count
    center_y = center_y / count
    print("Center x = {} -- Center y = {}" .format(center_x, center_y, __name__))
# @measure_time
# def flatten(mat):
#     H , W = mat.shape
#     return np.reshape(mat, (H*W, 1))

@measure_time
def better_way(mat):
    indices = np.argwhere(mat >= 240)
    sum = indices.sum(axis = 0)
    print(sum)
    print(indices.shape[0])
    center_x = sum[1] / indices.shape[0]
    center_y = sum[0] / indices.shape[0]
    print("Center x = {} -- Center y = {}. " .format(center_x, center_y, __name__))

if __name__ == '__main__':
        
    import numpy as np

    for i in range(20):
        print("Run in the {}th iteration".format(i+1))
        mat = np.random.randint(0, 255, (480, 640))

        brute_force(mat)
        better_way(mat)
    # start = time.time()
    # indices = np.argwhere(mat >= 240)
    # end = time.time()
    # print(end - start)
    # print(indices.sum(axis = 0) / indices.shape[0])