import cv2 as cv
import numpy as np

IMG_SIZE = (1280, 720)
W, H = IMG_SIZE
wTop =  957
hTop =  0
wBot =  262 
hBot =  720

params_processing = dict()
params_processing['dst_points'] = np.array([[wBot, hBot], [W - wBot, hBot], [wTop, hTop], [W - wTop, hTop]], dtype = np.float32)
params_processing['src_points'] = np.array([[10, 690], [1280, 690], [800   , 317], [380, 317]], dtype=np.float32)
params_processing['gaussian_kernel_size'] = 5
params_processing['sobel_kernel_size'] = 3
params_processing['lower_white'] = np.array([0, 150, 10])
params_processing['upper_white'] = np.array([250, 250, 255])
# class ImageProcessing:
#     def __init__(self):

#         self.white_low = np.array([0, 0, 180], dtype=np.uint8)
#         self.white_high = np.array([150, 6, 255], dtype=np.uint8)

#         self.verticalStructure1 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))
#         self.verticalStructure2 = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))
#         self.horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 1))
#         self.horizontalStructure2 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))

#         # NOTE BEV param
#         self.cutoff = 60
#         pts1 = np.float32([[151, 120 - self.cutoff], [322, 120 - self.cutoff],
#                         [0, 300 - self.cutoff], [480, 300 - self.cutoff]])
#         pts2 = np.float32([[98, 120 - self.cutoff], [258, 120 - self.cutoff],
#                         [98, 360 - self.cutoff], [258, 360 - self.cutoff]])
#         self.M = cv2.getPerspectiveTransform(pts1, pts2)


#     def colorFilterLine(self, img):
#         h, w= img.shape[0], img.shape[1]
#         white_mask = np.zeros(((h,w)), np.uint8)

#         hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
#         white_mask= cv2.inRange(hsv, self.white_low, self.white_high)

#         ver_img = cv2.dilate(white_mask, self.verticalStructure1)
#         hor_img = cv2.dilate(ver_img,self.horizontalStructure2)

#         black_screen = np.zeros((h,w), np.uint8)
#         contours, hie = cv2.findContours(hor_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         for cnt in contours:
#             area = cv2.contourArea(cnt)
#             if area >= 900:
#                 cv2.drawContours(black_screen, [cnt], -1, 255, -1)

#         return black_screen
    
#     def region_of_interest(self, frame):
#         roi = np.array([[(10, 690), (1280, 690), (800, 317), (380, 317)]], dtype = np.int32)
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Apply Gaussian blur to reduce noise
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)

#         # Apply Canny edge detection to detect edges
#         edges = cv2.Canny(blur, 50, 150)

#         # Apply region of interest mask to extract the lane markings
#         masked_edges = np.zeros_like(edges) 
#         cv2.fillPoly(masked_edges, roi, 255)
#         masked_edges = cv2.bitwise_and(edges, masked_edges)

#         return masked_edges


class Preprocessor:

    """ This class is used to preprocessing the scene which is input steamed from camera.
    Args: None
    Attributes: 

        gaussian_kernel_size: size of kernel in gaussian blur, we use this blurring to 
                                smooth the binary threshold version of the scene.

        sobel_kernel_size: size of kernel in sobel transformation
        lower_white: lower bound of white color when performing color-space transform.
        upper_white: upper bound of white color when performing color-space transform.

    Methods:

        grey_scale:
            args: image to be converted into gray.
            return: gray image
            example: self.grey_scale(image)
        
        gaussian_blur:
            args: image to be blurred.
            return: blurred image
            example: self.gaussian_blur(image)
        
        threshold:
            *   This is called binary threshold
            args: image to be converted into binary.
            return: binary image
            example: self.threshold(image)
        
        hls_threshold:
            *   This method takes threshold in HLS color-space then convert into binary threshold image.
            args: image.
            return: HLS image
            example: self.hls_threshold(image)

        warpImg:
            *   This method converts original image into another perspective view. Concretely, we
                would like to have a bird-eye view of the scene taken from the camera rather than the
                in front view.
            *   Note: The source and destination points are environment-specific. Therefore, if we
                        test in another environment, we need to fine-tune based on the visualization

            args: image to be warped.
            return: 
                birdEyeView: dictionary
                             * Key : Value
                             birdeye:   BEV image 
                             left:      left-scene of BEV
                             right:     right-scene of BEV
                             src:       source point of the transformation
                             dst:       destination point of the transformation

                transform_view : matrix of transformation from src to dst (npndarray)
                inverse_transform_view : matrix of transformation from dst to src (npndarray)
    """
    def __init__(self):
        self.gaussian_kernel_size = params_processing['gaussian_kernel_size']# Size of gaussian kernel
        self.sobel_kernel_size = params_processing['sobel_kernel_size'] 
        self.lower_white = params_processing['lower_white']
        self.upper_white = params_processing['upper_white']
        #self.tracker = Trackbars()
        print("Init processing images")
        

    def grey_scale(self, img):
        
        return cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    
    def gaussian_blur(self, img):
        return cv.GaussianBlur(img, (3,3), 0)

    def threshold(self, img):
        _, img = cv.threshold(img, 200, 250, cv.THRESH_BINARY) 
        return img

    def hls_threshold(self, img, lower = 200, upper = 255):
        hls = cv.cvtColor(img, cv.COLOR_RGB2HLS)

        # This is for testing
        s_binary = hls      
        # s_channel = hls[:,:,1]
        
        # # Creating image masked in S channel
        # s_binary = np.zeros_like(s_channel)
        # s_binary[(s_channel >= lower) & (s_channel <= upper)] = 1
        return s_binary

    def warpImg(self, img):

        
        birdeyeView = dict()
        H, W, C = img.shape

        src = params_processing['src_points']
        # src = self.tracker.getValPoints()
        dst = params_processing['dst_points']
        transform_view = cv.getPerspectiveTransform(src, dst)
        inverse_transform_view = cv.getPerspectiveTransform(dst, src)
        
        birdeye = cv.warpPerspective(img, transform_view, (W, H))           # Eye-bird view
        birdeyeLeft = birdeye[:, :W//2]
        birdeyeRight = birdeye[:, W//2: ]

        birdeyeView['birdeye'] = birdeye
        # birdeyeView['birdeye'] = img 
        birdeyeView['left'] = birdeyeLeft
        birdeyeView['right'] = birdeyeRight
        birdeyeView['src'] = src
        birdeyeView['dst'] = dst
        
        return birdeyeView, transform_view, inverse_transform_view

    def process(self, img):
        """Preproces pipeline

        args: raw image of scene
        pipeline: warpImage -> gray -> thresh -> blur -> canny
        return: dictionary : {
            key: value
            "birdeye_img" :         binary threshold bird eye view
            "transform_view":       transform matrix to new perspective
            "inverse_transform":    inverse to original perspective
            "thresh":               binary threshold of original perspective
            "canny":                
        }
        """        
        results = dict()
        H, W, C = img.shape
        assert (H == IMG_SIZE[1]), "Size of scene is not compatible. Expected {} but got {}".format(IMG_SIZE[1], H)

        birdeyeView, transformed_view, invMatrixTransform = self.warpImg(img)
        mask = cv.inRange(birdeyeView['birdeye'], self.lower_white, self.upper_white)
        hls_bin = cv.bitwise_and(birdeyeView['birdeye'], birdeyeView['birdeye'], mask=mask)
        gray = self.grey_scale(hls_bin)
        thresh = self.threshold(gray) 
        blur = cv.GaussianBlur(thresh, (3, 3), 0)
        canny = cv.Canny(blur, 40, 60)


        results['birdeye_img'] = birdeyeView['birdeye']
        results["birdeye"] = birdeyeView 
        results['inverse_transform'] = invMatrixTransform
        results['transformed_view'] = transformed_view
        results['gray'] = gray
        results['thresh'] = thresh
        results['blur'] = blur
        results['canny'] = canny

        return results 


