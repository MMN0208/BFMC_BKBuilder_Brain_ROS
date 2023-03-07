import cv2 as cv
import numpy as np
#import matplotlib.pyplot as plt
from .const import *
from .utils import Trackbars
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
        # self.tracker = Trackbars()
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
        # dst = self.tracker.getValPoints()
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


