import cv2 as cv
import numpy as np
from .utils import Trackbars
import matplotlib.pyplot as plt

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
        self.gaussian_kernel_size = 5                   # Size of gaussian kernel
        self.sobel_kernel_size = 3
        self.lower_white = np.array([0, 160, 10])
        self.upper_white = np.array([255, 255, 255])
        self.tracker = Trackbars()                      # Just for debugging and fine-tuning --> don't care
        print("Init processing images")

    def grey_scale(self, img):

        return cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    def gaussian_blur(self, img):
        return cv.GaussianBlur(img, (3,3), 0)

    def threshold(self, img):
        ret, img = cv.threshold(img, 220, 225, cv.THRESH_BINARY) 
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
    
    # def sobel_threshold(self, img, orient = 'x', lower=20, upper = 100 ):
    #     gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    #     if orient == 'x':
    #         sobelx = cv.Sobel(gray, cv.CV_64F, 1, 0, ksize=self.sobel_kernel_size) # Take the derivative in x
    #         abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    #         scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    #     else:
    #         sobely = cv.Sobel(gray, cv.CV_64F, 0, 1, ksize=self.sobel_kernel_size) # Take the derivative in x
    #         abs_sobely = np.absolute(sobely) # Absolute x derivative to accentuate lines away from horizontal
    #         scaled_sobel = np.uint8(255*abs_sobely/np.max(abs_sobely))
        
    #     # Creathing img masked in x gradient
    #     grad_bin = np.zeros_like(scaled_sobel)
    #     grad_bin[(scaled_sobel >= lower) & (scaled_sobel <= upper)] = 1
        
    #     return grad_bin
    
    # def mag_thresh(self, img, thresh_min=100, thresh_max=255):
    #     # Convert to grayscale
    #     gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    #     # Take both Sobel x and y gradients
    #     sobelx = cv.Sobel(gray, cv.CV_64F, 1, 0, ksize=self.sobel_kernel_size)
    #     sobely = cv.Sobel(gray, cv.CV_64F, 0, 1, ksize=self.sobel_kernel_size)
    #     # Calculate the gradient magnitude
    #     gradmag = np.sqrt(sobelx**2 + sobely**2)
    #     # Rescale to 8 bit
    #     scale_factor = np.max(gradmag)/255 
    #     gradmag = (gradmag/scale_factor).astype(np.uint8) 
    #     # Create a binary image of ones where threshold is met, zeros otherwise
    #     binary_output = np.zeros_like(gradmag)
    #     binary_output[(gradmag >= thresh_min) & (gradmag <= thresh_max)] = 1

    #     # Return the binary image
    #     return binary_output


    # def dir_thresh(self, img, thresh_min=0, thresh_max=np.pi/2):
    #     # Grayscale
    #     gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    #     # Calculate the x and y gradients
    #     sobelx = cv.Sobel(gray, cv.CV_64F, 1, 0, ksize=self.sobel_kernel_size)
    #     sobely = cv.Sobel(gray, cv.CV_64F, 0, 1, ksize=self.sobel_kernel_size)
    #     # Take the absolute value of the gradient direction, 
    #     # apply a threshold, and create a binary image result
    #     absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    #     binary_output =  np.zeros_like(absgraddir)
    #     binary_output[(absgraddir >= thresh_min) & (absgraddir <= thresh_max)] = 1

    #     # Return the binary image
    #     return binary_output


    # def lab_b_channel(self, img, thresh=(190,255)):
    #     lab = cv.cvtColor(img, cv.COLOR_RGB2Lab)
    #     lab_b = lab[:,:,2]
    #     # Don't normalize if there are no yellows in the image
    #     if np.max(lab_b) > 175:
    #         lab_b = lab_b*(255/np.max(lab_b))
    #     #  Apply a threshold
    #     binary_output = np.zeros_like(lab_b)
    #     binary_output[((lab_b > thresh[0]) & (lab_b <= thresh[1]))] = 1
    #     return binary_output

    def warpImg(self, img):

        H = img.shape[0]
        W = img.shape[1]
        wTop = 82 
        hTop = 170 
        wBot = 20 
        hBot = 312 
        birdeyeView = dict()

        src = np.array([[wTop, hTop], [W- wTop, hTop], [wBot, hBot], [W- wBot, hBot]], dtype = np.float32)
        # dst = np.float32([[32,0],[100,0],[100,128],[32,128]])

        # src =   self.tracker.getValPoints(W, H) 
        dst = np.float32([[0,0], [W,0], [0,H], [W,H]])

        transform_view = cv.getPerspectiveTransform(src, dst)
        inverse_transform_view = cv.getPerspectiveTransform(dst, src)
        
        birdeye = cv.warpPerspective(img, transform_view, (W, H), flags=cv.INTER_LINEAR)           # Eye-bird view
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
        
        results = dict()
        H, W, C = img.shape

        # img = cv.resize(img, (640, 360))
        birdeyeView, transformed_view, invMatrixTransform = self.warpImg(img)

        hsl_bin = self.hls_threshold(birdeyeView['birdeye'])
        mask = cv.inRange(birdeyeView['birdeye'], self.lower_white, self.upper_white)
        hls_bin = cv.bitwise_and(birdeyeView['birdeye'], birdeyeView['birdeye'], mask=mask)

        gray = self.grey_scale(hls_bin)
        _, thresh = cv.threshold(gray, 150, 255, cv.THRESH_BINARY)
        blur = cv.GaussianBlur(thresh, (3, 3), 0)
        canny = cv.Canny(blur, 40, 60)

        """
        lab_b_bin = self.lab_b_channel(warpedImg, thresh=(185, 255))

        # Gradient thresholding with sobel x
        x_bin = self.sobel_threshold(warpedImg, orient='x', lower = 20, upper = 100)
        
        # Gradient thresholding with sobel y
        y_bin = self.sobel_threshold(warpedImg, orient='y', lower=50, upper=150)
        
        # Magnitude of gradient thresholding
        mag_bin = self.mag_thresh(warpedImg, thresh_min=0, thresh_max=255)
        
        # Direction of gradient thresholding
        dir_bin = self.dir_thresh(warpedImg, thresh_min=0, thresh_max=np.pi/2)

        combined = np.zeros_like(x_bin)
        combined[(hsl_bin == 1) | (lab_b_bin == 1)] = 1 
        """

        results['birdeye_img'] = birdeyeView['birdeye']
        results["birdeye"] = birdeyeView 
        results['inverse_transform'] = invMatrixTransform
        results['transformed_view'] = transformed_view
        results['hsl_bin'] = hsl_bin
        results['gray'] = gray
        results['thresh'] = thresh
        results['blur'] = blur
        results['canny'] = canny

        return results 

class LaneDetection:

    def __init__(self):

        """

        """
        self.processor = Preprocessor()
        self.scene_height = 360
        self.scene_width = 640
        self.ym_per_pix = 30 / self.scene_height 
        self.xm_per_pix = 3.7 / self.scene_width
            

    def slide_window_search(self, binary_warped):
        try:
            histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis = 0)
            # Find the start of left and right lane lines using histogram info
            out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
            midpoint = int(histogram.shape[0] / 2)
            leftx_base = np.argmax(histogram[:midpoint])
            rightx_base = np.argmax(histogram[midpoint:]) + midpoint
            print("Left base: {}\nRight base: {}\nMid point: {}".format(leftx_base, rightx_base, midpoint))
            
            # A total of 9 windows will be used
            nwindows = 9
            window_height = np.int8(binary_warped.shape[0] / nwindows)
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            leftx_current = leftx_base
            rightx_current = rightx_base
            margin = 100 
            minpix = 50 
            left_lane_inds = []
            right_lane_inds = []

            #### START - Loop to iterate through windows and search for lane lines #####
            for window in range(nwindows):
                win_y_low = binary_warped.shape[0] - (window + 1) * window_height
                win_y_high = binary_warped.shape[0] - window * window_height
                win_xleft_low = leftx_current - margin
                win_xleft_high = leftx_current + margin
                win_xright_low = rightx_current - margin
                win_xright_high = rightx_current + margin
                cv.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
                (0,255,0), 2)
                cv.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
                (0,255,0), 2)
                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
                left_lane_inds.append(good_left_inds)
                right_lane_inds.append(good_right_inds)
                if len(good_left_inds) > minpix:
                    leftx_current = np.int8(np.mean(nonzerox[good_left_inds]))
                if len(good_right_inds) > minpix:
                    rightx_current = np.int8(np.mean(nonzerox[good_right_inds]))
            #### END - Loop to iterate through windows and search for lane lines #######

            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
            
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            
            result = dict()
            result['left_lane_inds'] = leftx_current
            result['right_lane_inds'] = right_lane_inds
            result['out_img'] = out_img
            
            return result
        
        except Exception as e:
            print(e)

    def margin_search(self, binary_warped, left_fit, right_fit):
        try:
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            margin = 50 
            left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
            left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
            left_fit[1]*nonzeroy + left_fit[2] + margin)))

            right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
            right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
            right_fit[1]*nonzeroy + right_fit[2] + margin)))

            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]

            #   Fit second order polynomial
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


            ## VISUALIZATION ###########################################################

            out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
            
            window_img = np.zeros_like(out_img)
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

            left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
            left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                        ploty])))])
            left_line_pts = np.hstack((left_line_window1, left_line_window2))
            right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
            right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
            right_line_pts = np.hstack((right_line_window1, right_line_window2))

            cv.fillPoly(window_img, np.int32(left_line_pts), (0, 255, 0))
            cv.fillPoly(window_img, np.int32(right_line_pts), (0, 255, 0))
            out_img= cv.addWeighted(out_img, 1, window_img, 0.3, 0)

            # # plt.imshow(result)
            # plt.plot(left_fitx,  ploty, color = 'yellow')
            # plt.plot(right_fitx, ploty, color = 'yellow')
            # plt.xlim(0, 1280)
            # plt.ylim(720, 0)

            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
                
            # Draw polyline on image
            right = np.asarray(tuple(zip(right_fitx, ploty)), np.int32)
            left = np.asarray(tuple(zip(left_fitx, ploty)), np.int32)
            cv.polylines(out_img, [right], False, (1,1,0), thickness=5)
            cv.polylines(out_img, [left], False, (1,1,0), thickness=5)
            
            ret = {}
            ret['leftx'] = leftx
            ret['rightx'] = rightx
            ret['left_fitx'] = left_fitx
            ret['right_fitx'] = right_fitx
            ret['ploty'] = ploty
            ret['result'] = out_img
            ret['left_lane_inds'] = left_lane_inds
            ret['right_lane_inds'] = right_lane_inds 

            return ret
        
        except Exception as e:
            print(e)
   
    def measure_lane_curvature(self, ploty, leftx, rightx):

        leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
        rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

        # Choose the maximum y-value, corresponding to the bottom of the image
        y_eval = np.max(ploty)

        # Fit new polynomials to x, y in world space
        left_fit_cr = np.polyfit(ploty*self.ym_per_pix, leftx*self.xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty*self.ym_per_pix, rightx*self.xm_per_pix, 2)

        # Calculate the new radii of curvature
        left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*self.ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*self.ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        # Now our radius of curvature is in meters
        # print(left_curverad, 'm', right_curverad, 'm')

        # Decide if it is a left or a right curve
        if leftx[0] - leftx[-1] > 60:
            curve_direction = 'Left Curve'
        elif leftx[-1] - leftx[0] > 60:
            curve_direction = 'Right Curve'
        else:
            curve_direction = 'Straight'

        return (left_curverad + right_curverad) / 2.0, curve_direction
    
    def draw_lane_lines(self, original_image, warped_image, Minv, draw_info):

        leftx = draw_info['leftx']
        rightx = draw_info['rightx']
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']

        warp_zero = np.zeros_like(warped_image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        mean_x = np.mean((left_fitx, right_fitx), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

        cv.fillPoly(color_warp, np.int32([pts]), (0, 255, 0))
        cv.fillPoly(color_warp, np.int32([pts_mean]), (0, 255, 255))

        newwarp = cv.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv.addWeighted(original_image, 1, newwarp, 0.3, 0)

        return pts_mean, result
    
    def offCenter(self, meanPts, inpFrame):

        # Calculating deviation in meters
        mpts = meanPts[-1][-1][-2].astype(int)
        pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
        deviation = pixelDeviation * self.xm_per_pix
        direction = "left" if deviation < 0 else "right"

        return deviation, direction
    
    def addText(self, img, radius, direction, deviation, devDirection):

        # Add the radius and center position to the image
        font = cv.FONT_HERSHEY_TRIPLEX

        if (direction != 'Straight'):
            text = 'Radius of Curvature: ' + '{:04.0f}'.format(radius) + 'm'
            text1 = 'Curve Direction: ' + (direction)

        else:
            text = 'Radius of Curvature: ' + 'N/A'
            text1 = 'Curve Direction: ' + (direction)

        cv.putText(img, text , (50,100), font, 0.8, (0,100, 200), 2, cv.LINE_AA)
        cv.putText(img, text1, (50,150), font, 0.8, (0,100, 200), 2, cv.LINE_AA)

        # Deviation
        deviation_text = 'Off Center: ' + str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
        cv.putText(img, deviation_text, (50, 200), cv.FONT_HERSHEY_TRIPLEX, 0.8, (0,100, 200), 2, cv.LINE_AA)

        return img