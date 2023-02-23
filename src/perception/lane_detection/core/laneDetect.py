import cv2 as cv
from .preprocess import  Preprocessor
from .const import *

class LaneDetection:

    def __init__(self):

        """LaneDetection class containts several methods for lane detection 
        with slideing window algorithm and hough transform

        Args: None
        Attrs: 
            processor: instance of Preprocessor class. It preprocesses scenes before performing
                        lane detection algorithms
            ym_per_pix: cm per pixel respected to vertical axis
            xm_per_pix: cm per pixel respected to horizontal axis

        Methods:
            slide_window_search(*args):
                args: binary threshold of BEV sccene
                return:
                example:
                    lanedetector = LaneDetection()
                    slide_window_search_res = lanedetector(binary_warp)

            margin_search(*args):
                args: 
                return:
                example:

            measure_lane_curvature(*args)):
                args:
                return:
                example:

            draw_lane_lines(*args):
                args:
                return:
                example:

            offCenter(*args):
                args:
                return:
                example:

            addText(*args):
                args:
                return:
                example:
                
        """
        self.processor  = Preprocessor()
        self.ym_per_pix =   params_lane_detection['ym_per_pix']  
        self.xm_per_pix =   params_lane_detection['xm_per_pix']
            

    def slide_window_search(self, binary_warped):
        try:

            H, W = binary_warped.shape
            histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis = 0)
           
           # Find the start of left and right lane lines using histogram info
            out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
            midpoint = int(histogram.shape[0] / 2)
            
            # A total of 9 windows will be used
            # edge margin is 50 pixels
            nwindows = 9
            window_height = np.int8( H / nwindows  )
            margin = 50
            minpix = 5 
            
            nonzero = binary_warped.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])


            left_lane_inds = []
            right_lane_inds = []
            
            leftx_base = np.argmax(histogram[:midpoint])
            rightx_base = np.argmax(histogram[midpoint:]) + midpoint
            leftx_current = leftx_base
            rightx_current = rightx_base
            
            #print("Left base: {}\nRight base: {}\nMid point: {}".format(leftx_base, rightx_base, midpoint))

            #### START - Loop to iterate through windows and search for lane lines #####
            for window in range(nwindows):
                win_y_low   = binary_warped.shape[0] - (window + 1) * window_height
                win_y_high  = binary_warped.shape[0] - window * window_height
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

            #   Concat the array of indices
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
            
            #   Extract left and right line pixel positions
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]

            #   Fit a second order polynomial to each 
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)

            #   Generate x and y values for plotting
            ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

            #   Visualization
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

            #   Draw line on image
            right = np.asarray(tuple(zip(right_fitx, ploty)), np.int32) 
            left = np.asarray(tuple(zip(left_fitx, ploty)), np.int32)
            cv.polylines(out_img, [right], False, (1,1,0), thickness=4)
            cv.polylines(out_img, [left], False, (1,1,0), thickness=4)

            result = dict()
            result['left_lane_inds'] = left_lane_inds 
            result['right_lane_inds'] = right_lane_inds 
            result['out_img'] = out_img
            result['right'] = right
            result['left'] = left
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
            _left_fit = np.polyfit(lefty, leftx, 2)
            _right_fit = np.polyfit(righty, rightx, 2)

            #print(_left_fit, _right_fit)

            ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
            left_fitx = _left_fit[0]*ploty**2 + _left_fit[1]*ploty + _left_fit[2]
            right_fitx = _right_fit[0]*ploty**2 + _right_fit[1]*ploty + _right_fit[2]


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
            
            ret = dict() 

            ret['out_img'] = out_img
            ret['left_lane_inds'] = left_lane_inds
            ret['right_lane_inds'] = right_lane_inds 
            ret['right'] = right
            ret['left'] = left
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


