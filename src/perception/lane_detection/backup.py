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