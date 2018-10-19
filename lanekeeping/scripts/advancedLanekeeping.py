#!/usr/bin/env python



import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob

# ROS imports
import rospy
from sensor_msgs.msg import Image


class LaneDedector():

    def __init__(self):
        # calibrate camera
        self.imgpoints, self.objpoints = self.collect_callibration_points() # calculate callibration points

        # subscriber
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.imgCallback)
        # publisher
        self.pub = rospy.Publisher('/lane/resultImage', Image, queue_size=1)

        rospy.spin()



    # Calibrate Camera
    def cal_undistort(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None,None)
        undist = cv2.undistort(img, mtx, dist, None, mtx)
        return undist, mtx, dist

    def collect_callibration_points(self):
        objpoints = []
        imgpoints = []

        images = glob.glob('./camera_cal/calibration*.jpg')
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1, 2)

        for fname in images:
            img = mpimg.imread(fname)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

            if ret == True:
                imgpoints.append(corners)
                objpoints.append(objp)

        return imgpoints, objpoints



    # Gradient Thresholds
    def abs_sobel_thresh(self, image, orient='x', sobel_kernel=3, thresh=(0, 255)):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        isX = True if orient == 'x' else False
        sobel = cv2.Sobel(gray, cv2.CV_64F, isX, not isX)
        abs_sobel = np.absolute(sobel)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        grad_binary = np.zeros_like(scaled_sobel)
        grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

        return grad_binary

    def mag_thresh(self, image, sobel_kernel=3, mag_thresh=(0, 255)):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        abs_sobel = np.sqrt(sobelx**2 + sobely**2)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
        mag_binary = np.zeros_like(scaled_sobel)
        mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1

        return mag_binary

    def dir_threshold(self, image, sobel_kernel=3, thresh=(0, np.pi/2)):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
        abs_sobelx = np.absolute(sobelx)
        abs_sobely = np.absolute(sobely)
        grad_dir = np.arctan2(abs_sobely, abs_sobelx)
        dir_binary = np.zeros_like(grad_dir)
        dir_binary[(grad_dir >= thresh[0]) & (grad_dir <= thresh[1])] = 1

        return dir_binary

    def apply_thresholds(self, image, ksize=3):
        gradx = self.abs_sobel_thresh(image, orient='x', sobel_kernel=ksize, thresh=(20, 100))
        grady = self.abs_sobel_thresh(image, orient='y', sobel_kernel=ksize, thresh=(20, 100))
        mag_binary = self.mag_thresh(image, sobel_kernel=ksize, mag_thresh=(30, 100))
        dir_binary = self.dir_threshold(image, sobel_kernel=ksize, thresh=(0.7, 1.3))

        combined = np.zeros_like(dir_binary)
        combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

        return combined

    def apply_color_threshold(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        s_channel = hls[:,:,2]
        s_thresh_min = 170
        s_thresh_max = 255
        s_binary = np.zeros_like(s_channel)
        s_binary[(s_channel >= s_thresh_min) & (s_channel <= s_thresh_max)] = 1

        return s_binary

    def combine_threshold(self, s_binary, combined):
        combined_binary = np.zeros_like(combined)
        combined_binary[(s_binary == 1) | (combined == 1)] = 1

        return combined_binary


    def warp(self, img):
        img_size = (img.shape[1], img.shape[0])

        src = np.float32(
            [[685, 450],
            [1090, 710],
            [220, 710],
            [595, 450]])

        dst = np.float32(
            [[900, 0],
            [900, 710],
            [250, 710],
            [250, 0]])

        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)

        binary_warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)

        return binary_warped, Minv

    def get_histogram(self, binary_warped):
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)

        return histogram

    def slide_window(self, binary_warped, histogram):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        midpoint = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = np.int(binary_warped.shape[0]/nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            (0,255,0), 2)
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            (0,255,0), 2)
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        plt.imshow(out_img)
        plt.plot(left_fitx, ploty, color='yellow')
        plt.plot(right_fitx, ploty, color='yellow')
        plt.xlim(0, 1280)
        plt.ylim(720, 0)

        return ploty, left_fit, right_fit

    def skip_sliding_window(self, binary_warped, left_fit, right_fit):
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
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
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


        ################################
        ## Visualization
        ################################

        out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
        window_img = np.zeros_like(out_img)
        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

        left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                    ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
        right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin,
                                    ploty])))])
        right_line_pts = np.hstack((right_line_window1, right_line_window2))

        cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

        plt.imshow(result)
        plt.plot(left_fitx, ploty, color='yellow')
        plt.plot(right_fitx, ploty, color='yellow')
        plt.xlim(0, 1280)
        plt.ylim(720, 0)

        ret = {}
        ret['leftx'] = leftx
        ret['rightx'] = rightx
        ret['left_fitx'] = left_fitx
        ret['right_fitx'] = right_fitx
        ret['ploty'] = ploty

        return ret

    def measure_curvature(self, ploty, lines_info):
        ym_per_pix = 30/720
        xm_per_pix = 3.7/700

        leftx = lines_info['left_fitx']
        rightx = lines_info['right_fitx']

        leftx = leftx[::-1]  
        rightx = rightx[::-1]  

        y_eval = np.max(ploty)
        left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        print(left_curverad, 'm', right_curverad, 'm')

        return left_curverad, right_curverad

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

        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

        return result

    def imgCallback(self, image):
        # undistort image
        self.undistorted, self.mtx, self.dist_coefficients = self.cal_undistort(image)

        # gradient thresholds
        self.combined = self.apply_thresholds(self.undistorted)

        # color thresholds
        self.s_binary = self.apply_color_threshold(self.undistorted)

        # combine color and gradient
        self.combined_binary = self.combine_threshold(self.s_binary, self.combined)

        # find lines
        self.binary_warped, self.Minv = self.warp(self.combined_binary)
        self.histogram = self.get_histogram(self.binary_warped)

        #slide window
        self.ploty, self.left_fit, self.right_fit = self.slide_window(self.binary_warped, self.histogram)

        # skipping slinding window
        self.draw_info = self.skip_sliding_window(self.binary_warped, self.left_fit, self.right_fit)

        # measure curvature
        self.left_curverad, self.right_curverad = self.measure_curvature(self.ploty, self.draw_info)

        # TODO: check for errors


        # draw image
        self.result = self.draw_lane_lines(self.undistorted, self.binary_warped, self.Minv, self.draw_info)

        self.pub.publish(self.result)




if __name__ == '__main__':
    # create lanekeeping node
	rospy.init_node('laneKeeping', anonymous=False)
	try:
		laneDedector = LaneDedector()
	except rospy.ROSInterruptException:  
		pass