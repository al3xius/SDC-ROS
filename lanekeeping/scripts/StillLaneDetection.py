#!/usr/bin/env python

'''
Get Lane Lines from Still Image
David Zechmeister
05.04.2018
'''

# Imports
import os
import cv2
import rospy
import numpy as np
from PIL import Image


# Get settings
hM = rospy.get_param("/lane/heightmultiplier")
cM = rospy.get_param("/lane/cropmultiplier")
bM = rospy.get_param("/lane/bottommultiplier")
low_threshold = rospy.get_param("/lane/low_threshold")
high_threshold = rospy.get_param("/lane/high_threshold")


laneLines = 0

"""
def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    """`img` should be the output of a Canny transform.
    Returns an image with hough lines drawn.
    
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
    
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    `initial_img` should be the image before any processing.
    The result image is computed as follows:
    initial_img * a + img * b + c
    NOTE: initial_img and img must be the same shape!
    
    return cv2.addWeighted(initial_img, a, img, b, c)


def process_frame(image):

    gray_image = grayscale(image)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    #hsv = [hue, saturation, value]
    #more accurate range for yellow since it is not strictly black, white, r, g, or b

    lower_yellow = np.array([20, 100, 100], dtype = "uint8")
    upper_yellow = np.array([30, 255, 255], dtype="uint8")

    mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(gray_image, 200, 255)
    mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)

    kernel_size = 5
    gauss_gray = gaussian_blur(mask_yw_image,kernel_size)

    #same as quiz values
    low_threshold = 50
    high_threshold = 150
    canny_edges = canny(gauss_gray,low_threshold,high_threshold)

    imshape = image.shape
    lower_left = [imshape[1]/9,imshape[0]]
    lower_right = [imshape[1]-imshape[1]/9,imshape[0]]
    top_left = [imshape[1]/2-imshape[1]/8,imshape[0]/2+imshape[0]/10]
    top_right = [imshape[1]/2+imshape[1]/8,imshape[0]/2+imshape[0]/10]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(canny_edges, vertices)

    #rho and theta are the distance and angular resolution of the grid in Hough space
    #same values as quiz
    rho = 2
    theta = np.pi/180
    #threshold is minimum number of intersections in a grid for candidate line to go to output
    threshold = 20
    min_line_len = 50
    max_line_gap = 200

    line_image = hough_lines(roi_image, rho, theta, threshold, min_line_len, max_line_gap)
    result = weighted_img(line_image, image, a=0.8, b=1., c=0.)
    return result
"""


def calcLaneLines(roiImage):
	rho = 2
	theta = np.pi/180
	threshold = 15
	minLineLength = 25
	maxLineGap = 100
	laneLines = []
	laneLines = cv2.HoughLinesP(roiImage, rho, theta, threshold, np.array(
		[]), minLineLength, maxLineGap)

	return laneLines


# Perform Hough Transformation on ROI-Image
# Draw Lines on Blank Image
def houghTransformation(roiImage):
	laneLines = calcLaneLines(roiImage)
	houghTransformation.laneLines = laneLines
	laneLineImage = np.zeros(
		(roiImage.shape[0], roiImage.shape[1], 3), dtype=np.uint8)

	try:
		for line in laneLines:
			for x1, y1, x2, y2 in line:
			    cv2.line(laneLineImage, (x1, y1), (x2, y2), [0, 0, 255], 2)
	except:
		result = "no Lines"

	return laneLineImage


# Delete all unnecessary objects from canny image


def regionOfInterest(img):
	# get params
	hM = rospy.get_param("/lane/heightmultiplier")
	cM = rospy.get_param("/lane/cropmultiplier")
	bM = rospy.get_param("/lane/bottommultiplier")
	imshape = img.shape
	lower_left = [imshape[1]/bM, imshape[0]]
	lower_right = [imshape[1]-imshape[1]/bM, imshape[0]]
	top_left = [imshape[1]/cM-imshape[1]/hM, imshape[0]/cM+imshape[0]/hM]
	top_right = [imshape[1]/cM+imshape[1]/hM, imshape[0]/cM+imshape[0]/hM]
	vertices = [np.array([lower_left, top_left, top_right,
                          lower_right], dtype=np.int32)]
    
    # Black Image in same size as original
	mask = np.zeros_like(img)

    #filling pixels inside the polygon defined by "vertices" with the fill color
	cv2.fillPoly(mask, vertices, 255)

    #returning the image only where mask pixels are nonzero
	masked_image = cv2.bitwise_and(img, mask)
	return masked_image


# Get Lanelines from Image
def getLaneLines(img):
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    maskWhite = cv2.inRange(grayImage, 200, 255)
    gaussBlur = cv2.GaussianBlur(maskWhite, (5, 5), 0)
    lowEnd = rospy.get_param("/lane/lowend")
    highEnd = rospy.get_param("/lane/highend")
    cannyConverted = cv2.Canny(gaussBlur, lowEnd, highEnd)
    roiImage = regionOfInterest(cannyConverted)
    laneLineImage = houghTransformation(roiImage)

    return laneLineImage

