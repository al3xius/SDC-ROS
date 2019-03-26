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
topWidth = rospy.get_param("/lane/topWidth")
height = rospy.get_param("/lane/height")
bottomWidth = rospy.get_param("/lane/bottomWidth")
low_threshold = rospy.get_param("/lane/low_threshold")
high_threshold = rospy.get_param("/lane/high_threshold")
lowEnd = rospy.get_param("/lane/low_threshold")
highEnd = rospy.get_param("/lane/high_threshold")
whiteMaskLow = rospy.get_param("/lane/whiteMaskLow")
whiteMaskHigh = rospy.get_param("/lane/whiteMaskHigh")

laneLines = 0


def calcLaneLines(roiImage):
    rho = 1
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
    """
    topWidth = rospy.get_param("/lane/topWidth")
    height = rospy.get_param("/lane/height")
    bottomWidth = rospy.get_param("/lane/bottomWidth")"""

    imshape = img.shape
    lower_left = [imshape[1]/bottomWidth, imshape[0]]
    lower_right = [imshape[1]-imshape[1]/bottomWidth, imshape[0]]
    top_left = [imshape[1]/2-imshape[1]/topWidth, imshape[0]/height]
    top_right = [imshape[1]/2+imshape[1]/topWidth, imshape[0]/height]
    vertices = [np.array([lower_left, top_left, top_right,
                          lower_right], dtype=np.int32)]

    # Black Image in same size as original
    mask = np.zeros_like(img)

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, [255, 255, 0])

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    cv2.polylines(masked_image, vertices, True, (0, 255, 255))

    return masked_image


# Get Lanelines from Image
def getLaneLines(img):
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    maskWhite = cv2.inRange(grayImage, whiteMaskLow, whiteMaskHigh)
    gaussBlur = cv2.GaussianBlur(maskWhite, (5, 5), 0)
    cannyConverted = cv2.Canny(gaussBlur, lowEnd, highEnd)
    roiImage = regionOfInterest(cannyConverted)
    laneLineImage = houghTransformation(roiImage)
    return laneLineImage
