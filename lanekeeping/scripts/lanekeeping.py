#!/usr/bin/env python

# Imports
import cv2
import rospy
from cv_bridge import CvBridge
import StillLaneDetection as laneDetection
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from sdc_msgs.srv import laneMask

# Variables
circeRadius = 5
bridge = CvBridge()
pub = rospy.Publisher('/lane/combinedImage', Image, queue_size=1)
pub2 = rospy.Publisher('/lane/result', Int32, queue_size=1)

topWidth = rospy.get_param("/lane/topWidth")
height = rospy.get_param("/lane/height")
bottomWidth = rospy.get_param("/lane/bottomWidth")


def makeCoordinates(img, lineParmeters):
    print(lineParmeters)
    if lineParmeters is not None:
        k = lineParmeters[0]
        d = lineParmeters[1]
        y1 = img.shape[0]
        y2 = int(y1 * (3/4))
        x1 = int((y1 - d)/k)
        x2 = int((y2 - d)/k)
        return np.array([x1, y1, x2, y2])


def averageSlopeIntercept(img, lines):
    leftFit = []
    rightFit = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            k = parameters[0]
            d = parameters[1]
            if k < 0:
                leftFit.append((k, d))
            else:
                rightFit.append((k, d))
        leftFitAvg = np.average(leftFit, axis=0)
        rightFitAvg = np.average(rightFit, axis=0)
        leftLane = makeCoordinates(img, leftFitAvg)
        rightLane = makeCoordinates(img, rightFitAvg)
        return np.array([leftLane, rightLane])
    else:
        return None


def printLines(img, lines):
    lineImg = np.zeros_like(img)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(lineImg, (x1, y1), (x2, y2), (255, 0, 0), 4)

    return lineImg


# get image
def callback(data):
    # convert to correct format
    image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    height, width, channels = image.shape
    mask = np.zeros_like(image)
    middle = int(width/2)

    laneDetection.getLaneLines(image)
    laneLines = laneDetection.houghTransformation.laneLines
    #avgLines = averageSlopeIntercept(mask, laneLines)
    #mask = printLines(mask, avgLines)

    leftValues = []
    rightValues = []
    try:
        for line in laneLines:
            for x1, y1, x2, y2 in line:
                if x2 > x1:
                    k = (y2-y1)/(x2-x1)
                else:
                    k = (y1-y2)/(x1-x2)

                # add lanes to left and make sure k is bigger than 1.8
                if (x2 <= middle) and k < -0.9:
                    leftValues.append(x1)
                    leftValues.append(x2)
                    cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 4)
                elif (x2 > middle) and k > 0.9:
                    rightValues.append(x1)
                    rightValues.append(x2)
                    cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 4)
                else:
                    pass
                pass
    except:
        result = "no Lines"

    # Get Smallest and Biggest Values of List
    try:
        minLeft = min(leftValues)
        maxLeft = max(leftValues)
    except ValueError:
        minLeft = 0
        maxLeft = 10

    try:
        minRight = min(rightValues)
        maxRight = max(rightValues)
    except ValueError:
        minRight = width - 10
        maxRight = width

    center = width/2

    # draw center circle
    cv2.circle(mask, (int(width/2), int(height-100)),
               circeRadius, [0, 255, 0], 5)
    # draw circles for those values
    yDist = height-100

    left = minLeft + (maxLeft - minLeft)/2
    right = minRight + (maxRight - minRight)/2
    l_dist = center - maxLeft
    r_dist = minRight - center
    cv2.circle(mask, (left, yDist), circeRadius, [255, 0, 255], 2)
    cv2.circle(mask, (right, yDist), circeRadius, [255, 0, 255], 2)

    # calculate offset
    laneCenter = left + (right - left)/2
    offset = center - laneCenter
    result = offset
    cv2.circle(mask, (laneCenter, yDist), circeRadius, [255, 255, 0], 2)

    # ROI - for testing only (is adding latency)
    """
    topWidth = rospy.get_param("/lane/topWidth")
    height = rospy.get_param("/lane/height")
    bottomWidth = rospy.get_param("/lane/bottomWidth")

    imshape = image.shape
    lower_left = [imshape[1]/bottomWidth, imshape[0]]
    lower_right = [imshape[1]-imshape[1]/bottomWidth, imshape[0]]
    top_left = [imshape[1]/2-imshape[1]/topWidth, imshape[0]/height]
    top_right = [imshape[1]/2+imshape[1]/topWidth, imshape[0]/height]
    vertices = [np.array(
        [lower_left, top_left, top_right, lower_right], dtype=np.int32)]

    cv2.polylines(mask, vertices, True, (0, 255, 255))  # draw ROI
    """
    combinedImage = cv2.addWeighted(image, 0.8, mask, 1, 1)

    # publish mask
    pub.publish(bridge.cv2_to_imgmsg(combinedImage, encoding="rgb8"))
    # publish offset
    pub2.publish(result)


def returnMask():
    return bridge.cv2_to_imgmsg(combinedImage, encoding="rgb8")


def simpleLaneKeeping():
    # Subscriber
    sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback)
    #service = rospy.Service("/lane/", laneMask, returnMask)

    rospy.spin()


if __name__ == '__main__':
    # create statemachine node
    rospy.init_node('laneKeeping', anonymous=False)
    try:
        simpleLaneKeeping()
    except rospy.ROSInterruptException:
        pass
