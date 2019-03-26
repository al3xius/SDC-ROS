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


# get image
def callback(data):
    # convert to correct format
    image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    height, width, channels = image.shape
    mask = np.zeros_like(image)
    middle = int(width/2)

    laneDetection.getLaneLines(image)
    laneLines = laneDetection.houghTransformation.laneLines

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
                if (x2 <= middle) and k < -0.5:
                    leftValues.append(x1)
                    leftValues.append(x2)
                    cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 2)
                elif (x2 > middle) and k > 0.5:
                    rightValues.append(x1)
                    rightValues.append(x2)
                    cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 2)
                else:
                    pass
                pass
    except:
        result = "no Lines"

    # draw center circle
    cv2.circle(mask, (int(width/2), int(height-100)),
               circeRadius, [0, 255, 0], 5)

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

    """
	# ROI - for testing only (is adding latency)

	topWidth = rospy.get_param("/lane/topWidth")
	height = rospy.get_param("/lane/height")
	bottomWidth = rospy.get_param("/lane/bottomWidth")

	imshape = image.shape
	lower_left = [imshape[1]/bottomWidth, imshape[0]]
	lower_right = [imshape[1]-imshape[1]/bottomWidth, imshape[0]]
	top_left = [imshape[1]/2-imshape[1]/topWidth, imshape[0]/height]
	top_right = [imshape[1]/2+imshape[1]/topWidth, imshape[0]/height]
	vertices = [np.array([lower_left, top_left, top_right, lower_right], dtype=np.int32)]

	cv2.polylines(mask, vertices, True, (0, 255, 255))  # draw ROI
	"""

    combinedImage = cv2.addWeighted(image, 0.7, mask, 0.5, 0)

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
