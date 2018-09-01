#!/usr/bin/env python

# Imports
import cv2
import rospy
from cv_bridge import CvBridge
import StillLaneDetection as laneDetection
import numpy as np
from sensor_msgs.msg import Image

# Variables
circeRadius = 5
bridge = CvBridge()
pub = rospy.Publisher('combinedImage', Image, queue_size=10)


# get image
def callback(data):
	image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
	height, width, channels = image.shape
	mask = np.zeros_like(image)

	laneDetection.getLaneLines(image)
	laneLines = laneDetection.houghTransformation.laneLines

	leftValues = []
	rightValues = []
	middle = int(width/2)
	for line in laneLines:
			for x1, y1, x2, y2 in line:
			    if (x1 <= middle) or (x2 <= middle):
			        leftValues.append(x1)
			        leftValues.append(x2)
			    else:
			        rightValues.append(x1)
			        rightValues.append(x2)
			    cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 2)

	cv2.circle(mask, (int(width/2), int(height/2)), circeRadius, [0, 255, 0], 5)

	# Get Smallest and Biggest Values of List
	minLeft = min(leftValues)
	maxLeft = max(leftValues)
	minRight = min(rightValues)
	maxRight = max(rightValues)

	# Draw circles for those values
	yDist = height-200
	cv2.circle(mask, (minLeft, yDist), circeRadius, [255, 0, 0], 2)
	cv2.circle(mask, (maxLeft, yDist), circeRadius, [255, 0, 0], 2)
	cv2.circle(mask, (minRight, yDist), circeRadius, [255, 0, 0], 2)
	cv2.circle(mask, (maxRight, yDist), circeRadius, [255, 0, 0], 2)

	combinedImage = cv2.addWeighted(image, 0.5, mask, 0.5, 0)
	pub.publish(bridge.cv2_to_imgmsg(combinedImage, encoding="rgb8"))

	center = width/2
	l_dist = center - maxLeft
	r_dist = minRight - center

	"""
	if l_dist > r_dist:
		print("Left")
	elif r_dist > l_dist:
		print("Right")
	else:
		print("Straight")"""



def simpleLaneKeeping():
	# Subscriber
	rospy.init_node('speedcontrol', anonymous=False)
	sub = rospy.Subscriber('usb_cam/image_raw', Image, callback)

	rospy.spin()

if __name__ == '__main__':
	simpleLaneKeeping() 
