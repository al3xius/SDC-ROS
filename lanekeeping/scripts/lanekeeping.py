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



# get image
def callback(data):
	# convert to correct format
	image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

	
	height, width, channels = image.shape
	mask = np.zeros_like(image)

	laneDetection.getLaneLines(image)
	laneLines = laneDetection.houghTransformation.laneLines

	leftValues = []
	rightValues = []
	middle = int(width/2)
	try:
		for line in laneLines:
				for x1, y1, x2, y2 in line:
    					if x2 > x1:
    						k = (y2-y1)/(x2-x1)
					else:
    						k = (y1-y2)/(x1-x2)

					if (x2 <= middle) and k < -1.8:
    						leftValues.append(x1)
					    	leftValues.append(x2)
						cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 2)
					elif (x1 > middle) and k > 1.8:
    						rightValues.append(x1)
					    	rightValues.append(x2)
						cv2.line(mask, (x1, y1), (x2, y2), [0, 0, 255], 2)
					else:
    						pass
					pass
	except:
		result = "no Lines"

	# draw center circle
	cv2.circle(mask, (int(width/2), int(height-200)), circeRadius, [0, 255, 0], 5)

	try:
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
		
		center = width/2
		l_dist = center - maxLeft
		r_dist = minRight - center

		laneCenter = maxLeft - (maxLeft - minRight)/2 

		offset = center - laneCenter

		# draw lane centre
		cv2.circle(mask, (laneCenter, yDist), circeRadius, [255, 255, 0], 2)

		result = offset
	except:
		result = 0

	topWidth = rospy.get_param("/lane/topWidth")
	height = rospy.get_param("/lane/height")
	bottomWidth = rospy.get_param("/lane/bottomWidth")

	imshape = image.shape
	lower_left = [imshape[1]/bottomWidth, imshape[0]]
	lower_right = [imshape[1]-imshape[1]/bottomWidth, imshape[0]]
	top_left = [imshape[1]/2-imshape[1]/topWidth, imshape[0]/height]
	top_right = [imshape[1]/2+imshape[1]/topWidth, imshape[0]/height]
	vertices = [np.array([lower_left, top_left, top_right,
							lower_right], dtype=np.int32)]

	cv2.polylines(mask, vertices, True, (0,255,255))
		

	combinedImage = cv2.addWeighted(image, 0.7, mask, 0.5, 0)
	
	pub.publish(bridge.cv2_to_imgmsg(combinedImage, encoding="rgb8"))
	pub2.publish(result)


def returnMask():	
	return bridge.cv2_to_imgmsg(combinedImage, encoding="rgb8")



def simpleLaneKeeping():
	# Subscriber
	sub = rospy.Subscriber('/usb_cam/image_raw', Image, callback)
	service = rospy.Service("/lane/", laneMask, returnMask)

	rospy.spin()


if __name__ == '__main__':
    # create statemachine node
	rospy.init_node('laneKeeping', anonymous=False)
	try:
		simpleLaneKeeping() 
	except rospy.ROSInterruptException:  
		pass