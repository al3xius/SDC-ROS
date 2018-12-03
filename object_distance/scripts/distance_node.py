#!/usr/bin/env python
import rospy 
from math import pi
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3DArray, Detection3D

class DistanceNode:
    def __init__(self):
        rospy.init_node("distance_node")
        rospy.loginfo("Starting DistanceNode.")

        rospy.Subscriber("/scan", LaserScan, self.lidar_calback)
        rospy.Subscriber("/objectDedector/objects", Detection2DArray, self.obj_callback)
        self.distance_pub = rospy.Publisher("/distance/objects", Detection3DArray, queue_size=1)
        
        self.cam_width = float(rospy.get_param("/usb_cam/image_width"))
        cam_maxAngle = float(rospy.get_param("/usb_cam/image_width_angle"))
        self.angleFactor = cam_maxAngle / (self.cam_width / 2)
        print(self.angleFactor)
        
        
        self.lastScan = LaserScan()
        self.lastObj = Detection2DArray()
        
    def lidar_calback(self, msg):
        self.lastScan = msg

    def obj_callback(self, msg):
        self.lastObj = msg
        self.calcDistance()


    def calcDistance(self):
        self.newObjArray = Detection3DArray()
        newObj = Detection3D()
        for obj in self.lastObj.detections:
            newObj.bbox.center.position.x = obj.bbox.center.x
            newObj.bbox.center.position.y = obj.bbox.center.y
            angle = self.calcAngle(obj.bbox.center.x, obj.bbox.size_x)
            distance = self.getDistance(angle)
            newObj.bbox.center.position.z = distance
            newObj.bbox.center.orientation.w = angle
            self.newObjArray.detections.append(newObj)

        self.distance_pub.publish(self.newObjArray)

    def calcAngle(self, x, width):
        
        
        angle = self.angleFactor * abs(x - self.cam_width/2)
        if x - self.cam_width/2 < 0:
            angle *= -1
        return angle

    def getDistance(self, angle):
        """if angle < 0:
            angle = 360 - angle"""
        
        angle = angle*pi/180
        index = int((angle - self.lastScan.angle_min) / float(self.lastScan.angle_increment))
        distance = self.lastScan.ranges[index]
        print(distance)
        return distance

if __name__ == "__main__":
    ros_node = DistanceNode()
    rospy.spin()