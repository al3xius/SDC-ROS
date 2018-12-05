#!/usr/bin/env python
import rospy 
from math import radians
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
        self.flip_image = bool(rospy.get_param("/usb_cam/flip_image", default="True"))
        
        cam_fov = float(rospy.get_param("/usb_cam/fov"))
        #self.focalLenght = (0.5 * cam_width) / math.tan(0.5 * cam_fov)

        self.angleFactor = cam_fov/self.cam_width
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
            angle, angle_min, angle_max = self.calcAngle(obj.bbox.center.x, obj.bbox.size_x)
            newObj.bbox.center.orientation.w = angle
            distance = self.getDistance(angle, angle_min, angle_max)
            newObj.bbox.center.position.z = distance
            self.newObjArray.detections.append(newObj)

        self.distance_pub.publish(self.newObjArray)

    def calcAngle(self, x, width):
        if self.flip_image:
            flip = -1
        else:
            flip = 1

        angle_min = self.angleFactor * (x - self.cam_width/2 - width/2) * flip
        angle_max = self.angleFactor * (x - self.cam_width/2 + width/2) * flip
        angle = self.angleFactor * (x - self.cam_width/2) * flip
        return angle, angle_min, angle_max

    def getDistance(self, angle, angle_min, angle_max):
        if angle < 0:
            angle = 360 - abs(angle)

        if angle_min < 0:
            angle_min = 360 - abs(angle_min)

        if angle_max < 0:
            angle_max = 360 - abs(angle_max)
        
        angle = radians(angle)
        angle_min = radians(angle_min)
        angle_max = radians(angle_max)
        try:
            index_min = int(angle_min / float(self.lastScan.angle_increment))
            index_max = int(angle_max / float(self.lastScan.angle_increment))
            distance = self.lastScan.ranges[int(angle / float(self.lastScan.angle_increment))]
            return distance
        except ZeroDivisionError as e:
            rospy.logerr("Can't get distance value! Error: {}".format(e))


if __name__ == "__main__":
    ros_node = DistanceNode()
    rospy.spin()