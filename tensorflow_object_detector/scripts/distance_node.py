#!/usr/bin/env python
import rospy
from math import radians
from sensor_msgs.msg import LaserScan, Image
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3DArray, Detection3D
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2


class DistanceNode:
    def __init__(self):
        # init Node
        rospy.init_node("distance_node")
        rospy.loginfo("Starting DistanceNode.")

        # Subscriber/Publisher
        rospy.Subscriber("/scan", LaserScan, self.lidar_calback)
        rospy.Subscriber("/objectDedector/objects",
                         Detection2DArray, self.obj_callback)
        self.distance_pub = rospy.Publisher(
            "/objectDedector/3Dobjects", Detection3DArray, queue_size=1)

        # get params
        self.cam_width = float(rospy.get_param("/usb_cam/image_width"))
        self.flip_image = bool(rospy.get_param(
            "/usb_cam/flip_image", default="True"))
        cam_fov = float(rospy.get_param("/usb_cam/fov"))
        self.angleFactor = cam_fov/self.cam_width

        # init messages
        self.lastScan = LaserScan()
        self.lastScan.angle_increment = 1
        self.lastObj = Detection2DArray()
        self.lastPub = Detection3DArray()

        # publish image with distances
        if True:
            rospy.Subscriber("/objectDedector/overlayImage",
                             Image, self.img_callback)
            self.img_pub = rospy.Publisher(
                "/objectDedector/overlayImage3D", Image, queue_size=1)
            self.bridge = CvBridge()
            self.font = cv2.FONT_HERSHEY_SIMPLEX

    # Callbacks

    def lidar_calback(self, msg):
        self.lastScan = msg

    def obj_callback(self, msg):
        self.lastObj = msg
        self.calcDistance()

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mask = np.zeros_like(image)
        objCount = 0
        for obj in self.lastPub.detections:
            center_x = obj.bbox.center.position.x
            center_y = obj.bbox.center.position.y
            size_x = obj.bbox.size.x
            size_y = obj.bbox.size.y
            pos = (int(center_x+size_x/2), int(center_y-size_y/2))
            distance = round(obj.bbox.center.position.z, 2)
            objCount += 1

            # draw Text
            try:
                cv2.putText(mask, str(distance)+"m", pos, self.font,
                            1, (0, 255, 0), 2, cv2.LINE_AA, False)
            except TypeError:
                pass

        # combine image with mask
        combinedImage = cv2.addWeighted(image, 1, mask, 1, 0)

        img = cv2.cvtColor(combinedImage, cv2.COLOR_BGR2RGB)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.img_pub.publish(image_out)

    def calcDistance(self):
        self.newObjArray = Detection3DArray()

        for obj in self.lastObj.detections:
            # keep old values
            newObj = Detection3D()
            newObj.bbox.center.position.x = obj.bbox.center.x
            newObj.bbox.center.position.y = obj.bbox.center.y
            newObj.bbox.size.x = obj.bbox.size_x
            newObj.bbox.size.y = obj.bbox.size_y

            # calculate values
            angle, angle_min, angle_max = self.calcAngle(
                obj.bbox.center.x, obj.bbox.size_x)
            distance = self.getDistance(angle, angle_min, angle_max)
            if distance == None:
                distance = 0

            # add calculated values
            newObj.bbox.center.orientation.w = angle
            newObj.bbox.center.position.z = distance

            self.newObjArray.detections.append(newObj)

        self.lastPub = self.newObjArray
        self.distance_pub.publish(self.newObjArray)

    def calcAngle(self, x, width):
        if self.flip_image:
            flip = -1
        else:
            flip = 1

        # calculate angles
        angle_min = self.angleFactor * (x - self.cam_width/2 - width/2) * flip
        angle_max = self.angleFactor * (x - self.cam_width/2 + width/2) * flip
        angle = self.angleFactor * (x - self.cam_width/2) * flip

        return angle, angle_min, angle_max

    def getDistance(self, angle, angle_min, angle_max):
        # make angles absolute
        if angle < 0:
            angle = 360 - abs(angle)

        if angle_min < 0:
            angle_min = 360 - abs(angle_min)

        if angle_max < 0:
            angle_max = 360 - abs(angle_max)

        # convert to radians
        angle = radians(angle)
        angle_min = radians(angle_min)
        angle_max = radians(angle_max)

        try:
            # calculate indexes
            index_min = int(angle_min / float(self.lastScan.angle_increment))
            index_max = int(angle_max / float(self.lastScan.angle_increment))

            # return closes distance to object
            if index_min > index_max:
                index_0 = int(
                    radians(0) / float(self.lastScan.angle_increment))
                index_360 = int(
                    radians(360) / float(self.lastScan.angle_increment))

                min_n = min(self.lastScan.ranges[index_min:index_360])
                min_p = min(self.lastScan.ranges[index_0:index_max])

                distance = min((min_n, min_p))
            else:
                distance = min(self.lastScan.ranges[index_min:index_max])

            return distance
        except:
            rospy.logerr("Can't get distance value! Error: {}")


if __name__ == "__main__":
    ros_node = DistanceNode()
    rospy.spin()
