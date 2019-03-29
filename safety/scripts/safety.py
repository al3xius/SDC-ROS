#!/usr/bin/env python

import rospy
from math import radians
from sdc_msgs.msg import state, arduinoIn
from sensor_msgs.msg import LaserScan


class SafetyNode:
    def stateCallback(self, msg):
        self.state = msg

        # calculate stoping distance
        self.stopingDistance = msg.throttle*msg.throttle * self.breakingFactor
        print(self.stopingDistance)

        self.publish()
        pass

    def laserCallback(self, msg):
        # calculate angles
        angle_min = radians(180 - self.emergencyBreakAngle)
        angle_max = radians(180 + self.emergencyBreakAngle)
        self.lastScan = msg

        # calculate index
        index_0 = int(radians(0) / float(self.lastScan.angle_increment))
        index_360 = int(radians(360) / float(self.lastScan.angle_increment))
        index_min = int(angle_min / float(self.lastScan.angle_increment))
        index_max = int(angle_max / float(self.lastScan.angle_increment))

        # get min value
        min_n = min(self.lastScan.ranges[index_min:index_max])
        #min_p = min(self.lastScan.ranges[index_0:index_min])

        #emergencyDistance = min((min_n, min_p))
        emergencyDistance = min_n

        # break if object in front of car
        if emergencyDistance < 2.0 or emergencyDistance < self.stopingDistance * 0.9:
            rospy.logerr("Emergency Break!")
            self.state.mode = "break"
            self.state.enableSteering = False
            self.state.steeringAngle = 0  # TODO: Steer away
            self.state.enableMotor = False
            self.state.throttle = 0
            self.state.direction = 0
            self.state.breaking = 100
            #self.state.light = msg.light
            self.state.indicate = "Both"

        self.publish()

    def publish(self):
        self.state_pub.publish(self.state)
        pass

    def __init__(self):
        rospy.init_node("safety_node")
        rospy.loginfo("Starting SafetyNode.")

        self.stopingDistance = None
        normBreakingDistance = float(rospy.get_param(
            "/safety/normBreakingDistance", default="10"))
        normVelocity = float(
            rospy.get_param("/safety/normVelocity", default="10"))
        self.emergencyBreakAngle = rospy.get_param(
            "/safety/emergencyBreakAngle", default="10")

        self.breakingFactor = float(
            normBreakingDistance)/(normVelocity*normVelocity)
        print("breakf:" + str(self.breakingFactor))

        # subscribe
        rospy.Subscriber("/state/unchecked", state, self.stateCallback)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)

        # publish
        self.state = state()
        self.state_pub = rospy.Publisher("/state", state, queue_size=1)
        pass


if __name__ == "__main__":
    safety_node = SafetyNode()
    rospy.spin()
