#!/usr/bin/env python
import rospy 
from math import radians
from sdc_msgs.msg import state, arduinoIn
from sensor_msgs.msg import LaserScan

class SafetyNode:
    def stateCallback(self, msg):
        self.state = msg
        # calculate stoping distance
        self.stopingDistance = msg.velocity*msg.velocity * self.breakingFactor

        self.publish()
        pass

    def laserCallback(self, msg):
        angle_min = radians(self.emergencyBreakAngle + self.state.steeringAngel)
        angle_max = radians(360) - (self.emergencyBreakAngle + self.state.steeringAngel)

        index_0 = int(radians(0)/ float(self.lastScan.angle_increment))
        index_360 = int(radians(360)/ float(self.lastScan.angle_increment))
        index_min = int(angle_min / float(self.lastScan.angle_increment))
        index_max = int(angle_max / float(self.lastScan.angle_increment))

        min_n = min(self.lastScan.ranges[index_max:index_360])
        min_p = min(self.lastScan.ranges[index_0:index_min])

        emergencyDistance = min((min_n, min_p))

        if emergencyDistance < self.stopingDistance*0.9:
            rospy.logerr("Emergency Break!")
            self.state.state = "break"
            self.state.enableSteering = True
            self.state.steeringAngle = 0 #TODO: Steer away
            self.state.enableMotor = False
            self.state.throttle = 0
            self.state.direction = 0
            self.state.breaking = 100
            #self.state.light = msg.light
            self.state.indicate = "Both"
        pass

        self.publish()

    def publish(self):
        self.state_pub.publish(self.state)
        pass

    def __init__(self):
        rospy.init_node("safety_node")
        rospy.loginfo("Starting SafetyNode.")
    

        self.stopingDistance = None
        normBreakingDistance = rospy.get_param("/safety/normBreakingDistance", default="10")
        normVelocity = rospy.get_param("/safety/normVelocity", default="10")
        self.emergencyBreakAngle = rospy.get_param("/safety/emergencyBreakAngle", default="10")
        
        
        self.breakingFactor = normBreakingDistance/(normVelocity*normVelocity)


        rospy.Subscriber("/state/unchecked", state, self.stateCallback)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        


        self.state = state()
        self.state_pub = rospy.Publisher("/state", state, queue_size=1)
              
        pass


if __name__ == "__main__":
    safety_node = SafetyNode()
    rospy.spin()