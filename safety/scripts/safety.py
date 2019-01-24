#!/usr/bin/env python
import rospy 
from sdc_msgs.msg import state, arduinoIn
from sensor_msgs.msg import LaserScan

class SafetyNode:
    def stateCallback(self, msg):
        self.state = msg
        self.stopingDistance = msg.velocity*msg.velocity * self.breakingFactor

        self.publish()
        pass

    def laserCallback(self, msg):
        if min(msg) < self.stopingDistance*0.9:
            self.state.state = "stopping"
            self.state.enableSteering = True
			self.state.steeringAngle = msg.steeringAngle #TODO: Steer away
			self.state.enableMotor = False
			self.state.throttle = 0
			self.state.direction = 0
			self.state.light = msg.light
			self.state.indicate = "Both"
        pass

    def publish(self):
        self.state_pub.publish(self.state)
        pass

    def __init__(self):
        rospy.init_node("safety_node")
        rospy.loginfo("Starting SafetyNode.")
    

        self.stopingDistance = None
        normBreakingDistance = rospy.get_param("/normBreakingDistance", default="10")
        normVelocity = rospy.get_param("/normVelocity", default="10")
        
        self.breakingFactor = normBreakingDistance/normVelocity*normVelocity


        rospy.Subscriber("/state/unchecked", state, self.stateCallback)
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        


        self.state = state()
        self.state_pub = rospy.Publisher("/state", state, queue_size=1)
              
        pass


if __name__ == "__main__":
    safety_node = SafetyNode()
    rospy.spin()