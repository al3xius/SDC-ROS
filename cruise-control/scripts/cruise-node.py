#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sdc_msgs.msg import state, arduinoIn
from simple_pid import PID
from numpy import interp


class ControlNode():
    def __init__(self):
        # pid controler
        self.testing = False

        # throttle
        self.p_vel = float(rospy.get_param("/throttle/p"))
        self.i_vel = float(rospy.get_param("/throttle/i"))
        self.d_vel = float(rospy.get_param("/throttle/d"))
        self.velPid = PID(self.p_vel, self.i_vel, self.d_vel, setpoint=0)
        self.velPid.sample_time = 0.1
        self.velPid.output_limits = (int(rospy.get_param(
            "/throttle/limitLow")), int(rospy.get_param("/throttle/limitHigh")))

        # steering
        self.p_steer = float(rospy.get_param("/steering/p"))
        self.i_steer = float(rospy.get_param("/steering/i"))
        self.d_steer = float(rospy.get_param("/steering/d"))
        self.steerPid = PID(
            self.p_steer, self.i_steer, self.d_steer, setpoint=0)
        self.steerPid.sample_time = 0.1
        self.steerPid.proportional_on_measurement = True

        self.steeringLimitLow = int(rospy.get_param("/steering/limitLow"))
        self.steeringLimitHigh = int(rospy.get_param("/steering/limitHigh"))

        self.steerPid.output_limits = (
            self.steeringLimitLow, self.steeringLimitHigh)

        # init varibles
        self.throttle = 0
        self.steeringAngle = 0
        self.offset = 0

        self.i = 0

        # subscriber
        self.sub1 = rospy.Subscriber('/lane/result', Int32, self.laneCallback)
        self.sub2 = rospy.Subscriber('/state', state, self.stateCallback)

        # publisher
        self.pub = rospy.Publisher('/cruise/state', state, queue_size=1)
        self.cruiseState = state()
        self.lastState = state()

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.publishMsg()
            r.sleep()

        rospy.spin()

    def laneCallback(self, data):
        if data.data != 0:
            pass
        self.offset = interp(
            int(data.data), [-100, 100], [self.steeringLimitLow, self.steeringLimitHigh])
        self.updateMsg()

    def stateCallback(self, data):
        self.lastState = data

    def updateMsg(self):
        if self.testing:
            self.velPid.output_limits = (int(rospy.get_param(
                "/throttle/limitLow")), int(rospy.get_param("/throttle/limitHigh")))
            self.steerPid.output_limits = (int(rospy.get_param(
                "/steering/limitLow")), int(rospy.get_param("/steering/limitHigh")))

        self.vel = self.lastState.velocity
        self.targetVelocity = self.lastState.targetVelocity
        self.velPid.setpoint = self.targetVelocity
        self.throttle = self.velPid(self.vel)
        self.throttle = self.targetVelocity * 5

        self.cruiseState.throttle = int(self.throttle)

        prevAngle = self.steeringAngle
        self.steeringAngle = self.steerPid(self.offset)
        if abs(prevAngle - self.steeringAngle) > 15 and self.i < 5:
            self.steeringAngle = prevAngle
            self.i += 1
        else:
            self.i = 0
        self.cruiseState.steeringAngle = int(self.steeringAngle)

    def publishMsg(self):
        self.pub.publish(self.cruiseState)


if __name__ == '__main__':
    rospy.init_node('cruiseControl', anonymous=False)
    rospy.loginfo("Cruise Control: Node started.")
    try:
        node = ControlNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Cruise Control: Node stoped.")
