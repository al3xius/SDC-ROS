#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sdc_msgs.msg import state, arduinoIn
from simple_pid import PID


class ControlNode():
    def __init__(self):
        # pid controler
        # throttle
        self.p_vel = float(rospy.get_param("/throttle/p"))
        self.i_vel = float(rospy.get_param("/throttle/i"))
        self.d_vel = float(rospy.get_param("/throttle/d"))
        self.velPid = PID(self.p_vel, self.i_vel, self.d_vel, setpoint=0)
        self.velPid.sample_time = 0.01
        self.velPid.output_limits = (0, 25)

        # steering
        self.p_steer = float(rospy.get_param("/steering/p"))
        self.i_steer = float(rospy.get_param("/steering/i"))
        self.d_steer = float(rospy.get_param("/steering/d"))
        self.steerPid = PID(
            self.p_steer, self.i_steer, self.d_steer, setpoint=0)
        self.steerPid.sample_time = 0.01
        self.steerPid.output_limits = (-10, 10)

        # init varibles
        self.throttle = 0
        self.steeringAngle = 0

        self.i = 0

        # subscriber
        self.lane = rospy.Subscriber('/lane/result', Int32, self.laneCallback)
        #self.state = rospy.Subscriber('/arduino/in', arduinoIn, self.stateCallback)

        # publisher
        self.pub = rospy.Publisher('/cruise/state', state, queue_size=1)
        self.cruiseState = state()
        self.stateMach = arduinoIn()
        rospy.spin()

    def laneCallback(self, data):
        self.offset = int(data.data)
        self.steeringAngle = self.steerPid(self.offset)
        self.publishMsg()

    def stateCallback(self, data):
        self.stateMach = data
        """if data.mode == "cruise":
            # get curent velocity
            self.vel = data.analog[6]
            self.targetVelocity = data.targetVelocity

            self.velPid.setpoint = self.targetVelocity
            self.throttle = self.velPid(self.vel)
        else:
            self.throttle = 0"""
        #self.publishMsg()

    def publishMsg(self):
        """self.vel = self.stateMach.analog[6]
        self.targetVelocity = 5  # data.targetVelocity

        self.velPid.setpoint = self.targetVelocity
        self.throttle = self.velPid(self.vel)"""

        self.cruiseState.steeringAngle = int(self.steeringAngle)
        self.cruiseState.throttle = 25  # int(self.throttle)
        self.pub.publish(self.cruiseState)


if __name__ == '__main__':
    rospy.init_node('cruiseControl', anonymous=False)
    rospy.loginfo("Cruise Control: Node started.")
    try:
        node = ControlNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Cruise Control: Node stoped.")
