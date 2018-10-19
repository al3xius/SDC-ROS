#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sdc_msgs.msg import state
from simple_pid import PID

class ControlNode():
    def __init__(self):
        # subscriber
        self.lane = rospy.Subscriber('/lane/result', string, self.laneCallback)
        self.state = rospy.Subscriber('/state', state, self.stateCallback)

        # publisher
        self.pub = rospy.Publisher('/cruise/state', state, queue_size=1)

        # pid controler
        # throttle
        self.p_vel = float(rospy.get_param("/throttle/p"))
        self.i_vel = float(rospy.get_param("/throttle/i"))
        self.d_vel = float(rospy.get_param("/throttle/d"))
        self.velPid = PID(self.p_vel, self.i_vel, self.d_vel, setpoint=0)
        self.velPid.output_limits(0, 100)

        # steering TODO: get absolute value from lanekeeping
        self.p_steer = float(rospy.get_param("/steering/p"))
        self.i_steer = float(rospy.get_param("/steering/i"))
        self.d_steer = float(rospy.get_param("/steering/d"))
        self.steerPid = PID(self.p_steer, self.i_steer, self.d_steer, setpoint=0)


    def laneCallback(self, data):
        pass

    def stateCallback(self, data):
        self.state = data.state

        if state == "cruise":
            # get curent velocity
            self.vel = data.velocity
            self.targetVelocity = data.targetVelocity

            self.velPid.setpoit = self.targetVelocity
            self.pub.throttle = self.velPid(self.vel)
        else:
            self.pub.throttle = 0


if __name__ == '__main__':
    # create statemachine node
	rospy.init_node('cruiseControl', anonymous=False)
	try:
		node = ControlNode()
	except rospy.ROSInterruptException:  
		pass