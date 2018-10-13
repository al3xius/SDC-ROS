#!/usr/bin/env python
"""TODO: 	+ calibrate BatteryVoltage
"""

import rospy
import message_filters
from numpy import interp
# Messages
from rosserial_arduino.msg import Adc
from std_msgs.msg import Int16
from sdc_msgs.msg import state, arduinoIn
from sensor_msgs.msg import Joy


def limitValue(value, min, max):
    """Limits value to a min or max value
    
    Arguments:
        value {int, float} -- value
        min {int, float} -- minValue
        max {int, float} -- maxValue
    
    Returns:
        int, float -- limited value
    """

    if value > max:
        out = max
    elif value < min:
        out = min
    else:
        out = value
    return out


class StateMachine():

	def arduCallback(self, arduinoIn):
		# Battery Voltage -> Percent
		voltage = interp(getattr(arduinoIn, self.batteryPin), [0, 1015], [0, 5]) * self.batteryFactor
		self.percent = limitValue((voltage - self.batteryD)/self.batteryK, 0, 100)

		# Gaspeddal -> output
		self.gasPedal = limitValue(interp(getattr(arduinoIn, self.gasPedalPin), [0, 1015], [0, 100]), 0, 100)
		
		# Direction
		if not getattr(arduinoIn, self.forwardInPin) and not getattr(arduinoIn, self.gasPedalSwitchPin) and getattr(arduinoIn, self.backwardInPin):
			self.manEnableMotor = True
			self.manDirection = 1
		elif not getattr(arduinoIn, self.backwardInPin) and not getattr(arduinoIn, self.gasPedalSwitchPin) and getattr(arduinoIn, self.forwardInPin):
			self.manEnableMotor = True
			self.manDirection = -1
		else:
			self.manEnableMotor = False
			self.manDirection = 0


		self.publishState()

	def joyCallback(self, Joy):
		#reset to previous mode
		if Joy.buttons[5] and not self._remote:
			self.pervMode = self.mode

		if Joy.buttons[5]:
			self._remote = True
			self.mode = "remote"
		else:
			self._remote = False
			#safety
			if self.pervMode == "remote":
				self.pervMode = "manual"
				self.mode = self.pervMode
			else:
				self.mode = self.pervMode
		
		self.joyThrottle = interp(Joy.axes[1], [-1, 1], [-100, 100])
		self.joySteeringAngle = interp(Joy.axes[0], [-1, 1], [-100, 100])
		self.publishState()

	def publishState(self):

		if self.mode == "manual":
			self.enableSteering = False
			self.steeringAngle = 0
			self.throttle = self.gasPedal
			self.enableMotor = self.manEnableMotor
			self.direction = self.manDirection

		elif self.mode == "cruse" and not getattr(arduinoIn, self.stopPin):
			# TODO Get data from cruse control node
			self.throttle = 0
			self.enableMotor = False
			self.steeringAngle = 0
			self.enableSteering = False

		elif self.mode == "remote":
			self.throttle = abs(self.joyThrottle)
			if self.joyThrottle < 0:
				self.direction = -1
			elif self.joyThrottle > 0:
				self.direction = 1
			else:
				self.direction = 0

			self.enableMotor = True
			self.enableSteering = True
			self.steeringAngle = self.joySteeringAngle

		else:
			self.enableSteering = False
			self.steeringAngle = 0
			self.enableMotor = False
			self.throttle = 0
			self.direction = 0

		self.state.throttle = limitValue(self.throttle, 0, 100)
		self.state.mode = self.mode
		self.state.batteryCharge = self.percent
		self.state.gasPedal = self.gasPedal
		self.state.velocity = 0
		self.state.steeringAngle = limitValue(self.steeringAngle, -100, 100)
		self.state.enableSteering = self.enableSteering
		self.state.enableMotor = self.enableMotor
		self.state.direction = self.direction

		self.pub.publish(self.state)

	def __init__(self):
		# get params
		# Pins
		self.batteryPin = rospy.get_param("/arduino/batteryPin")
		self.gasPedalPin = rospy.get_param("/arduino/gasPedalPin")
		self.forwardInPin = rospy.get_param("/arduino/forwardInPin")
		self.backwardInPin = rospy.get_param("/arduino/backwardInPin")
		self.gasPedalSwitchPin = rospy.get_param("/arduino/gasPedalSwitchPin")
		self.stopPin = rospy.get_param("/arduino/stopPin")
		self.keySwitchPin = rospy.get_param("/arduino/keySwitchPin")

		# Battery
		self.batteryK = float(rospy.get_param("/battery/k"))
		self.batteryD = float(rospy.get_param("/battery/d"))
		self.batteryFactor = float(rospy.get_param("/battery/factor"))

		#init
		self.mode = "manual"
		self.pervMode = self.mode
		self._remote = False

		self.state = state()


		#subscribe
		self.sub1 = rospy.Subscriber('arduino/in', arduinoIn, self.arduCallback)
		self.sub2 = rospy.Subscriber('joy', Joy, self.joyCallback)

		#publish
		self.pub = rospy.Publisher("/state", state, queue_size=1)

		rospy.spin()



if __name__ == '__main__':
	rospy.init_node('stateMachine', anonymous=False)
	try:
		machine = StateMachine()
	except rospy.ROSInterruptException:  
		pass

                   
