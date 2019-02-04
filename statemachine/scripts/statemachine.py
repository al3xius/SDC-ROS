#!/usr/bin/env python
"""TODO: 	+ calibrate BatteryVoltage
"""

import rospy
import message_filters
from numpy import interp
# Messages
# from rosserial_arduino.msg import Adc #unused
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
    
	def toggleLight(self, input):
		"""toggles light by pressing button in car or in joy"""

		if input and self._prevLightIn:
			self.light = not self.light
			self._prevLightIn = False
		elif not input:
			self._prevLightIn = True

	def arduCallback(self, arduinoIn):
		"""processes Arduino callback"""

		# Battery Voltage -> Percent
		voltage = interp(arduinoIn.analog[self.batteryPin], [0, 1015], [0, 5]) * self.batteryFactor
		self.percent = limitValue((voltage - self.batteryD)/self.batteryK, 0, 100)

		# get Speed
		self.state.velocity = arduinoIn.analog[6]

		# get gaspeddal
		# TODO: calibrate gaspedal
		#self.gasPedal = limitValue(interp(arduinoIn.analog[self.gasPedalPin], [self.gasPedalMin, self.gasPedalMax], [0, 100]), 0, 100)
		self.gasPedal = limitValue(arduinoIn.analog[self.gasPedalPin], 0, 100)
		
		# get direction
		if not arduinoIn.digital[self.forwardInPin] and not arduinoIn.digital[self.gasPedalSwitchPin] and arduinoIn.digital[self.backwardInPin]:
			self.manEnableMotor = True
			self.manDirection = 1
		elif not arduinoIn.digital[self.backwardInPin] and not arduinoIn.digital[self.gasPedalSwitchPin] and arduinoIn.digital[self.forwardInPin]:
			self.manEnableMotor = True
			self.manDirection = -1
		else:
			self.manEnableMotor = False
			self.manDirection = 0
 
		# toggle light
		self.toggleLight(not arduinoIn.digital[self.lightInPin])

		# set indicator
		if not arduinoIn.digital[self.indicatorInLPin]:
			self.manIndicate= "Left"
		elif not arduinoIn.digital[self.indicatorInRPin]:
			self.manIndicate = "Right"
		else:
			self.manIndicate = "None"
		

		self.key = arduinoIn.digital[self.keySwitchPin]


		self.publishState()

	def joyCallback(self, Joy):
		"""processes Joystick callback
			button assigment:
			leftStick: steering | throttle
			r1: enable remote control
			start: toggle light

		"""

		# set mode to remote
		if Joy.buttons[5] and not self._remote:
			self.pervMode = self.mode # reset to previous mode

		if Joy.buttons[5]:
			self._remote = True
			self.mode = "remote"
		else:
			self._remote = False
			# safety: prevent gettings stuck in a loop
			if self.pervMode == "remote" or self.pervMode == "cruise":
				self.pervMode = "manual"
			
			self.mode = self.pervMode
		
		self.toggleLight(Joy.buttons[8])

		self.joyThrottle = interp(Joy.axes[1], [-1, 1], [-100, 100])
		self.joySteeringAngle = interp(Joy.axes[0], [-1, 1], [-100, 100])
		self.publishState()

	def cruiseCallback(self, state):
		self.cruiseState = state

	def guiCallback(self, state):
    	self.guiState = state


	def publishState(self):
		if not self.key and not self._key:
			self.mode = "manual"
			self._key = True
		elif self.key:
			self.mode = "locked"
			self._key = False
		
		if self.mode == "manual":
			self.enableSteering = False
			self.steeringAngle = 0
			self.throttle = self.gasPedal
			self.enableMotor = self.manEnableMotor
			self.direction = self.manDirection
			self.indicate = self.manIndicate

		elif self.mode == "cruise":
			self.throttle = self.cruiseState.throttle
			self.targetVelocity = self.guiState.targetVelocity
			self.enableMotor = True
			self.direction = 1
			self.steeringAngle = self.cruiseState.steeringAngle
			self.enableSteering = True

		elif self.mode == "remote":
			self.throttle = abs(self.joyThrottle) # make values positive
			# set direction
			if self.joyThrottle < 0:
				self.enableMotor = True
				self.direction = -1
			elif self.joyThrottle > 0:
				self.enableMotor = True
				self.direction = 1
			else:
				self.enableMotor = False
				self.direction = 0
			
			self.enableSteering = True
			self.steeringAngle = self.joySteeringAngle

			# Indicate that the vehicle is controlled remotely
			self.indicate = "Both"
		
		#TODO implement locked mode
		elif self.mode == "locked":
			self.enableSteering = False
			self.steeringAngle = 0
			self.enableMotor = False
			self.throttle = 0
			self.direction = 0
			self.light = False
			self.indicate = "None"

		else:
			self.enableSteering = False
			self.steeringAngle = 0
			self.enableMotor = False
			self.throttle = 0
			self.direction = 0
			self.light = False
			self.indicate = "None"


		# publish curent state
		self.state.throttle = limitValue(self.throttle, 0, 100)
		self.state.mode = self.mode
		self.state.batteryCharge = self.percent
		self.state.gasPedal = self.gasPedal
		self.state.steeringAngle = limitValue(self.steeringAngle, -100, 100)
		self.state.enableSteering = self.enableSteering
		self.state.enableMotor = self.enableMotor
		self.state.direction = self.direction
		self.state.light = self.light
		self.state.indicate = self.indicate

		self.pub.publish(self.state)

	def updateParams(self):
		# get params
		# Pins
		self.batteryPin = int(rospy.get_param("/arduino/pin/battery")[3:])
		self.gasPedalPin = int(rospy.get_param("/arduino/pin/gasPedal")[3:])
		self.forwardInPin = int(rospy.get_param("/arduino/pin/forwardIn")[1:])
		self.backwardInPin = int(rospy.get_param("/arduino/pin/backwardIn")[1:])
		self.gasPedalSwitchPin = int(rospy.get_param("/arduino/pin/gasPedalSwitch")[1:])
		self.stopPin = int(rospy.get_param("/arduino/pin/stop")[1:])
		self.keySwitchPin = int(rospy.get_param("/arduino/pin/keySwitch")[1:])

		# Lights
		self.indicatorInLPin = int(rospy.get_param("/arduino/pin/indicatorInL")[1:])
		self.indicatorInRPin = int(rospy.get_param("/arduino/pin/indicatorInR")[1:])
		self.lightInPin = int(rospy.get_param("/arduino/pin/lightIn")[1:])

		# Battery
		self.batteryK = float(rospy.get_param("/battery/k"))
		self.batteryD = float(rospy.get_param("/battery/d"))
		self.batteryFactor = float(rospy.get_param("/battery/factor"))

		# Gaspedal
		self.gasPedalMin = int(rospy.get_param("/gaspedal/min"))
		self.gasPedalMax = int(rospy.get_param("/gaspedal/max"))


	def __init__(self):
		rospy.init_node('stateMachine', anonymous=False)
		rospy.loginfo("Starting State Machine.")

		self.updateParams()

		#init
		self.mode = "manual"
		self.indicate = "None"
		self.pervMode = self.mode
		self._remote = False
		self.key = False
		self._key = False
		self._prevLightIn = True
		self.light = False

		self.state = state()
		self.cruiseState = state()
		self.guiState = state()
		self.arduInit = arduinoIn()

		# subscriber
		self.sub1 = rospy.Subscriber('/arduino/in', arduinoIn, self.arduCallback)
		self.sub2 = rospy.Subscriber('/joy', Joy, self.joyCallback)
		self.sub3 = rospy.Subscriber('/cruise/state', state, self.cruiseCallback)
		self.sub4 = rospy.Subscriber('/gui/state', state, self.guiCallback)

		# publisher
		self.pub = rospy.Publisher("/state/unchecked", state, queue_size=1)


		# service
		#param_serv = rospy.Service('updateParams', statemachine.srv.updateParams, self.updateParams)

		self.arduCallback(self.arduInit)

		rospy.spin()



if __name__ == '__main__':
	try:
		machine = StateMachine()
	except rospy.ROSInterruptException:  
		rospy.logerr("State Machine: Node stoped.")
