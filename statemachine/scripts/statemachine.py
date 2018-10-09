"""TODO: 	+ format state topic
			+ calculate factor for real voltage
"""


import rospy
from rosserial_arduino import adc_msg
from std_msgs.msg import Int16


def callback(data):
	voltage = data * factor
	percent = (voltage - batteryD)/batteryK
	pub.publish(percent)

def listener():
    sub = rospy.Subscriber('adc', adc_msg, callback)
    rospy.spin()

if __name__ == '__main__':
    # get Param
    batteryPin = rospy.get_param("/arduino/batteryPin")
	batteryK = rospy.get_param("/battery/k")
	batteryD = rospy.get_param("/battery/d")
	batteryFactor = rospy.get_param("/battery/factor")
	pub = rospy.Publisher("/state/battery", Int16)
	listener()
                   
