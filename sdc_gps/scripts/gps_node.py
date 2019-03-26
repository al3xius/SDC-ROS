#!/usr/bin/env python

# GPS module
# parts of code from https://learn.adafruit.com/adafruit-ultimate-gps/circuitpython-parsing

import time
import serial
import rospy
import adafruit_gps
from sensor_msgs.msg import NavSatFix


# define Serial interface
uart = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3000)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

gps.send_command(b'PMTK220,1000')


def gps_talker():
    rospy.loginfo("GPS: Node started.")
    pub = rospy.Publisher('gps', NavSatFix, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    msg = NavSatFix()
    seq = 0
    lastPub = time.monotonic()

    while not rospy.is_shutdown():
        gps.update()
        currentTime = time.monotonic()

        # publish every second
        if currentTime - lastPub >= 1.0:
            seq += 1
            lastPub = currentTime

            # fill header
            msg.header.seq = seq
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'

            # log if gps knows its position (fix)
            if prev_has_fix != gps.has_fix:
                if gps.has_fix:
                    rospy.loginfo("GPS: has fix")
                else:
                    rospy.loginfo("GPS: lost fix")

            prev_has_fix = gps.has_fix

            if not gps.has_fix:
                # Try again if no fix
                msg.status.status = int(0)

                # publish
                pub.publish(msg)
                continue
            else:
                msg.status.status = int(1)

            msg.status.service = int(0)

            # position
            msg.latitude = float(gps.latitude)
            msg.longitude = float(gps.longitude)

            #!!!!!!using altitude as velocity!!!!!
            if gps.gps.speed_knots is not None:
                msg.altitude = float(gps.speed_knots)*1.852

            # publish
            pub.publish(msg)


if __name__ == '__main__':
    try:
        # init
        rospy.init_node('gps', anonymous=False)

        # start node
        gps_talker()
    except rospy.ROSInterruptException:
        pass
