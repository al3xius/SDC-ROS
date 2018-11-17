#!/usr/bin/env python

# Simple GPS module demonstration.
# Will wait for a fix and print a message every second with the current location
# and other details.
# parts of code from https://learn.adafruit.com/adafruit-ultimate-gps/circuitpython-parsing
import time, serial, rospy
from sensor_msgs.msg import NavSatFix


import adafruit_gps




# for a computer, use the pyserial library for uart access
uart = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=3000)

# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on just minimum info (RMC only, location):
#gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
#gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Tuen on everything (not all of it is parsed!)
#gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Set update rate to once a second (1hz) which is what you typically want.
gps.send_command(b'PMTK220,1000')
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
#gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
#gps.send_command(b'PMTK220,500')

# Main loop runs forever printing the location, etc. every second.

def gps_talker():
    rospy.loginfo("GPS: Node started.")
    pub = rospy.Publisher('gps', NavSatFix, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    msg = NavSatFix()
    seq = 0
    last_print = time.monotonic()

    
    while not rospy.is_shutdown():
        gps.update()
        current = time.monotonic()

        if current - last_print >= 1.0:
            seq += 1
            last_print = current

            msg.header.seq = seq
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'
            if prev_has_fix != gps.has_fix:
                if gps.has_fix:
                    rospy.loginfo("GPS: has fix")
                else:
                    rospy.loginfo("GPS: lost fix")

            prev_has_fix = gps.has_fix

            if not gps.has_fix:
                # Try again if we don't have a fix yet.
                msg.status.status = int(0)
                #publish
                pub.publish(msg)
                continue
            else:
                msg.status.status = int(1)
            # We have a fix! (gps.has_fix is true)
            # Print out details about the fix like location, date, etc.
            msg.status.service = int(0)
            
            # position
            msg.latitude = float(gps.latitude)
            msg.longitude = float(gps.longitude)
            
            #print('Fix quality: {}'.format(gps.fix_quality))
            # Some attributes beyond latitude, longitude and timestamp are optional
            # and might not be present.  Check if they're None before trying to use!
            """if gps.satellites is not None:
                print('# satellites: {}'.format(gps.satellites))"""

            #!!!!!!using altitude as velocity!!!!!
            if gps.gps.speed_knots is not None:
                msg.altitude = float(gps.speed_knots)*1.852

            #TODO: velocity
            """if gps.track_angle_deg is not None:
                print('Speed: {} knots'.format(gps.speed_knots))
            if gps.track_angle_deg is not None:
                print('Track angle: {} degrees'.format(gps.track_angle_deg))
            if gps.horizontal_dilution is not None:
                print('Horizontal dilution: {}'.format(gps.horizontal_dilution))
            if gps.height_geoid is not None:
                print('Height geo ID: {} meters'.format(gps.height_geoid))"""

            #publish
            pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('gps', anonymous=False)
        gps_talker()
    except rospy.ROSInterruptException:
        pass