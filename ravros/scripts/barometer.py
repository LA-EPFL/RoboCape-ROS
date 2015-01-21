#!/usr/bin/env python
# Filename: barometer.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com
#
# ROS node example that publishes barometer data to a ROS topic

import argparse
import rospy
import ravlib.sensors as sensors
import rav_msgs.msg as rav_msgs

def talker(barometer_addr, update_rate):
    # Initialise sensor
    LPS331_I2C_ADDRESS = 0x5c
    barometer = sensors.Barometer(address=barometer_addr)
    barometer.display_barometer()
    print "Barometer: Check."

    # Initialise publisher
    barometer_pub = rospy.Publisher('barometer', queue_size=1)
    rospy.init_node('barometer_stream', anonymous=True)
    rate = rospy.Rate(update_rate)

    # Infinite loop of reading & publishing
    while not rospy.is_shutdown():
        # Initialise message structure
        barometer_msg = rav_msgs.Barometer()

        # Read barometer
        pressure, altitude, temperature = barometer.get_data()

        # Write timestamp
        current_time = rospy.get_rostime()
        barometer_msg.header.stamp.secs = current_time.secs
        barometer_msg.header.stamp.nsecs = current_time.nsecs

        # Write sensor data: pressure in Pa, altitude in m, temperature in C
        barometer_msg.pressure = pressure
        barometer_msg.pressure_variance = -1 # No info
        barometer_msg.altitude_estimate = altitude
        barometer_msg.temperature = temperature
        barometer_msg.temperature_variance = -1 # No info

        # Publish
        barometer_pub.publish(barometer_msg)
        rate.sleep()

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='ROS node that publishes \
                                     Barometer data to a ROS topic')
    parser.add_argument('barometer_addr', metavar='addr', type=str, \
                        default='0x5c', help='Barometer address on I2C bus')
    parser.add_argument('update_rate', metavar='rate', type=int, default=100, \
                        help='Update rate (Hz)')
    args, unknown = parser.parse_known_args()
    barometer_addr = int(args.barometer_addr, 0) # Get hex from string
    update_rate = args.update_rate

    # Launch publisher
    try:
        talker(barometer_addr, update_rate)
    except rospy.ROSInterruptException:
        pass

