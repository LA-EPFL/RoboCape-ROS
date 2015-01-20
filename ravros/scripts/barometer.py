#!/usr/bin/env python
# Filename: barometer.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com
#
# ROS node example that publishes barometer data to a ROS topic

import rospy
import ravlib.sensors as sensors
import std_msgs.msg as std_msgs

def talker():
    # Initialise sensor
    LPS331_I2C_ADDRESS = 0x5c
    barometer = sensors.Barometer(address=LPS331_I2C_ADDRESS)
    barometer.display_barometer()
    print "Barometer: Check."

    # Initialise publisher
    barometer_pub = rospy.Publisher('barometer', queue_size=1)
    rospy.init_node('barometer_stream', anonymous=True)
    rate = rospy.Rate(2) # 2hz

    # Infinite loop of reading & publishing
    while not rospy.is_shutdown():
        # Initialise message structure
        barometer_msg.header = std_msgs.Header()
        barometer_msg.pressure = std_msgs.Float64()
        barometer_msg.pressure_variance = std_msgs.Float64()
        barometer_msg.altitude = std_msgs.Float64()
        barometer_msg.temperature = std_msgs.Float64()
        barometer_msg.temperature_variance = std_msgs.Float64()

        # Read barometer
        pressure, altitude, temperature = barometer.get_data()

        # Write timestamp
        current_time = rospy.get_rostime()
        barometer_msg.header.stamp.secs = current_time.secs
        barometer_msg.header.stamp.nsecs = current_time.nsecs

        # Write sensor data: pressure in Pa, altitude in m, temperature in C
        barometer_msg.pressure = pressure
        barometer_msg.pressure_variance = -1 # No info
        barometer_msg.altitude = altitude
        barometer_msg.temperature = temperature
        barometer_msg.temperature_variance = -1 # No info

        #rospy.loginfo("Vector published")
        barometer_pub.publish(barometer_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

