#!/usr/bin/env python
# Filename: gps.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com
#
# ROS node example that publishes GPS data to a ROS topic

import rospy
import ravlib.sensors as sensors
import sensor_msgs.msg as sensor_msgs

def talker():
    # Initialise sensor
    gps = sensors.Gps(channel=sensors.UART_1, update_rate=1, baudrate=9600)
    gps.display_gps()
    print "GPS: Check."

    # Initialise publisher
    gps_data_pub = rospy.Publisher('gps', sensor_msgs.NavSatFix, queue_size=5)
    rospy.init_node('gps_stream', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    # Infinite loop of reading & publishing
    while not rospy.is_shutdown():
        # Initialise message structure
        gps_data_msg = sensor_msgs.NavSatFix()

        # Read GPS
        fix, sat, lat, lon, alt, spd, hdg = gps.get_data()

        # Write timestamp
        current_time = rospy.get_rostime()
        gps_data_msg.header.stamp.secs = current_time.secs
        gps_data_msg.header.stamp.nsecs = current_time.nsecs

        # Write GPS data
        gps_data_msg.status.status = fix
        gps_data_msg.status.service = gps_data_msg.status.SERVICE_GPS
        gps_data_msg.latitude = lat
        gps_data_msg.longitude = lon
        gps_data_msg.altitude = alt

        # Covariance information from the datasheet
        gps_data_msg.position_covariance = [3.0**2, 0.0, 0.0, \
                                            0.0, 3.0**2, 0.0, \
                                            0.0, 0.0, 4.5**2]
        gps_data_msg.position_covariance_type = \
            gps_data_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Publish
        gps_data_pub.publish(gps_data_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
