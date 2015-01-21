#!/usr/bin/env python
# Filename: imu.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com
#
# ROS node example that publishes IMU data to a ROS topic

import argparse
import rospy
import ravlib.sensors as sensors
import sensor_msgs.msg as sensor_msgs

def talker(imu_addr, update_rate):
    # Initialise sensor
    imu = sensors.Imu(address=imu_addr)
    imu.display_imu()
    print "IMU: Check."

    # Initialise publisher
    imu_pub = rospy.Publisher('imu', sensor_msgs.Imu, queue_size=25)
    rospy.init_node('imu_stream', anonymous=True)
    rate = rospy.Rate(update_rate)

    # Infinite loop of reading & publishing
    while not rospy.is_shutdown():
        # Initialise message structure
        imu_msg = sensor_msgs.Imu()

        # Read IMU
        ax, ay, az, wx, wy, wz, cx, cy, cz, temp = imu.get_data()

        # Write timestamp
        current_time = rospy.get_rostime()
        imu_msg.header.stamp.secs = current_time.secs
        imu_msg.header.stamp.nsecs = current_time.nsecs

        # Write accelerometer data (m/s^2)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.linear_acceleration_covariance = [0.04**2, 0.0, 0.0, \
                                                  0.0, 0.04**2, 0.0, \
                                                  0.0, 0.0, 0.04**2]

        # Write gyroscope data (rad/s)
        imu_msg.angular_velocity.x = wx
        imu_msg.angular_velocity.y = wy
        imu_msg.angular_velocity.z = wz
        imu_msg.angular_velocity_covariance = [0.0011**2, 0.0, 0.0, \
                                               0.0, 0.0011**2, 0.0, \
                                               0.0, 0.0, 0.0011**2]

        # Write compass normalised vector
        imu_msg.orientation.x = cx
        imu_msg.orientation.y = cy
        imu_msg.orientation.z = cz
        imu_msg.orientation.w = -1  # This is not a quaternion
        imu_msg.orientation_covariance = [ -1, 0.0, 0.0, \
                                          0.0, 0.0, 0.0, \
                                          0.0, 0.0, 0.0]

        # Publish
        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    # Parse arguments
    parser = argparse.ArgumentParser(description='ROS node that publishes IMU \
                                     data to a ROS topic')
    parser.add_argument('imu_addr', metavar='addr', type=str, default='0x68', \
                        help='IMU address on I2C bus')
    parser.add_argument('update_rate', metavar='rate', type=int, default=100, \
                        help='Update rate (Hz)')
    args, unknown = parser.parse_known_args()
    imu_addr = int(args.imu_addr, 0) # Get hex from string
    update_rate = args.update_rate

    # Launch publisher
    try:
        talker(imu_addr, update_rate)
    except rospy.ROSInterruptException:
        pass
