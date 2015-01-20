#!/usr/bin/env python
# Filename: imu.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com
#
# ROS node example that publishes IMU data to a ROS topic

import rospy
import ravlib.sensors as sensors
import sensor_msgs.msg as sensor_msgs

def talker():
    # Initialise sensor
    MPU9150_I2C_ADDRESS = 0x68
    imu = sensors.Imu(address=MPU9150_I2C_ADDRESS)
    imu.display_imu()
    print "IMU: Check."

    # Initialise publisher
    imu_pub = rospy.Publisher('imu', sensor_msgs.Imu, queue_size=25)
    rospy.init_node('imu_stream', anonymous=True)
    rate = rospy.Rate(100) # 100hz

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
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

