#!/usr/bin/env python
# Filename: robotsCar.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com

import threading
import time
import socket
import ravlib.sensors as sensors
import rospy
from geometry_msgs.msg import Vector3

def imu_handler():
    # Initialisation
    MPU9150_I2C_ADDRESS = 0x68
    imu = sensors.Imu(address=MPU9150_I2C_ADDRESS)

    imu.display_imu()
    print "IMU: Check."

    pub = rospy.Publisher('imu', Vector3, queue_size=10)
    rospy.init_node('accel', anonymous=True)
    rate = rospy.Rate(100) # 100hz

    # Infinite Loop
    while not rospy.is_shutdown():
        a_x, a_y, a_z, g_x, g_y, g_z = imu.get_data()
        vector = Vector3()
        vector.x = a_x
        vector.y = a_y
        vector.z = a_z
        rospy.loginfo("Accelerometer")
        pub.publish(vector)
        rate.sleep()

if __name__ == "__main__":
    try:
        imu_handler()
    except rospy.ROSInterruptException:
        pass

