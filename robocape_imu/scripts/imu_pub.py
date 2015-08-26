#!/usr/bin/env python
from __future__ import division
import rospy
import tf
from sensor_msgs.msg import Imu
from mpu9150_driver import Mpu9150


class ImuHandler:
    def __init__(self):
        node_name = 'imu_mpu9150_handler'
        rospy.init_node(node_name)
        rospy.loginfo(rospy.get_caller_id() + 'Initialising {} node'.format(node_name))

        # Get all parameters from config (rosparam)
        address = int(rospy.get_param('imu/address', MPU9150_I2C_ADDR_DEFAULT))
        gyro_scale = int(rospy.get_param('imu/gyroscope/scale', 250))
        acc_scale = int(rospy.get_param('imu/accelerometer/scale', 2))

        self.imu = Mpu9150(address, gyro_scale, acc_scale)

    def publish(self):
        queue_size = rospy.get_param('imu/queue', 1)
        self.pub = rospy.Publisher('imu_readings', Imu, queue_size=queue_size)
        self.rate = rospy.Rate(rospy.get_param('imu/rate', 10))

        self.msg = self.msg_template()

        while not rospy.is_shutdown():
            rospy.loginfo(rospy.get_caller_id() + "publishing IMU data")

            self.imu.update()
            self.pack_message()
            self.pub.publish(self.msg)

            self.rate.sleep()

    def msg_template(self):
        msg = Imu()
        msg.header.frame_id = "robocape"

        msg.linear_acceleration_covariance = (0.1, 0.0, 0.0,
                                              0.0, 0.1, 0.0,
                                              0.0, 0.0, 0.1)

        msg.angular_velocity_covariance = (1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0)

        msg.orientation_covariance = (10.0, 0.0,  0.0,
                                      0.0,  10.0, 0.0,
                                      0.0,  0.0,  10.0)

        return msg

    def pack_message(self):
        now = rospy.get_rostime()
        self.msg.header.stamp.secs = now.secs
        self.msg.header.stamp.nsecs = now.nsecs

        self.msg.linear_acceleration.x = self.imu.accel[0]
        self.msg.linear_acceleration.y = self.imu.accel[1]
        self.msg.linear_acceleration.z = self.imu.accel[2]

        self.msg.angular_velocity.x = self.imu.gyro[0]
        self.msg.angular_velocity.y = self.imu.gyro[1]
        self.msg.angular_velocity.z = self.imu.gyro[2]

        quaternion = tf.transformations.quaternion_from_euler(self.imu.compass[0],
                                                              self.imu.compass[1],
                                                              self.imu.compass[2])
        self.msg.orientation.x = quaternion[0]
        self.msg.orientation.y = quaternion[1]
        self.msg.orientation.z = quaternion[2]
        self.msg.orientation.w = quaternion[3]


if __name__ == '__main__':
    try:
        imu = ImuHandler()
        imu.publish()
    except rospy.ROSInterruptException:
        pass
