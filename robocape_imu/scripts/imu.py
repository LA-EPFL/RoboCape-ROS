#!/usr/bin/env python
from __future__ import division
import math
import rospy
import tf
from sensor_msgs.msg import Imu
import mraa

### MPU 9150 Register Map

MPU9150_SELF_TEST_X        = 0x0D   # R/W
MPU9150_SELF_TEST_Y        = 0x0E   # R/W
MPU9150_SELF_TEST_X        = 0x0F   # R/W
MPU9150_SELF_TEST_A        = 0x10   # R/W
MPU9150_SMPLRT_DIV         = 0x19   # R/W
MPU9150_CONFIG             = 0x1A   # R/W
MPU9150_GYRO_CONFIG        = 0x1B   # R/W
MPU9150_ACCEL_CONFIG       = 0x1C   # R/W
MPU9150_FF_THR             = 0x1D   # R/W
MPU9150_FF_DUR             = 0x1E   # R/W
MPU9150_MOT_THR            = 0x1F   # R/W
MPU9150_MOT_DUR            = 0x20   # R/W
MPU9150_ZRMOT_THR          = 0x21   # R/W
MPU9150_ZRMOT_DUR          = 0x22   # R/W
MPU9150_FIFO_EN            = 0x23   # R/W
MPU9150_I2C_MST_CTRL       = 0x24   # R/W
MPU9150_I2C_SLV0_ADDR      = 0x25   # R/W
MPU9150_I2C_SLV0_REG       = 0x26   # R/W
MPU9150_I2C_SLV0_CTRL      = 0x27   # R/W
MPU9150_I2C_SLV1_ADDR      = 0x28   # R/W
MPU9150_I2C_SLV1_REG       = 0x29   # R/W
MPU9150_I2C_SLV1_CTRL      = 0x2A   # R/W
MPU9150_I2C_SLV2_ADDR      = 0x2B   # R/W
MPU9150_I2C_SLV2_REG       = 0x2C   # R/W
MPU9150_I2C_SLV2_CTRL      = 0x2D   # R/W
MPU9150_I2C_SLV3_ADDR      = 0x2E   # R/W
MPU9150_I2C_SLV3_REG       = 0x2F   # R/W
MPU9150_I2C_SLV3_CTRL      = 0x30   # R/W
MPU9150_I2C_SLV4_ADDR      = 0x31   # R/W
MPU9150_I2C_SLV4_REG       = 0x32   # R/W
MPU9150_I2C_SLV4_DO        = 0x33   # R/W
MPU9150_I2C_SLV4_CTRL      = 0x34   # R/W
MPU9150_I2C_SLV4_DI        = 0x35   # R
MPU9150_I2C_MST_STATUS     = 0x36   # R
MPU9150_INT_PIN_CFG        = 0x37   # R/W
MPU9150_INT_ENABLE         = 0x38   # R/W
MPU9150_INT_STATUS         = 0x3A   # R
MPU9150_ACCEL_XOUT_H       = 0x3B   # R
MPU9150_ACCEL_XOUT_L       = 0x3C   # R
MPU9150_ACCEL_YOUT_H       = 0x3D   # R
MPU9150_ACCEL_YOUT_L       = 0x3E   # R
MPU9150_ACCEL_ZOUT_H       = 0x3F   # R
MPU9150_ACCEL_ZOUT_L       = 0x40   # R
MPU9150_TEMP_OUT_H         = 0x41   # R
MPU9150_TEMP_OUT_L         = 0x42   # R
MPU9150_GYRO_XOUT_H        = 0x43   # R
MPU9150_GYRO_XOUT_L        = 0x44   # R
MPU9150_GYRO_YOUT_H        = 0x45   # R
MPU9150_GYRO_YOUT_L        = 0x46   # R
MPU9150_GYRO_ZOUT_H        = 0x47   # R
MPU9150_GYRO_ZOUT_L        = 0x48   # R
MPU9150_EXT_SENS_DATA_00   = 0x49   # R
MPU9150_EXT_SENS_DATA_01   = 0x4A   # R
MPU9150_EXT_SENS_DATA_02   = 0x4B   # R
MPU9150_EXT_SENS_DATA_03   = 0x4C   # R
MPU9150_EXT_SENS_DATA_04   = 0x4D   # R
MPU9150_EXT_SENS_DATA_05   = 0x4E   # R
MPU9150_EXT_SENS_DATA_06   = 0x4F   # R
MPU9150_EXT_SENS_DATA_07   = 0x50   # R
MPU9150_EXT_SENS_DATA_08   = 0x51   # R
MPU9150_EXT_SENS_DATA_09   = 0x52   # R
MPU9150_EXT_SENS_DATA_10   = 0x53   # R
MPU9150_EXT_SENS_DATA_11   = 0x54   # R
MPU9150_EXT_SENS_DATA_12   = 0x55   # R
MPU9150_EXT_SENS_DATA_13   = 0x56   # R
MPU9150_EXT_SENS_DATA_14   = 0x57   # R
MPU9150_EXT_SENS_DATA_15   = 0x58   # R
MPU9150_EXT_SENS_DATA_16   = 0x59   # R
MPU9150_EXT_SENS_DATA_17   = 0x5A   # R
MPU9150_EXT_SENS_DATA_18   = 0x5B   # R
MPU9150_EXT_SENS_DATA_19   = 0x5C   # R
MPU9150_EXT_SENS_DATA_20   = 0x5D   # R
MPU9150_EXT_SENS_DATA_21   = 0x5E   # R
MPU9150_EXT_SENS_DATA_22   = 0x5F   # R
MPU9150_EXT_SENS_DATA_23   = 0x60   # R
MPU9150_MOT_DETECT_STATUS  = 0x61   # R
MPU9150_I2C_SLV0_DO        = 0x63   # R/W
MPU9150_I2C_SLV1_DO        = 0x64   # R/W
MPU9150_I2C_SLV2_DO        = 0x65   # R/W
MPU9150_I2C_SLV3_DO        = 0x66   # R/W
MPU9150_I2C_MST_DELAY_CTRL = 0x67   # R/W
MPU9150_SIGNAL_PATH_RESET  = 0x68   # R/W
MPU9150_MOT_DETECT_CTRL    = 0x69   # R/W
MPU9150_USER_CTRL          = 0x6A   # R/W
MPU9150_PWR_MGMT_1         = 0x6B   # R/W
MPU9150_PWR_MGMT_2         = 0x6C   # R/W
MPU9150_FIFO_COUNTH        = 0x72   # R/W
MPU9150_FIFO_COUNTL        = 0x73   # R/W
MPU9150_FIFO_R_W           = 0x74   # R/W
MPU9150_WHO_AM_I           = 0x75   # R

# MPU9150 Compass
MPU9150_CMPS_XOUT_L        = 0x4A   # R
MPU9150_CMPS_XOUT_H        = 0x4B   # R
MPU9150_CMPS_YOUT_L        = 0x4C   # R
MPU9150_CMPS_YOUT_H        = 0x4D   # R
MPU9150_CMPS_ZOUT_L        = 0x4E   # R
MPU9150_CMPS_ZOUT_H        = 0x4F   # R

# Default I2C address of IMU
MPU9150_I2C_ADDR_LOW       = 0x68
MPU9150_I2C_ADDR_HIGH      = 0x69
MPU9150_I2C_ADDR_DEFAULT   = MPU9150_I2C_ADDR_LOW


class Mpu9150:
    """Defines a sensor object of type 9-DOF IMU"""
    def __init__(self, address=MPU9150_I2C_ADDR_DEFAULT, gyro_scale=250.0, accel_scale=2.0):
        self.address = address         # 0x68 or 0x69
        self.gyro_scale = gyro_scale   # 250, 500, 1000 or 2000 deg/s
        self.accel_scale = accel_scale # 2, 4, 8 or 16 g
        self.compass_scale = 1200.0    # 1200 uT

        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.compass = [0.0, 0.0, 0.0]
        self.temp = 0.0

        if (self.address is not MPU9150_I2C_ADDR_LOW) and (self.address is not MPU9150_I2C_ADDR_HIGH):
            self.address = MPU9150_I2C_ADDR_DEFAULT

        self.dev = mraa.I2c(1)
        self.dev.address(self.address)
        self.dev.frequency(mraa.I2C_FAST)

        self.setup_conf()

        # Compute scaling factors
        self.gyro_scaling = (65536.0 / (2 * gyro_scale)) * (180.0 / math.pi)
        self.accel_scaling = 65536.0 / (2 * accel_scale * 9.81)
        self.compass_scaling = 4000.0 / self.compass_scale

    def setup_conf(self):
        'Runs boot up configuration to wake up IMU and sets options as wanted'
        # Turn ON
        self.dev.writeReg(MPU9150_PWR_MGMT_1, 0x01)  # Power up
        self.dev.writeReg(MPU9150_INT_PIN_CFG, 0x00)
        self.dev.writeReg(MPU9150_USER_CTRL, 0x00)

        # Configure Gyroscope and Accelerometer
        self.dev.writeReg(MPU9150_GYRO_CONFIG, 0x00)
        self.dev.writeReg(MPU9150_ACCEL_CONFIG, 0x00)
        self.dev.writeReg(MPU9150_SMPLRT_DIV, 0x0A)  # Set sample rate at 100Hz
        # Setup low pass filter
        self.dev.writeReg(MPU9150_CONFIG, 0x00)
        self.dev.writeReg(MPU9150_CONFIG, 0x05)  # Set lowpass response at 10Hz
        # Set Gyroscope update rate at 1kHz (same as accelerometer)
        self.dev.writeReg(MPU9150_SMPLRT_DIV, 0x07)

        # Boot compass & configure it
        self.dev.writeReg(0x0A,0x00)
        self.dev.writeReg(0x0A,0x0F) # Self-test
        self.dev.writeReg(0x0A,0x00)

        # Configure i2c communication/reading
        self.dev.writeReg(MPU9150_I2C_MST_CTRL,0x40)  # Wait for Data at slave0
        self.dev.writeReg(MPU9150_I2C_SLV0_ADDR,0x8C) # Set slave0 i2c addr at 0x0C
        self.dev.writeReg(MPU9150_I2C_SLV0_REG,0x02)  # Set reading start at slave0
        self.dev.writeReg(MPU9150_I2C_SLV0_CTRL,0x88) # Enable & set offset
        self.dev.writeReg(MPU9150_I2C_SLV1_ADDR,0x0C) # Set slave1 i2c addr at 0x0C
        self.dev.writeReg(MPU9150_I2C_SLV1_REG,0x0A)  # Set reading start at slave1
        self.dev.writeReg(MPU9150_I2C_SLV1_CTRL,0x81) # Enable at set length to 1
        self.dev.writeReg(MPU9150_I2C_SLV1_DO,0x01)
        self.dev.writeReg(MPU9150_I2C_MST_DELAY_CTRL,0x03) # Set delay rate
        self.dev.writeReg(0x01,0x80)                   # Set i2c slave4 delay
        self.dev.writeReg(MPU9150_I2C_SLV4_CTRL,0x04)
        self.dev.writeReg(MPU9150_I2C_SLV1_DO,0x00)    # Clear user settings
        self.dev.writeReg(MPU9150_USER_CTRL,0x00)
        self.dev.writeReg(MPU9150_I2C_SLV1_DO,0x01)    # Override register
        self.dev.writeReg(MPU9150_USER_CTRL,0x20)      # Enable master i2c mode
        self.dev.writeReg(MPU9150_I2C_SLV4_CTRL,0x13)  # Disable slv4

        # Set Gyroscope scale
        if self.gyro_scale <= 250.0:
            self.gyro_scale = 250
            self.dev.writeReg(MPU9150_GYRO_CONFIG, 0x00)
        elif self.gyro_scale <= 500.0:
            self.gyro_scale = 500
            self.dev.writeReg(MPU9150_GYRO_CONFIG, 0x08)
        elif self.gyro_scale <= 1000.0:
            self.gyro_scale = 1000
            self.dev.writeReg(MPU9150_GYRO_CONFIG, 0x10)
        else:
            self.gyro_scale = 2000
            self.dev.writeReg(MPU9150_GYRO_CONFIG, 0x18)

        # Set Accelerometer scale
        if self.accel_scale <= 2:
            self.accel_scale = 2
            self.dev.writeReg(MPU9150_ACCEL_CONFIG, 0x00)
        elif self.accel_scale <= 4:
            self.accel_scale = 4
            self.dev.writeReg(MPU9150_ACCEL_CONFIG, 0x08)
        elif self.accel_scale <= 8:
            self.accel_scale = 8
            self.dev.writeReg(MPU9150_ACCEL_CONFIG, 0x10)
        else:
            self.accel_scale = 16
            self.dev.writeReg(MPU9150_ACCEL_CONFIG, 0x18)

    def display_info(self):
        'Show sensor raw data from IMU'
        self.read()
        print "Acceleration vector (in m/s^2):"
        print "x: %.5f, y: %.5f, z: %.5f" \
                % (self.accel[0], self.accel[1], self.accel[2])
        print "Angular velocity vector (in rad/s):"
        print "x: %.5f, y: %.5f, z: %.5f" \
                % (self.gyro[0], self.gyro[1], self.gyro[2])
        print "Compass heading vector (in uT):"
        print "x: %.1f, y: %.1f, z: %.1f" \
                % (self.compass[0], self.compass[1], self.compass[2])
        print "Temperature (in C): %f" % self.temp

    def read_word(self, reg_h, reg_l):
        'Reads data from high & low registers and returns the combination'
        # Read register values
        val_h = self.dev.readReg(reg_h)
        val_l = self.dev.readReg(reg_l)

        if val_h > 127:
            return (((val_h - 256) << 8) - val_l)
        else:
            return ((val_h << 8) + val_l)

    def read(self):
        'Get all sensors data'
        self.accel[0] = self.read_word(MPU9150_ACCEL_XOUT_H, MPU9150_ACCEL_XOUT_L) / self.accel_scaling
        self.accel[1] = self.read_word(MPU9150_ACCEL_YOUT_H, MPU9150_ACCEL_YOUT_L) / self.accel_scaling
        self.accel[2] = self.read_word(MPU9150_ACCEL_ZOUT_H, MPU9150_ACCEL_ZOUT_L) / self.accel_scaling

        self.gyro[0] = self.read_word(MPU9150_GYRO_XOUT_H, MPU9150_GYRO_XOUT_L) / self.gyro_scaling
        self.gyro[1] = self.read_word(MPU9150_GYRO_YOUT_H, MPU9150_GYRO_YOUT_L) / self.gyro_scaling
        self.gyro[2] = self.read_word(MPU9150_GYRO_ZOUT_H, MPU9150_GYRO_ZOUT_L) / self.gyro_scaling

        self.compass[0] = self.read_word(MPU9150_CMPS_XOUT_H, MPU9150_CMPS_XOUT_L) / self.compass_scaling
        self.compass[1] = self.read_word(MPU9150_CMPS_YOUT_H, MPU9150_CMPS_YOUT_L) / self.compass_scaling
        self.compass[2] = self.read_word(MPU9150_CMPS_ZOUT_H, MPU9150_CMPS_ZOUT_L) / self.compass_scaling

        self.temp = self.read_word(MPU9150_TEMP_OUT_H, MPU9150_TEMP_OUT_L) / 340.0 + 35

    def get_compass_normalised(self):
        'Returns compass measurement normalised'
        norm = (self.compass[0]**2 + self.compass[1]**2 + self.compass[2]**2)**0.5
        return self.compass[0] / norm, self.compass[1] / norm, self.compass[2] / norm


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

        while not rospy.is_shutdown():
            rospy.loginfo(rospy.get_caller_id() + "publishing IMU data")

            self.imu.read()
            msg = self.pack_message()
            self.pub.publish(msg)

            self.rate.sleep()

    def pack_message(self):
        msg = Imu()

        now = rospy.get_rostime()
        msg.header.stamp.secs = now.secs
        msg.header.stamp.nsecs = now.nsecs
        msg.header.frame_id = "robocape"

        msg.linear_acceleration.x = self.imu.accel[0]
        msg.linear_acceleration.y = self.imu.accel[1]
        msg.linear_acceleration.z = self.imu.accel[2]
        msg.linear_acceleration_covariance = 0.01 * (1.0, 0.0, 0.0,
                                                     0.0, 1.0, 0.0,
                                                     0.0, 0.0, 1.0)

        msg.angular_velocity.x = self.imu.gyro[0]
        msg.angular_velocity.y = self.imu.gyro[1]
        msg.angular_velocity.z = self.imu.gyro[2]
        msg.angular_velocity_covariance = (1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0)

        quaternion = tf.transformations.quaternion_from_euler(self.imu.compass[0],
                                                              self.imu.compass[1],
                                                              self.imu.compass[2])
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]
        msg.orientation_covariance = 10 * (1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0)

        return msg

if __name__ == '__main__':
    try:
        imu = ImuHandler()
        imu.publish()
    except rospy.ROSInterruptException:
        pass
