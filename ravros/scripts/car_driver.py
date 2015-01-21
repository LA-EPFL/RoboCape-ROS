#!/usr/bin/env python
# Filename: car_driver.py
# Author: Salah-Eddine Missri - missrisalaheddine@gmail.com
#
# ROS node that drives a car by listening to command signals

import rospy
from geometry_msgs.msg import Twist

def callback(data):
    global steeringServo, forwardMotor
    steeringServo.set_angle(data.angular.z)
    forwardMotor.set_speed(data.linear.x)

def listener():
    rospy.init_node('car_driver', anonymous=True)
    rospy.Subscriber("car_commands", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # Initialise actuators
    global steeringServo, forwardMotor
    steeringServo = actuators.Servo(channel=actuators.PWM_3, freq=50.0, \
                                    duty_min=5.0, duty_max=10.0, \
                                    angle_min=-45.0, angle_max=45.0)
    forwardMotor = actuators.Motor(channel=actuators.PWM_1, freq=50.0, \
                                   duty_min=5.0, duty_max=10.0, \
                                   speed_min=-50.0, speed_max=50.0)
    steeringServo.set_angle(0.0)
    forwardMotor.set_speed(0.0)
    steeringServo.display_servo()
    forwardMotor.display_motor()
    print "Actuators: Check."

    # Launch listener
    listener()
