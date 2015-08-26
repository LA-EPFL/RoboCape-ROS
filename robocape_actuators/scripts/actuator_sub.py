#!/usr/bin/env python
from __future__ import division
import sys
import rospy
from std_msgs.msg import Float64
from actuator import Actuator

class ActuatorHandler:
    def __init__(self, name):
        self.name = name
        node_name = 'actuator_{}_handler'.format(self.name)
        rospy.init_node(node_name)
        rospy.loginfo(rospy.get_caller_id() + 'Initialising {} node'.format(node_name))

        # Get all parameters from config (rosparam)
        output_pin = int(rospy.get_param('actuators/' + name + '/output_pin', 1))
        board_pin = int(rospy.get_param('actuators/' + name + '/board_pin', 13))
        period_us = int(1e6 / float(rospy.get_param('actuators/' + name + '/frequency', 50)))
        duty_min = float(rospy.get_param('actuators/' + name + '/duty/min', 0.05))
        duty_max = float(rospy.get_param('actuators/' + name + '/duty/max', 0.10))
        output_min = float(rospy.get_param('actuators/' + name + '/output/min', -1.0))
        output_max = float(rospy.get_param('actuators/' + name + '/output/max', 1.0))

        self.actuator = Actuator(board_pin, period_us, duty_min, duty_max, output_min, output_max)

    def subscribe(self):
        topic_name = 'actuator_{}_update'.format(self.name)
        rospy.Subscriber(topic_name, Float64, self.callback)

        rospy.spin()

    def callback(self, msg):
        self.actuator.output = msg.data
        rospy.loginfo(rospy.get_caller_id()
                      + "actuator {} output updated to {:.3f}".format(self.name, self.actuator.output))

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: actuator actuator_name")
    else:
        try:
            actuator = ActuatorHandler(sys.argv[1])
            actuator.subscribe()
        except rospy.ROSInterruptException:
            pass
