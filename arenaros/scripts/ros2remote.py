#!/usr/bin/env python
import socket
import struct
import time
import numpy as np
import rospy
from geometry_msgs.msg import Vector3

def callback(data):
    x = [data.x, data.y, data.z]
    t = tuple(x)
    msg = struct.pack('>' + 'd' * len(x),*t)
    sock.sendto(msg, (IP, MATLAB_PORT))

    rospy.loginfo(rospy.get_caller_id() + "I heard\n %f,\n %f,\n %f" % (data.x, data.y, data.z))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("imu", Vector3, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    IP = "127.0.0.1"
    MY_PORT = 20142
    MATLAB_PORT = 20105
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((IP, MY_PORT))
    print 'Ready!'

    listener()
    sock.close()

