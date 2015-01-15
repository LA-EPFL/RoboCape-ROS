#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3

def talker():
    pub = rospy.Publisher('chatter', Vector3, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        vector = Vector3()
        vector.x = float(rospy.get_time())
        vector.y = float(rospy.get_time())
        vector.z = float(rospy.get_time())
        rospy.loginfo("Vector published")
        pub.publish(vector)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

