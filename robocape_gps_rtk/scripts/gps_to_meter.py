#!/usr/bin/env python
from __future__ import division
import rospy
import argparse
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D

class PositionPublisher(object):
    def __init__(self, origin_lon, origin_lat):
        rospy.init_node('position_publisher')

        self.origin_lon = origin_lon
        self.origin_lat = origin_lat

        self.sub = rospy.Subscriber('fix', NavSatFix, self.callback)
        self.pub = rospy.Publisher('position', Pose2D, queue_size=1)

        self.msg = Pose2D()

        rospy.spin()

    def haversine(self, lon1, lat1, lon2, lat2):
        '''Calculate the great circle distance between two points (specified in decimal degrees)'''
        # convert decimal degrees to radians
        lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])
        # haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
        c = 2 * np.arcsin(np.sqrt(a))
        meters = 6371009 * c
        return meters

    def gps_to_meter(self, lon1, lat1, lon2, lat2):
        lon_meter = haversine(lon1, 0, lon2, 0)
        lat_meter = haversine(0, lat1, 0, lat2)

        if(lon2 < lon1):
            lon_meter = - lon_meter
        if(lat2 < lat1):
            lat_meter = - lat_meter

        return (lon_meter, lat_meter)

    def callback(self, data):
        lon = data.longitude
        lat = data.latitude

        (x, y) = self.gps_to_meter(self.origin_lon, self.origin_lat, lon, lat)

        self.msg.x = x
        self.msg.y = y
        self.msg.heading = 0

        self.pub.publish(self.msg)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert GPS fix data to meters position')

    parser.add_argument('longitude', type=float, help='Longitude of the origin point')
    parser.add_argument('latitude', type=float, help='Latitude of the origin point')

    args = parser.parse_args()

    try:
        position_pub = PositionPublisher()
    except rospy.ROSInterruptException:
        pass

