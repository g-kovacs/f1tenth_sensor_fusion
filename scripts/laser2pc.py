#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/cloud", PointCloud2, queue_size=100)
        self.laserSub = rospy.Subscriber(
            "/scan", LaserScan, self.laserCallback)

    def laserCallback(self, data):
        out = self.laserProj.projectLaser(data)
        self.pcPub.publish(out)


if __name__ == "__main__":
    rospy.init_node("laser2PC")
    node = Laser2PC()
    rospy.spin()
