#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class simple_controller :

    def __init__(self):
        rospy.init_node("simple_controller", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, msg):
        for theta,r in enumerate(msg.ranges):
            if 170<theta and theta<190:
                # print(theta, r)
                if r<0.4:
                    return 1
                else :
                    return 0

# if __name__ == '__main__':
#     try:
#         test_track=simple_controller()
#     except rospy.ROSInterruptException:
#         pass