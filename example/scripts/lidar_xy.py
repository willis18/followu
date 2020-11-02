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

        self.pcd_pub = rospy.Publisher('laser2pcd', PointCloud, queue_size= 1)

        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, msg):
        pcd=PointCloud()
        motor_msg=Float64()
        pcd.header.frame_id = msg.header.frame_id
        angle=0

        for r in msg.ranges:
            tmp_point = Point32()
            tmp_point.x = r*cos(angle)
            tmp_point.y = r*sin(angle)
            angle = angle+(1.0/180*pi)
            print(angle,tmp_point.x, tmp_point.y)
            
            if r<12 :
                pcd.points.append(tmp_point)

        self.pcd_pub.publish(pcd)

if __name__ == '__main__':
    try:
        test_track=simple_controller()
    except rospy.ROSInterruptException:
        pass