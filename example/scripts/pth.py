#!/usr/bin/env python
# _*_ coding: utf-8 _*_
#k시티 맵뿌리기
import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud, Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import csv
import glob

class path_pub:

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        self.path_pub = rospy.Publisher('/pc', PointCloud, queue_size =1)
        self.path_msg = PointCloud()
        self.path_msg.header.frame_id ='/map'

        northOffset = 4122635.537
        eastOffset = 302459.942

        file_list = glob.glob('/home/pc/catkin_ws/src/example/kcity_PM0138/'+"A1LANE_*")
        for file in file_list:
            f = open(file, 'r')
            rdr = csv.reader(f, delimiter = '\t')
            for i in range(8):
                next(rdr)

            for line in rdr:
                print(line)
                read_pose = Point32()
                read_pose.x = float(line[0]) - eastOffset
                read_pose.y = float(line[1]) - northOffset
                read_pose.z = 0
                self.path_msg.points.append(read_pose)
        f.close()
     
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.path_pub.publish(self.path_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        test_track=path_pub()
    except rospy.ROSInterruptException:
        pass