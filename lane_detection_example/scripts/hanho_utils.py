import numpy as np
import cv2
import math
import rospy
from std_msgs.msg import Float64,String
from sensor_msgs.msg import LaserScan,PointCloud
from math import cos,sin,pi

from sklearn import linear_model
import random

import matplotlib.pyplot as plt

from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped, Point32



class purePursuit:
	def __init__(self):
		
		self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)

		self.position_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

	def pub_cmd(self, control_speed, steer_lv):
		#Servo Steer Level: 0.15(L) - 0.5304(C) - 0.85(R)		
		self.position_pub.publish(steer_lv)
		self.speed_pub.publish(control_speed)


class simple_controller :

	def __init__(self):		
		self.simple_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		self.msg = None
		
	def laser_callback(self, msg):		
		self.msg = msg

	def front_end(self):
		temp = 1000
		for theta,r in enumerate(self.msg.ranges):
			if 170<theta and theta<190:
				if r<0.3:
					temp = 0					
		return temp
					
	def corner(self, steer):

		for theta, r in enumerate(self.msg.ranges):
			if steer == 0.8:
				if 90<=theta and theta<=170:
					if r<0.4:
						temp = 0.2
						return temp										
					else :
						temp = 0.8
						
			elif steer == 0.2:
				if 190<=theta and theta<=270:
					if r<0.4:
						temp = 0.8
						return temp						
					else :
						temp = 0.2

			elif steer == 0.5304:
				if 90<=theta and theta<=170:
					if r<0.4:
						temp = 0.2
						return temp
				elif 190<=theta and theta<=270:
					if r<0.4:
						temp = 0.8
						return temp				
				else:
					temp = 0.5304

		return temp
						