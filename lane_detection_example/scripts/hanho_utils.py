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
	def __init__(self, lfd):
		self.is_look_forward_point=False
		self.vehicle_length=0.5

		self.lfd=lfd
		self.min_lfd=0.7
		self.max_lfd=1.2

		self.lpath_sub = rospy.Subscriber('/lane_path',Path, self.lane_path_callback)


		self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)

		self.position_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
		self.lpath = None
	
	def lane_path_callback(self, msg):
		self.lpath = msg

	def steering_angle(self):

		self.is_look_forward_point = False

		for i in self.lpath.poses:

			path_point=i.pose.position

			if path_point.x>0:
				dis_i = np.sqrt(np.square(path_point.x)+np.square(path_point.y))

				if dis_i >= self.lfd:
					self.is_look_forward_point = True
					break
		theta= math.atan2(path_point.y, path_point.x)

		if self.is_look_forward_point:
			steering_deg=math.atan2((2*self.vehicle_length*math.sin(theta)), self.lfd)*180/math.pi

			self.steering=np.clip(steering_deg, -17,17)/34+0.5
			print(self.steering)
		else:
			self.steering=0.5
			print("no found forward point")
	
	def pub_cmd(self, control_speed, steer_lv):
		#self.position_pub.publish(self.steering)
		#Servo Steer Level: 0.15(L) - 0.5304(C) - 0.85(R)
		if steer_lv < 0.15: steer_lv = 0.15
		if steer_lv > 0.85: steer_lv = 0.85
		
		self.position_pub.publish(steer_lv)
		self.speed_pub.publish(control_speed)



class simple_controller :

	def __init__(self):
		# rospy.init_node("simple_controller", anonymous=True)
		rospy.Subscriber("/scan", LaserScan, self.laser_callback)
		# while not rospy.is_shutdown():
		#     rospy.spin()
		


	def laser_callback(self, msg):
		# for theta,r in enumerate(msg.ranges):
		# print(theta, r)
		self.msg = msg

	def front_end(self):
		temp = 1000
		for theta,r in enumerate(self.msg.ranges):
			if 170<theta and theta<190:
				# print(theta, r)
				if r<0.3:
					temp = 0
					
		return temp
					
	def corner(self, steer):
		tempo=0
		for theta, r in enumerate(self.msg.ranges):
			if steer == 0.8:
				if 90<=theta and theta<=170:
					if r<0.4:
						temp = 0.2
						return temp
					# elif 0.4<=r and r<0.5:
					# 	tempo = 1
											
					else :
						temp = 0.8
						
			elif steer == 0.2:
				if 190<=theta and theta<=270:
					if r<0.4:
						temp = 0.8
						return temp
					# elif 0.4<=r and r<0.5:
					# 	tempo= 1
						
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

		# if tempo == 1:
		# 	temp = 0.5304


		return temp
						