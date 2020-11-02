#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import rospkg

from std_msgs.msg import Float64
from math import cos, sin, pi, sqrt, pow,atan2
from geometry_msgs.msg import Point32, PoseStamped, Point, PoseWithCovarianceStamped

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class pure_pursuit:
	def __init__(self):
    		
		rospy.init_node('make_path', anonymous=True)
		
		rospy.Subscriber("/offset", Float64, self.offset_callback)

		self.motor_pub = rospy.Publisher('commands/motor/speed', Float64, queue_size=1)
		self.servo_pub = rospy.Publisher('commands/servo/position', Float64, queue_size=1)
		
		self.motor_msg=Float64()
		self.servo_msg=Float64()

		self.is_offset=False
		self.offset = 0

		self.steering=0
		self.servo_gain = -0.013
		self.steering_angle_to_servo_offset=0.5304

		rate = rospy.Rate(10)
	
		while not rospy.is_shutdown():
			if self.is_offset ==True :

				if self.offset == 0:
					self.steering = 0 
				else:
					self.steering = self.offset / 7
					if self.steering >=20:
						self.steering = 22
					elif self.steering <= -20:
						self.steering = -22

				self.motor_msg.data=1000
				
				self.steering_command=(self.steering * self.servo_gain)+self.steering_angle_to_servo_offset
				self.servo_msg.data=self.steering_command

				print(self.steering, self.steering_command)			
				self.servo_pub.publish(self.servo_msg)
				self.motor_pub.publish(self.motor_msg)

			rate.sleep()

	def offset_callback(self,msg):
		self.is_offset=True
		self.offset=msg.data

if __name__ == '__main__':
	try:
		test_track=pure_pursuit()
	except rospy.ROSInterruptException:
		pass






