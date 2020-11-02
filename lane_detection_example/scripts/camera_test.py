#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

class IMGParser:
	def __init__(self):
		rospy.init_node('camera_test', anonymous=True)	
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback)

		self.offset_pub = rospy.Publisher('/offset', Float64, queue_size=1)
		#self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
		
		self.offset_msg = Float64()
	
		self.img_wlane = None

		self.point_height = None
		self.left_weight= None
		self.right_weight= None

		self.pre_right = 0
		self.pre_left = 0

		self.center_weight = None
		
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			if self.img_wlane is not None:
				self.offset_pub.publish(self.offset_msg)
				
				cv2.circle(self.roi_img, (self.left_weight,self.point_height),3,(255,0,0),10)
				cv2.circle(self.roi_img, (self.right_weight,self.point_height),3,(0,0,255),5)
				cv2.circle(self.roi_img, (self.center_weight,self.point_height),3,(0,255,0),5)

				cv2.imshow('test',self.img_wlane)
				cv2.imshow("camera", self.roi_img)
				cv2.waitKey(1)

			rate.sleep()

	def callback(self, msg):
		np_arr=np.fromstring(msg.data, np.uint8)
		img_bgr=cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		self.roi_img =img_bgr[240:480, 0:640]

		img_gray = cv2.cvtColor(self.roi_img, cv2.COLOR_BGR2GRAY)
		
		ret, self.img_wlane = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
		
		height, weight = self.img_wlane.shape

		point_weight = weight / 2
		self.point_height = height * 3/5

		self.left_weight = point_weight
		self.right_weight = point_weight

		left_check =False
		right_check = False

		
		while left_check==False or right_check==False :
			if self.img_wlane[self.point_height][self.left_weight] != 0 or self.left_weight<=0:
				left_check = True
			else:
				self.left_weight -= 1

			if self.img_wlane[self.point_height][self.right_weight] != 0 or self.right_weight >= weight-1:
				right_check= True
			else:	
				self.right_weight += 1

		if self.right_weight-self.left_weight<200:
			self.right_weight = self.pre_right
			self.left_weight =  self.pre_left
			#print("pre", self.right_weight, self.left_weight)
		else:
			self.pre_right = self.right_weight 
			self.pre_left = self.left_weight

		self.center_weight = (self.right_weight  + self.left_weight) / 2
		self.offset_msg.data = (weight/2) - self.center_weight

if __name__ == '__main__':
	try:
		image_parser = IMGParser()
	except rospy.ROSInterruptException:
		pass



