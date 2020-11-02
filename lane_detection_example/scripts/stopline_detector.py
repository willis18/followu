#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from utils import BEVTransform, STOPLineEstimator

class IMGParser:
	def __init__(self):
		self.img = None
		self.set_cam(1)
	
	def set_cam(self, _index):
		self.cam = cv2.VideoCapture(int(_index))

	def get_image(self):
		ret, img = self.cam.read()
		return ret, img

	def get_bi_img(self):
		ret, img_bgr =self.get_image()

		img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
		
		lower_wlane = np.array([75,0,220])
		upper_wlane =np.array([175,20,255])
		img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

		#self.img_wlane[int(0.7*img_hsv.shape[0]):, :] =0  (delete another line except stopline)

		return img_wlane
		

if __name__ == '__main__':
	rp= rospkg.RosPack()

	currentPath = rp.get_path("lane_detection_example")

	with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
		sensor_params = json.load(fp)

	params_cam = sensor_params["params_cam"]

	rospy.init_node('image_parser',  anonymous=True)
	
	image_parser = IMGParser()
	bev_op = BEVTransform(params_cam=params_cam)

	sline_detector = STOPLineEstimator()

	rate = rospy.Rate(30)

	while not rospy.is_shutdown():
		if image_parser.img_wlane is not None:
			#img_warp= bev_op.warp_bev_img(image_parser.img_wlane)
			lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

			sline_detector.get_x_points(lane_pts)
			sline_detector.estimate_dist(0.3)
			
			#sline_detector.visualize_dist()

			sline_detector.pub_sline()

			#cv2.imshow("Image window", img_warp1)
			#cv2.waitKey(1)
	
			rate.sleep()




