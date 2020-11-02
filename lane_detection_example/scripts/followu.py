#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy, cv2
import numpy as np
import os, rospkg, json, time
from threading import Thread

from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount

from hanho_utils import purePursuit, simple_controller

class IMGParser:
    def __init__(self, currentPath):
        self.img = None
        #region fixed code cam set
        self.cameraFeed = True

        self.cameraNo = 1
        self.cameraWidth = 640
        self.cameraHeight = 480
        self.frameWidth = 640
        self.frameHeight = 480
        #endregion
        self.set_cam()
        self.saving_ret = False
        self.saving_img = None
        self.keep_reading = True

        self.yolo_feed = False
        self.obj_feed = False
        self.yolo_time = 0.0
        self.obj_count = 0
        self.xmid = 0
        self.dst_steer_level = 0.5304

        self.yolo_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_callback)
        self.obj_sub = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.obj_callback)
        self.img_pub = rospy.Publisher("/detector_img", Image, queue_size=1)
        self.img_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.img_callback)

    def yolo_callback(self, msg):        
        if self.obj_feed:
            boxes = msg.bounding_boxes
            self.yolo_feed = True
            self.xmid = ((boxes[0].xmin+boxes[0].xmax) / 2)
            if self.xmid < 270 :
                self.dst_steer_level = 0.2
            elif self.xmid >370:
                self.dst_steer_level = 0.8
            else : 
                self.dst_steer_level = 0.5304 
            
        else: self.yolo_feed = False     

    def turn(self):
        
        return self.dst_steer_level , self.xmid

    def img_callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        
    def obj_callback(self, msg):
        self.obj_count = int(msg.count)
        if self.obj_count > 0:
            self.obj_feed = True
        else: 
            self.obj_feed = False
            self.stimer = True

    def pub_img(self, img):
        pub_img = Image()
        pub_img.header.stamp = rospy.Time.now()
        pub_img.height = self.cameraHeight
        pub_img.width = self.cameraWidth
        pub_img.data = img.tostring()
        self.img_pub.publish(pub_img)

    def set_cam(self):
        if self.cameraFeed:
            self.cam = cv2.VideoCapture(self.cameraNo)
            self.cam.set(3, self.cameraWidth)
            self.cam.set(4, self.cameraHeight)
            self.cam.set(28, 0)
        else:
            self.cam = cv2.VideoCapture(self.videoPath)
        
    def get_image(self):
        ret, image = self.cam.read()
      

        return ret, image
    
    def get_image_continue(self):
        while(self.keep_reading):
            self.saving_ret, self.saving_img = self.get_image()
            time.sleep(0.03)
        
    

if __name__ == '__main__':
    rp= rospkg.RosPack()
    currentPath = rp.get_path("lane_detection_example")
    rospy.init_node('lane_detector',  anonymous=True)

    
    ctrl_lat = purePursuit(lfd=0.8)
    stop_lidar = simple_controller()
    image_parser = IMGParser(currentPath)
    check_turn = 0
    dst_steer_level = 0.5304
   
    rate = rospy.Rate(20) #20hz

    while not rospy.is_shutdown():
        curTime = time.time()
        
        ret, img_wlane = image_parser.get_image()
        
            
            #Servo Steer Level: 0.15(L) - 0.5304(C) - 0.85(R)
           
        if image_parser.obj_feed :       
            set_speed = stop_lidar.front_end()    
            dst_steer_level , xmid = image_parser.turn() 
            if dst_steer_level == 0.2 or dst_steer_level == 0.8:
                check_turn = 1
                temp_steer = dst_steer_level
            else :
                check_turn = 0
            dst_steer_level = stop_lidar.corner(dst_steer_level)
        else :
            set_speed = stop_lidar.front_end() 
            if check_turn == 0:
                print("target missing")
                set_speed = 0
            else :
                dst_steer_level = stop_lidar.corner(temp_steer)
           
        speed = set_speed
      
        print("speed", speed, "steer", dst_steer_level)
        ctrl_lat.pub_cmd(speed, dst_steer_level)     
       
        
        if cv2.waitKey(1) == 13:
            image_parser.keep_reading = False
            break
        rate.sleep()

    image_parser.keep_reading = False
