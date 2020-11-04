#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy, cv2
import numpy as np
import os, rospkg, json, time
from threading import Thread

from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount

from hanho_utils import purePursuit, simple_controller

class IMGParser:
    def __init__(self, currentPath):        
        self.yolo_feed = False
        self.obj_feed = False
        self.obj_count = 0
        self.xmid = 0
        self.dst_steer_level = 0.5304

        self.yolo_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_callback)
        self.obj_sub = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.obj_callback)

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

    def obj_callback(self, msg):
        self.obj_count = int(msg.count)
        if self.obj_count > 0:
            self.obj_feed = True
        else: 
            self.obj_feed = False


if __name__ == '__main__':
    rp= rospkg.RosPack()
    currentPath = rp.get_path("lane_detection_example")
    rospy.init_node('lane_detector',  anonymous=True)

    stop_lidar = simple_controller()
    time.sleep(2)
    ctrl_lat = purePursuit()
    image_parser = IMGParser(currentPath) 
    
    check_turn = 0
    dst_steer_level = 0.5304
   
    rate = rospy.Rate(20) #20hz

    while not rospy.is_shutdown():          
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

        rate.sleep()

