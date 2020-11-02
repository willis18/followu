#!usr/bin/env python
# _*_ coding: utf-8 _*_
#Translaters from NavSatFix to GPSfix and back
import rospy
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg import GPSMessage

wecar_pub = rospy.Publisher("fix", NavSatFix, queue_size= 10)
def wecar_callback(wecar_msg):
    navsat_msg = NavSatFix()
    navsat_msg.header.frame_id = 'map'
    navsat_msg.header.stamp = rospy.Time.now()
    navsat_msg.latitude = wecar_msg.latitude
    navsat_msg.longitude = wecar_msg.longitude
    navsat_msg.altitude = wecar_msg.altitude
    wecar_pub.publish(navsat_msg)

if __name__ == '__main__':
    rospy.init_node('GPSMessage_translator', anonymous=True)
    wecar_sub = rospy.Subscriber("gps", GPSMessage, wecar_callback)
    rospy.spin()
