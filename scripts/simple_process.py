#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def img_proc_callback(raw_img):
	print "Hello"


if __name__ == '__main__':
    rospy.init_node('simple_img_proc', anonymous=True)
    img_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_proc_callback, queue_size = 10)

    while not rospy.is_shutdown():
        rospy.spin()