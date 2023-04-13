#!/usr/bin/env python                                                                        
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CameraInit:
	def __init__(self):
		self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.imageCallback)
		self.image_pub = rospy.Publisher("/ground_img", Image, queue_size=1)
		self.bridge = CvBridge()
	
	
	def imageCallback(self, msg):
		try:
			raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
		except CvBridgeError as e:
			print(e)
		
		raw_im_size = np.shape(raw_image)
		raw_image = raw_image[:, 0:int(raw_im_size[1]/2)-1, :] #left side of raw image is the image looking downward
		
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(raw_image,"bgr8"))
		return


def main():
	rospy.init_node('camera_init', anonymous=True)
	ci = CameraInit()
	try:
		rospy.spin()
	except rospy.Exceptions.ROSException:
		print("Exception!")


if __name__ == '__main__':
	main()
