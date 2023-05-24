#!/usr/bin/env python                                                                        
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from line_detection.cfg import reconfigConfig		# ~/catkin_ws/devel/lib/python3/dist-packages/line_detection/cfg/
import time



class ImageConverter:
	def __init__(self,D,K,R,P,S,simulation):
		self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
		self.image_pub = rospy.Publisher("/lane_img", Image, queue_size=1)
		self.bridge = CvBridge()
		self.map1,self.map2 = cv2.fisheye.initUndistortRectifyMap(K,D,R,P,S,cv2.CV_16SC2)
		self.sim = simulation
		self.mask_img = cv2.imread("/home/ubuntu/catkin_ws/src/line_detection/src/mask_robot_image3.png",cv2.IMREAD_GRAYSCALE)
		self.img_size = S
		self.template = np.zeros((9,3), np.uint8); self.template[3:6] = 255
		self.thresh_val1 = 160
		self.thresh_val2 = 0.7
		self.show = False
		self.time_counter = [0.,0]
		self.counter = 0
	
	def callback(self, data):
		#start = time.perf_counter()
		if not hasattr(self, "sim"):
			return
		
		"""
		if self.counter == 0:
			self.counter = 1
		else:
			self.counter = 0
			return
		"""
		
		try:
			raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8") 
		except CvBridgeError as e:
			print(e)
		
		if not self.sim:
			cv2.imshow("/cv_camera/image_raw", cv2.resize(raw_image, dsize=None, fx=0.5, fy=0.5))
			raw_im_size = np.shape(raw_image)
			raw_image = raw_image[:, 0:int(raw_im_size[1]/2)-1, :] #left side of raw image is the image looking downward
			im_size = np.shape(raw_image)
			rotate_mat = cv2.getRotationMatrix2D(center=(int(im_size[0]/2),int(im_size[1]/2)), angle=-90, scale=1)
			raw_image = cv2.warpAffine(raw_image, rotate_mat, im_size[:2])
		
		#-------------Undistortion-------------
		cv_image = cv2.remap(raw_image, self.map1,self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
		
		#-------------Convert to gray scale & resize------------
		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
		output_size = (120,120)
		gray_image = cv2.resize(gray_image, output_size)
		
		#-------------Template matching-------------
		_, thresh_gray_image = cv2.threshold(gray_image, self.thresh_val1, 255, cv2.THRESH_TOZERO) # threshold before template matching
		#thresh_gray_image = cv2.bitwise_and(thresh_gray_image, self.mask_img)
		#-----Noise remove-----
		#kernel = np.full((2,2),0.1,np.float32)
		#thresh_gray_image = cv2.morphologyEx(thresh_gray_image, cv2.MORPH_OPEN, kernel)
		
		template_hor = self.template
		template_ver = template_hor.T # transpose
		res_hor = cv2.matchTemplate(thresh_gray_image, template_hor, eval("cv2.TM_CCORR_NORMED"))
		res_ver = cv2.matchTemplate(thresh_gray_image, template_ver, eval("cv2.TM_CCORR_NORMED"))
		
		_, result_hor = cv2.threshold(res_hor, self.thresh_val2, 1, cv2.THRESH_BINARY)  # threshold after 
		_, result_ver = cv2.threshold(res_ver, self.thresh_val2, 1, cv2.THRESH_BINARY)
		
		#-------------Padding & merge-------------
		up = int(template_hor.shape[0]/2)
		low = output_size[0] - up - result_hor.shape[0]
		left = int(template_hor.shape[1]/2)
		right = output_size[1] - left - result_hor.shape[1]
		result_hor = np.pad(result_hor, [(up,low),(left,right)], "constant")
		
		up = int(template_ver.shape[0]/2)
		low = output_size[0] - up - result_ver.shape[0]
		left = int(template_ver.shape[1]/2)
		right = output_size[1] - left - result_ver.shape[1]
		result_ver = np.pad(result_ver, [(up,low),(left,right)], "constant")
		
		result_img = result_hor + result_ver
		
		#-------------Noise remove---------------
		kernel = np.ones((7,7),np.uint8)
		result_img2 = cv2.morphologyEx(result_img, cv2.MORPH_OPEN, kernel)
		#-------------Dilation-------------------
		kernel = np.ones((2,2),np.uint8)
		result_img = cv2.dilate(result_img, kernel, iterations=1)
		
		#-------------Mask robot & publish images-------------
		result_img = np.array(result_img, dtype="u1")
		result_img = cv2.bitwise_and(result_img, self.mask_img)
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_img, "8UC1"))
		
		"""  
		#-------------Time count-------------
		end = time.perf_counter()
		self.time_counter[0] += + (end-start)
		self.time_counter[1] += 1
		if self.time_counter[1] == 30:
			print("imInit ", self.time_counter[0]/30*1000, "[ms]")
			self.time_counter = [0.,0]
		"""
		
		#-------------Show images-------------
		if self.show:
			each_size = (300,300)
			raw_image = cv2.resize(raw_image, dsize=each_size)
			gray_image = cv2.resize(cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR), dsize=each_size)
			thresh_gray_image = cv2.resize(cv2.cvtColor(thresh_gray_image, cv2.COLOR_GRAY2BGR), dsize=each_size)
			result_img = 255 * cv2.resize(cv2.cvtColor(result_img, cv2.COLOR_GRAY2BGR), dsize=each_size)
			vpartition = np.zeros((each_size[0], 5, 3), dtype="u1")
			vpartition[:,:,0:2] = 255
			
			res_ver = cv2.resize(cv2.cvtColor(np.array(res_ver*255, dtype="u1"), cv2.COLOR_GRAY2BGR), dsize=each_size)
			res_hor = cv2.resize(cv2.cvtColor(np.array(res_hor*255, dtype="u1"), cv2.COLOR_GRAY2BGR), dsize=each_size)
			hpartition = np.zeros((5, each_size[1], 3), dtype="u1")
			hpartition[:,:,0:2] = 255
			ver_hor = cv2.vconcat([res_ver, hpartition, res_hor])
			ver_hor = cv2.resize(ver_hor, dsize=(int(each_size[0]/2), each_size[1]))
			showed_img = cv2.hconcat([raw_image, vpartition, gray_image, vpartition, thresh_gray_image, vpartition, ver_hor, vpartition, result_img])
			cv2.imshow("Raw => Undistorted Gray => Threshold => Template match & Threshold & Mask", showed_img)
			
			"""
			cv2.imshow("Raw Image",cv2.resize(raw_image, dsize=each_size))
			cv2.imshow("Undistorted Gray Image",cv2.resize(gray_image, dsize=each_size))
			cv2.imshow("Undistorted Threshold Gray Image",cv2.resize(thresh_gray_image, dsize=each_size))
			cv2.imshow("Template Matching Result: Vertical", cv2.resize(res_ver, dsize=each_size))
			cv2.imshow("Template Matching Result: Horizontal", cv2.resize(res_hor, dsize=each_size))
			cv2.imshow("Merge & Threshold Image", cv2.resize(result_img, dsize=each_size)*255)
			"""
			
			cv2.waitKey(1)
		else:
			cv2.destroyAllWindows()
			

	def reconfigCallback(self, config, level):
		self.show = config.show_images
		self.thresh_val1 = config.thresh_before_temp_match
		self.thresh_val2 = config.thresh_result_temp_match
		form_str = config.template_format
		form = form_str.split(",")
		form = (int(form[0]), int(form[1]), int(form[2]))
		self.template = np.zeros(form[:2], np.uint8)
		self.template[int((form[0]-form[2])/2):int((form[0]-form[2])/2)+form[2], :] = 255
		return config


def main():
	rospy.init_node('imInit_igvc2022', anonymous=True)
	simulation = rospy.get_param("/sim") #is true when simulation is launched
	if simulation: # for simulation camera
		D = np.float32([[0.08515002883616878, 0.012936405125198296, -0.0018582939071174716, 0.0008693014693496443]])
		K = np.float32([[197.33531703864534, 0.0, 399.5746325526872], [0.0, 197.33723610911758, 399.51648288464474], [0.0,0.0,1.0]])
		P = K
		S = (800,800)
	else: # for real device: insta360_air
		"""
		D = np.float32([[-0.05367925701562622, 0.07140332123124533, -0.08089849852274485, 0.03184938555925065]])
		K = np.float32([[472.4101671967401, 0.5799913144184203, 750.6027871515556], [0.0, 472.753090371875, 753.8213756085806], [0.0,0.0,1.0]])
		P = K
		S = (1504,1504)
		#S = (736,736)
		"""
		D = np.float32([[-0.008173216167374261, -0.05337521624866415, 0.06211795804586437, -0.02542501550578094]])
		K = np.float32([[230.3124073007367, 0.8902268984725723, 366.0547101660642], [0, 230.3903534611963, 367.3569954767625], [0.0,0.0,1.0]])
		P = np.float32([[115.1562036503684, 0.8902268984725723, 366.0547101660642, 0], [0, 115.1951767305981, 367.3569954767625, 0], [0, 0, 1, 0]])
		S = (736,736)
		#subprocess.run(["bash","/home/ubuntu/catkin_ws/src/line_detection/src/init_camera_property.sh"])	# for insta360_air
		
		
	
	R = np.eye(3)
	ic = ImageConverter(D,K,R,P,S,simulation)
	recfg_srv = Server(reconfigConfig, ic.reconfigCallback)
	
	try:
		rospy.spin()
	except rospy.Exceptions.ROSException:
		print("Exception!")

    
    
    
if __name__ == '__main__':
	main()

