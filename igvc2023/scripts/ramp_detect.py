#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import actionlib
import numpy as np
import cv2
import math
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RampDetector:
	def __init__(self):
		self.scan_sub = rospy.Subscriber("/hokuyo_scan", LaserScan, self.scanCallback)
		self.imu_sub = rospy.Subscriber("/imu", Imu, self.imuCallback)
		self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
		self.cmd_pub = rospy.Publisher("/igvc_robot/cmd_vel", Twist, queue_size=1)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.pitch = 0.0
		self.scan_img_size = (150, 299)
		self.center = (self.scan_img_size[0]-1, int(self.scan_img_size[1]/2))
		self.max_range = 6.0 #[m]
		self.scale = (self.scan_img_size[0]-1)/self.max_range
		self.process_id = 0 # 0:approach, 1:climb, 2:down
		#self.latest_range_num = 0
		self.run_counter = 0
		self.run_hz = 30


	#==========Imu callback function==========#
	def imuCallback(self, msg):
		(_, self.pitch, _) = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
		return


	#==========LaserScan callback function==========#
	def scanCallback(self, msg):
		node_flag = rospy.get_param("node_flag", 1)

		#----------Frequency control----------
		self.run_counter += 1
		if self.run_counter != self.run_hz:
			return
		else:
			self.run_counter = 0

		#----------main process----------
		if node_flag == 3: # Only ramp detection. When detected, set node_flag to 4
			local_goal_x, local_goal_y, _ = self.detectRamp(msg)

			if local_goal_x==-1: # can't detect ramp
				return
			elif np.linalg.norm([local_goal_x, local_goal_y]) < self.max_range: #<2.0
				rospy.set_param("node_flag", 4)

		elif node_flag == 4: # approach, climb, and down ramp. When finished, set node_flag to 1]
			if self.process_id==0:
				self.approachToRamp(msg)

			elif self.process_id==1:
				self.climbRamp(0.8)

			elif self.process_id==2:
				self.downRamp(0.5) # in this function, set node_flag to 1

		else:
			return


	#==========Ramp detection (node_flag is 3)==========#
	def detectRamp(self, scan_msg):
		scan_img = self.scan2image(scan_msg) #Convert laserscan data to image
		#----------Hough transform----------
		lines = cv2.HoughLinesP(scan_img, rho=1, theta=math.pi/360, threshold=30, minLineLength=20, maxLineGap=10)
		try:
			x1, y1, x2, y2 = lines[0][0]
		except (ValueError, TypeError):
			rospy.loginfo("Cannot detect line by hough transform")
			cv2.imshow("Scan Image", scan_img)
			cv2.waitKey(1)
			return (-1, -1)

		#----------Find the point front of ramp----------
		#-----x1,y1,x2,y2 are indexs of line end point in scan_img-----
		if (x1-x2)==0:
			return

		if (y2-y1)==0:
			slope_ver = math.inf   #(y2-y1)==0 -> slope=0 -> slope_ver=infinity
		else:
			slope = -(y2-y1) / (x2-x1)
			slope_ver = -1/slope

		midpoint = (int((x1+x2)/2), int((y1+y2)/2))
		front_y = midpoint[1] + int((self.scan_img_size[0] - midpoint[1])/2)
		front_x = midpoint[0] - int((front_y-midpoint[1])/slope_ver)
		ramp_front = (front_x, front_y)
		#----------Convert indexes(ramp_front) to real coordinate xy----------
		local_goal_x = (self.center[0]-ramp_front[1])/self.scale
		local_goal_y = (self.center[1]-ramp_front[0])/self.scale
		""" """
		#----------Show image----------
		rgb_img = cv2.cvtColor(scan_img, cv2.COLOR_GRAY2RGB)
		rgb_img = cv2.line(rgb_img, (x1,y1), (x2,y2), (0,255,0), 2)
		rgb_img = cv2.circle(rgb_img, midpoint, 5, (255,255,0), thickness=2)
		rgb_img = cv2.line(rgb_img, midpoint, ramp_front, (0,255,255), 2)
		rgb_img = cv2.circle(rgb_img, ramp_front, 5, (0,0,255), thickness=-1)
		cv2.imshow("Scan Image", rgb_img)
		cv2.waitKey(1)

		return (local_goal_x, local_goal_y, slope_ver)


	#==========Process to approach to ramp (node_flag is 4)==========#
	def approachToRamp(self, scan_msg):
		local_goal_x, local_goal_y, slope_ver= self.detectRamp(scan_msg)
		theta = math.atan2(local_goal_y, local_goal_x)
		#-----Listen tf "map" -> "base_link"-----
		try:
			stamp = scan_msg.header.stamp
			tf = self.tfBuffer.lookup_transform("map", "base_link", stamp, rospy.Duration(1.0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			return

		(_, _, yaw) = euler_from_quaternion([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
		robot_pose = [tf.transform.translation.x, tf.transform.translation.y, yaw]

		#----------Publish goal msg or command vel----------
		goal_msg = self.newGoalMsg(local_goal_x, local_goal_y, theta, robot_pose)
		d = np.linalg.norm([robot_pose[0]-goal_msg.pose.position.x, robot_pose[1]-goal_msg.pose.position.y])
		if d<2.0 and abs(slope_ver)>20:
			self.client.wait_for_server()
			self.client.cancel_all_goals()
			self.process_id = 1
			self.run_hz = 10
			#self.latest_range_num = np.count_nonzero(scan_img>0)
		else:
			self.goal_pub.publish(goal_msg)

		return


	def climbRamp(self, speed):
		self.goStraight(speed)
		print("Climbing with speed of", speed, "m/s")
		if self.pitch > 0.1:
			self.process_id = 2
		"""
		# not using imu
		scan_img = self.scan2image(msg) #Convert laserscan data to image
		obs_range_num = np.count_nonzero(scan_img>0)
		if obs_range_num - self.latest_range_num > 120:
			self.latest_range_num = obs_range_num
			self.process_id = 2
		else:
			self.goStraight(0.8)
			self.latest_range_num = obs_range_num
		"""
		return


	def downRamp(self, speed):
		if abs(self.pitch) < 0.01:
			self.goStraight(0)
			rospy.set_param("node_flag",1)
			cv2.destroyAllWindows()
			rospy.sleep(1.0)
		else:
			self.goStraight(speed)
			print("Downing with speed of", speed, "m/s")
		"""
		# not using imu
		scan_img = self.scan2image(msg) #Convert laserscan data to image
		obs_range_num = np.count_nonzero(scan_img>0)
		if self.latest_range_num - obs_range_num > 120:
			self.goStraight(0)
			rospy.set_param("node_flag",2)
		else:
			self.goStraight(0.5)
			self.latest_range_num = obs_range_num
		"""
		return


	#==========Publish cmd_vel to go straight ahead (Using on the ramp)==========#
	def goStraight(self, speed):
		cmd = Twist()
		cmd.linear.x = speed
		self.cmd_pub.publish(cmd)
		return


	#==========Convert LaserScan data to image==========#
	def scan2image(self, scan_msg):
		scan_img = np.zeros(self.scan_img_size, dtype="uint8")
		for i in range(0,len(scan_msg.ranges)):
			if scan_msg.ranges[i] > self.max_range:
				continue
			th = scan_msg.angle_min + scan_msg.angle_increment*i
			"""
			if -math.pi/6<th or th<math.pi/6:
				continue
			"""
			r = self.scan_img_size[0]-1 - int(math.cos(th)*scan_msg.ranges[i]*self.scale)
			c = self.center[1] - int(math.sin(th)*scan_msg.ranges[i]*self.scale)
			scan_img[r,c] = 255
		return scan_img


	#==========Get new goal msg to approach to ramp==========#
	def newGoalMsg(self, lgx, lgy, lgth, rp):
		#-----local goal xy => global xy-----
		rpx = rp[0]
		rpy = rp[1]
		rpth = rp[2]
		goal_msg = PoseStamped()
		goal_msg.header.stamp = rospy.Time.now()
		goal_msg.header.frame_id = "map"
		goal_msg.pose.position.x = lgx*math.cos(rpth) - lgy*math.sin(rpth) + rpx
		goal_msg.pose.position.y = lgx*math.sin(rpth) + lgy*math.cos(rpth) + rpy
		goal_msg.pose.position.z = 0
		q = quaternion_from_euler(0, 0, lgth+rpth)
		goal_msg.pose.orientation.x = q[0]
		goal_msg.pose.orientation.y = q[1]
		goal_msg.pose.orientation.z = q[2]
		goal_msg.pose.orientation.w = q[3]
		return goal_msg



#################### main ####################
if __name__ == "__main__":
	rospy.init_node("ramp_detector", anonymous=True)

	ramp_detector = RampDetector()

	try:
		rospy.spin()
	except rospy.Exceptions.ROSException:
		cv2.destroyAllWindows()


