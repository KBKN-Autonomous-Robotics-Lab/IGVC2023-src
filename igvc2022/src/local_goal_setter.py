#!/usr/bin/env python3
# maintainer:kbkn/kubota and mori

import rospy
import tf2_ros
import actionlib
import numpy as np
import cv2
import math
import ruamel.yaml
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction
from std_srvs.srv import Empty


class LocalGoalSetter:
	def __init__(self):
		self.costmap_sub = rospy.Subscriber("move_base/local_costmap/costmap", OccupancyGrid, self.callback)
		self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer)
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.init_flag = True
		self.refer_idx = ()
		self.theta = []
		self.pre_theta = 0.0
		self.run_counter = 0
		self.time_counter = [0.,0]
		#self.waypoints_file = "/home/ubuntu/catkin_ws/src/igvc2022/config/gps_waypoints/gps_waypoints.yaml"
		self.waypoints_file = rospy.get_param("waypoints_file")
		self.waypoints = []
		self.dist = []
		self.select_goal_flag = True
		self.waypoints_goal = 2
		self.clear_costmap = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
		self.clear_counter = 0
	
	
	def callback(self, msg):
		node_flag = rospy.get_param("node_flag",1) # mori: edited
		if (node_flag!=1 and node_flag!=3) or len(msg.data)==0:
			return
		
		width = msg.info.width
		height = msg.info.height
		#-----Only first callback-----
		if self.init_flag:
			start_msg = """
###################
  Node1 started!!
###################
"""
			print("\033[35m" + start_msg + "\033[0m")
			min_angle = -math.pi/2 + np.deg2rad(20)
			max_angle = math.pi/2 - np.deg2rad(20)
			direction_num = 9
			self.refer_idx, self.theta = self.setReferenceIndex(width, height, min_angle, max_angle, direction_num)
			self.init_flag = False
		
		#-----Frequency control-----
		""""""
		self.run_counter += 1
		if self.run_counter != 3:
			return
		else:
			self.run_counter = 0
		
		#-----Clear costmaps perodically-----
		self.clear_counter += 1
		if self.clear_counter == 7:
			self.clear_costmap()
			self.clear_counter = 0
			rospy.sleep(0.5)
		
		#-----Listen tf map->base_link-----
		try:
			stamp = msg.header.stamp
			tf = self.tfBuffer.lookup_transform("map", "base_link", stamp, rospy.Duration(1.0))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			return
		"""
		start = time.perf_counter()
		"""
		#-----Convert costmap msg to image format-----
		costmap = np.empty((width,height), dtype=np.int8)
		for r in range(0,width):
			costmap[width-r-1,:] = msg.data[r*width:(r+1)*width]
		
		costmap = cv2.rotate(costmap, cv2.ROTATE_90_COUNTERCLOCKWISE)
		np.place(costmap, costmap<0, 0)
		costmap = costmap.astype(np.uint8)
		
		#-----Rotate costmap image to match robot orientation-----
		(_, _, yaw) = euler_from_quaternion([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
		rotmat = cv2.getRotationMatrix2D(center=(width/2, height/2), angle=math.degrees(-yaw), scale=1)
		rot_costmap = cv2.warpAffine(costmap, rotmat, (width, height))
		
		#-----Search best direction-----
		center = (int(width/2), int(height/2))
		scores = np.zeros(len(self.theta), dtype=np.float32) # distance to obstacle(cost) from center
		score = 0.0
		sid = 0
		window = 6
		for i in range(1, len(self.refer_idx[0])):
			r = self.refer_idx[0][i]
			c = self.refer_idx[1][i]
			if r==-1 and c==-1:
				if scores[sid] == 0:
					scores[sid] = np.linalg.norm([center[1]-self.refer_idx[0][i-1], center[0]-self.refer_idx[1][i-1]]) - abs(self.theta[sid])*0.1
				sid+=1
				continue
			
			if r-window<0 or c-window<0 or c+window>=width:
				continue
			
			if np.sum(rot_costmap[r-window:r+window, c-window:c+window]) > 0:
				score = np.linalg.norm([center[1]-r, center[0]-c])
				if scores[sid]==0 or scores[sid]>score:
					scores[sid] = score
			
		#-----Determine local goal-----
		consider_num = 1
		max_idx = np.argmax(scores)
		if max_idx>consider_num-1 and max_idx<len(scores)-consider_num: # consider front and rear angles scores
			max_score = np.average(scores[max_idx-consider_num:max_idx+consider_num+1])
			weight = [(w/max_score)**4 for w in scores[max_idx-consider_num:max_idx+consider_num+1]] # to increase score difference, **3
			max_score_theta = np.average(self.theta[max_idx-consider_num:max_idx+consider_num+1], weights=weight)
		else:
			max_score = np.max(scores)
			max_score_theta = self.theta[np.argmax(scores)]
		
		if abs(max_score_theta-self.pre_theta) > math.pi/4:
			rospy.loginfo("Local goal changed significantly compared to previous step: %s",abs(max_score_theta-self.pre_theta))
			self.client.wait_for_server()
			self.client.cancel_all_goals()
			rospy.sleep(1.0)
			self.pre_theta = max_score_theta
			return
		else:
			self.pre_theta = max_score_theta
		
		
		reduct = 0.9
		local_goal_x = max_score * math.cos(max_score_theta) * reduct
		local_goal_y = max_score * math.sin(max_score_theta) * reduct
		resolution = msg.info.resolution # multiplie this value to convert to real distance
		robot_pose = [tf.transform.translation.x, tf.transform.translation.y, yaw]
		goal_msg = self.newGoalMsg(local_goal_x*resolution, local_goal_y*resolution, max_score_theta, robot_pose)
		
		#-----Distance to finish point from base_link-----
		'''
		finish_point_x = rospy.get_param("local_goal_setter/finish_point_x")
		finish_point_y = rospy.get_param("local_goal_setter/finish_point_y")
		distance_to_finpt = np.linalg.norm([robot_pose[0]-finish_point_x, robot_pose[1]-finish_point_y])
		if distance_to_finpt < 2:
			rospy.set_param("mode",2) #change mode to gps waypoint navigation
			cv2.destroyWindow("Robot Frame Costmap")
			return
		else:
			self.goal_pub.publish(goal_msg)
		'''
		#-----mori: edited-----
		if self.select_goal_flag:
			self.dist = []
			yaml = ruamel.yaml.YAML()
			with open(self.waypoints_file) as file:
				self.waypoints = yaml.load(file) # IGVC2022 has 4 pre-defined gps waypoint
			for i in range(len(self.waypoints["waypoints"])):
				self.dist.append(self.calc_dist(robot_pose, i, 0))
			self.dist.append(self.calc_dist(robot_pose, 0, 1)) # 5th in the list is the distance from the origin
			
			if self.dist.index(min(self.dist)) == 0: # if the robot is close to waypoint1
				self.waypoints_goal = 0 # origin
			elif self.dist.index(min(self.dist)) == 1: # if the robot is close to waypoint2
				self.waypoints_goal = 2 # ramp
			elif self.dist.index(min(self.dist)) == 2: # if the robot is close to waypoint3
				self.waypoints_goal = 2 # ramp
			elif self.dist.index(min(self.dist)) == 3: # if the robot is close to waypoint4
				self.waypoints_goal = 0 # origin
			elif self.dist.index(min(self.dist)) == 4: # if the robot is close to origin
				self.waypoints_goal = 1 # waypoint1 or waypoint4
			self.select_goal_flag = False
		
		distance = 1.5 # [m]
		if self.waypoints_goal == 0:
			print("\r" + "\033[34m" + "Distance from goal: " + str(self.calc_dist(robot_pose, 0, 2)) + "\033[0m", end = "")
		elif self.waypoints_goal == 1:
			if self.calc_dist(robot_pose, 0, 0) < self.calc_dist(robot_pose, 3, 0):
				print("\r" + "\033[34m" + "Distance from waypoint1: " + str(self.calc_dist(robot_pose, 0, 0)) + "\033[0m", end = "")
			else:
				print("\r" + "\033[34m" + "Distance from waypoint4: " + str(self.calc_dist(robot_pose, 3, 0)) + "\033[0m", end = "")
		elif node_flag == 1 and self.waypoints_goal == 2:
			if self.calc_dist(robot_pose, 1, 0) < self.calc_dist(robot_pose, 2, 0):
				print("\r" + "\033[34m" + "Distance from waypoint2: " + str(self.calc_dist(robot_pose, 1, 0)) + "\033[0m", end = "")
			else:
				print("\r" + "\033[34m" + "Distance from waypoint3: " + str(self.calc_dist(robot_pose, 2, 0)) + "\033[0m", end = "")
			
		if self.waypoints_goal == 0 and self.calc_dist(robot_pose, 4, 2) < 0.5:
			goal_msg = """
###################
  Goal reached!!
###################
"""
			print("\033[35m" + goal_msg + "\033[0m")
			rospy.set_param("node_flag", 5) # node_flag == 5 then disable all node
			cv2.destroyAllWindows()
			return
		elif self.waypoints_goal == 1 and (self.calc_dist(robot_pose, 0, 0) < distance or self.calc_dist(robot_pose, 3, 0) < distance):
			print("")
			print("\033[32m" + "Activate next node!!" + "\033[0m")
			rospy.set_param("node_flag", 2) # node_flag == 2 then activate gps_goal_setter node
			self.select_goal_flag = True
			cv2.destroyAllWindows()
			return
		elif self.waypoints_goal == 2 and node_flag == 1 and (self.calc_dist(robot_pose, 1, 0) < distance or self.calc_dist(robot_pose, 2, 0) < distance):
			print("")
			print("\033[32m" + "Activate next node!!" + "\033[0m")
			rospy.set_param("node_flag", 2) # node_flag == 2 then activate gps_goal_setter node
			self.select_goal_flag = True
			cv2.destroyAllWindows()
			return
		
		self.goal_pub.publish(goal_msg)
		
		"""
		#-----Time count-----
		end = time.perf_counter()
		self.time_counter[0] += + (end-start)
		self.time_counter[1] += 1
		if self.time_counter[1] == 30:
			print("imInit ", self.time_counter[0]/30*1000, "[ms]")
			self.time_counter = [0.,0]
		"""
		
		#-----Show Images-----
		rgb_costmap = cv2.cvtColor(rot_costmap, cv2.COLOR_GRAY2RGB)
		for i in range(0, len(self.theta)):
			x = center[0] - int(scores[i] * math.sin(self.theta[i]))
			y = height - (center[1] + int(scores[i] * math.cos(self.theta[i])))
			rgb_costmap = cv2.line(rgb_costmap, center, (x,y), (0,100,0),1)
		rgb_costmap = cv2.circle(rgb_costmap, (center[0]-int(local_goal_y), height-(center[1]+int(local_goal_x))), 5, (0,0,100), thickness=-1)
		cv2.imshow("Robot Frame Costmap", rgb_costmap/100*255)
		cv2.waitKey(1)
		
		return
	
	
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
	
	
	def setReferenceIndex(self, w, h, min_ang, max_ang, num):
		increment = (max_ang-min_ang) / (num-1)
		theta = [min_ang+increment*i for i in range(0,num)]
		center = (int(w/2), int(h/2))
		max_range = min(center)
		
		ref_ind_r = np.array([], dtype=np.int16)
		ref_ind_c = np.array([], dtype=np.int16)
		for th in theta:
			x = center[0] - int(max_range * math.sin(th))
			y = h - (center[1] + int(max_range * math.cos(th)))
			img = np.zeros((w,h), np.uint8)
			line_img = cv2.line(img, center, (x,y), 255, 1)
			ind = np.where(line_img==255)
			ref_ind_r = np.append(ref_ind_r, -1)
			ref_ind_c = np.append(ref_ind_c, -1)
			ref_ind_r = np.append(ref_ind_r, ind[0])
			ref_ind_c = np.append(ref_ind_c, ind[1])
			ref_ind_r = np.append(ref_ind_r, y)
			ref_ind_c = np.append(ref_ind_c, x)
		
		return (ref_ind_r, ref_ind_c), theta
		
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
		
	#-----mori: edited-----
	def calc_dist(self, robot_pose, num, stat):
		if stat == 0:
			return math.sqrt((robot_pose[0] - self.waypoints["waypoints"][num]["point"]["x"]) ** 2 + (robot_pose[1] - self.waypoints["waypoints"][num]["point"]["y"]) ** 2)
		elif stat == 1:
			return math.sqrt(robot_pose[0] ** 2 + robot_pose[1] ** 2)
		else:
			return math.sqrt((robot_pose[0] + 1) ** 2 + robot_pose[1] ** 2)
    
if __name__ == "__main__":
	rospy.init_node("local_goal_setter", anonymous=True)
	
	goal_setter = LocalGoalSetter()
	
	try:
		rospy.spin()
	except rospy.Exceptions.ROSException:
		cv2.destroyAllWindows()
