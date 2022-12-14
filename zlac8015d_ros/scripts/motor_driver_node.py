#!/usr/bin/env python3
"""
Copyright (C) 2022  rasheeddo
Copyright (C) 2022  Alpaca-zip

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import rospy
import tf2_ros
import numpy as np
import time
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from pymodbus.client.sync import ModbusSerialClient as ModbusClient

class MotorDriverNode:
	def __init__(self):
		self.client = ModbusClient(method = 'rtu', port = rospy.get_param("/motor_driver_node/port"), baudrate = 115200, timeout = 1)
		self.client.connect()
		self.ID = 1

		# -----Register Address-----
		# Common
		self.CONTROL_REG = 0x200E
		self.OPR_MODE = 0x200D
		self.L_ACL_TIME = 0x2080
		self.L_DCL_TIME = 0x2082

		# Speed RPM Control
		self.L_CMD_RPM = 0x2088

		# Position Control
		self.POS_CONTROL_TYPE = 0x200F
		self.L_MAX_RPM_POS = 0x208E
		self.L_CMD_REL_POS_HI = 0x208A
		self.L_FB_POS_HI = 0x20A7

		# -----Control CMDs (REG)-----
		self.EMER_STOP = 0x05
		self.ALRM_CLR = 0x06
		self.DOWN_TIME = 0x07
		self.ENABLE = 0x08
		self.POS_L_START = 0x11
		self.POS_R_START = 0x12

		# -----Operation Mode-----
		self.ASYNC = 0

		# -----Initialize Publisher-----
		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
		self.odom_msg = Odometry()

		# -----Initialize subscriber-----
		rospy.Subscriber(rospy.get_param("/motor_driver_node/twist_cmd_vel_topic"), Twist, self.twist_cmd_callback)
		rospy.Subscriber(rospy.get_param("/motor_driver_node/cmd_vel_topic"), Float32MultiArray, self.vel_cmd_callback)
		rospy.Subscriber(rospy.get_param("/motor_driver_node/cmd_rpm_topic"), Float32MultiArray, self.rpm_cmd_callback)
		rospy.Subscriber(rospy.get_param("/motor_driver_node/cmd_deg_topic"), Float32MultiArray, self.deg_cmd_callback)
		rospy.Subscriber(rospy.get_param("/motor_driver_node/cmd_dist_topic"), Float32MultiArray, self.dist_cmd_callback)
		rospy.Subscriber("/estop", Bool, self.estop_callback)

		# -----Initialize Control Mode-----
		self.control_mode = int(rospy.get_param("/motor_driver_node/control_mode"))
		if self.control_mode == 3:
			self.speed_mode_init()
		elif self.control_mode == 1:
			self.position_mode_init()

		# -----Initialize Variable-----
		self.linear_vel_cmd = 0.0
		self.angular_vel_cmd = 0.0
		self.got_twist_cmd = False

		self.left_vel_cmd = 0.0
		self.right_vel_cmd = 0.0
		self.got_vel_cmd = False

		self.left_rpm_cmd = 0.0
		self.right_rpm_cmd = 0.0
		self.got_vel_rpm_cmd = False

		self.left_pos_deg_cmd = 0.0
		self.right_pos_deg_cmd = 0.0
		self.got_pos_deg_cmd = False

		self.left_pos_dist_cmd = 0.0
		self.right_pos_dist_cmd = 0.0
		self.got_pos_dist_cmd = False

		self.estop = False

		self.last_subscribed_time = 0.0
		self.callback_timeout = float(rospy.get_param("/motor_driver_node/callback_timeout"))
		self.wheels_base_width = float(rospy.get_param("/motor_driver_node/wheels_base_width"))
		self.left_wheel_radius = float(rospy.get_param("/motor_driver_node/left_wheel_radius"))
		self.right_wheel_radius = float(rospy.get_param("/motor_driver_node/right_wheel_radius"))
		self.computation_left_wheel_radius = float(rospy.get_param("/motor_driver_node/computation_left_wheel_radius"))
		self.computation_right_wheel_radius = float(rospy.get_param("/motor_driver_node/computation_right_wheel_radius"))
		self.cpr = int(rospy.get_param("/motor_driver_node/cpr"))
		self.deadband_rpm = int(rospy.get_param("/motor_driver_node/deadband_rpm"))
		self.left_rpm_lim = int(rospy.get_param("/motor_driver_node/max_left_rpm"))
		self.right_rpm_lim = int(rospy.get_param("/motor_driver_node/max_right_rpm"))
		self.publish_TF = rospy.get_param("/motor_driver_node/publish_TF")
		self.publish_odom = rospy.get_param("/motor_driver_node/publish_odom")
		self.TF_header_frame = rospy.get_param("/motor_driver_node/TF_header_frame")
		self.TF_child_frame = rospy.get_param("/motor_driver_node/TF_child_frame")
		self.odom_header_frame = rospy.get_param("/motor_driver_node/odom_header_frame")
		self.odom_child_frame = rospy.get_param("/motor_driver_node/odom_child_frame")
		self.debug = rospy.get_param("/motor_driver_node/debug")
		self.period = 0.05
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.l_meter = 0.0
		self.r_meter = 0.0
		self.prev_l_meter = 0.0
		self.prev_r_meter = 0.0
		self.l_meter_init, self.r_meter_init = self.get_wheels_travelled()
		self.t = TransformStamped()
		self.br = tf2_ros.TransformBroadcaster()
		self.state_vector = np.zeros((3, 1))

		print("\033[32m" + "Start ZLAC8015D motor driver node" + "\033[0m")
		print("#########################")

	"""
	##############################
	## speed_mode_init function ##
	##############################
	Initialization of speed RPM control mode.
	"""
	def speed_mode_init(self):
		# -----Disable Motor-----
		result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit = self.ID)

		# -----Set Accel Time-----
		AL_ms = int(rospy.get_param("/motor_driver_node/set_accel_time_left"))
		AR_ms = int(rospy.get_param("/motor_driver_node/set_accel_time_right"))
		if AL_ms > 32767:
			AL_ms = 32767
		elif AL_ms < 0:
			AL_ms = 0

		if AR_ms > 32767:
			AR_ms = 32767
		elif AR_ms < 0:
			AR_ms = 0

		result = self.client.write_registers(self.L_ACL_TIME, [AL_ms, AR_ms], unit = self.ID)

		# -----Set Decel Time-----
		DL_ms = int(rospy.get_param("/motor_driver_node/set_decel_time_left"))
		DR_ms = int(rospy.get_param("/motor_driver_node/set_decel_time_right"))
		if DL_ms > 32767:
			DL_ms = 32767
		elif DL_ms < 0:
			DL_ms = 0

		if DR_ms > 32767:
			DR_ms = 32767
		elif DR_ms < 0:
			DR_ms = 0

		result = self.client.write_registers(self.L_DCL_TIME, [DL_ms, DR_ms], unit = self.ID)

		# -----Set Mode-----
		mode = 3
		result = self.client.write_register(self.OPR_MODE, mode, unit=self.ID)
		print("\033[32m" + "Set mode as speed RPM control" + "\033[0m")

		# -----Enable Motor-----
		result = self.client.write_register(self.CONTROL_REG, self.ENABLE, unit = self.ID)

	"""
	#################################
	## position_mode_init function ##
	#################################
	Initialization of relative position control mode.
	"""
	def position_mode_init(self):
		# -----Disable Motor-----
		result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit = self.ID)

		# -----Set Accel Time-----
		AL_ms = int(rospy.get_param("/motor_driver_node/set_accel_time_left"))
		AR_ms = int(rospy.get_param("/motor_driver_node/set_accel_time_right"))
		if AL_ms > 32767:
			AL_ms = 32767
		elif AL_ms < 0:
			AL_ms = 0

		if AR_ms > 32767:
			AR_ms = 32767
		elif AR_ms < 0:
			AR_ms = 0

		result = self.client.write_registers(self.L_ACL_TIME, [AL_ms, AR_ms], unit = self.ID)

		# -----Set Decel Time-----
		DL_ms = int(rospy.get_param("/motor_driver_node/set_decel_time_left"))
		DR_ms = int(rospy.get_param("/motor_driver_node/set_decel_time_right"))
		if DL_ms > 32767:
			DL_ms = 32767
		elif DL_ms < 0:
			DL_ms = 0

		if DR_ms > 32767:
			DR_ms = 32767
		elif DR_ms < 0:
			DR_ms = 0

		result = self.client.write_registers(self.L_DCL_TIME, [DL_ms, DR_ms], unit = self.ID)

		# -----Set Mode-----
		mode = 1
		result = self.client.write_register(self.OPR_MODE, mode, unit=self.ID)
		print("\033[32m" + "Set mode as relative position control" + "\033[0m")

		# -----Set Position Async Control-----
		result = self.client.write_register(self.POS_CONTROL_TYPE, self.ASYNC, unit = self.ID)

		# -----Set Position Async Control-----
		max_L_rpm = int(rospy.get_param("/motor_driver_node/max_left_rpm"))
		max_R_rpm = int(rospy.get_param("/motor_driver_node/max_right_rpm"))
		if max_L_rpm > 1000:
			max_L_rpm = 1000
		elif max_L_rpm < 1:
			max_L_rpm = 1

		if max_R_rpm > 1000:
			max_R_rpm = 1000
		elif max_R_rpm < 1:
			max_R_rpm = 1

		result = self.client.write_registers(self.L_MAX_RPM_POS, [max_L_rpm, max_R_rpm], unit = self.ID)

		# -----Enable Motor-----
		result = self.client.write_register(self.CONTROL_REG, self.ENABLE, unit = self.ID)

	"""
	#################################
	## twist_cmd_callback function ##
	#################################
	Callback function to subscribe for "/zlac8015d/twist/cmd_vel".
	"""
	def twist_cmd_callback(self, msg):
		self.linear_vel_cmd = msg.linear.x
		self.angular_vel_cmd = msg.angular.z
		self.got_twist_cmd = True
		self.last_subscribed_time = time.perf_counter()

	"""
	###############################
	## vel_cmd_callback function ##
	###############################
	Callback function to subscribe for "/zlac8015d/vel/cmd_vel".
	"""
	def vel_cmd_callback(self, msg):
		self.left_vel_cmd = msg.data[0]
		self.right_vel_cmd = -msg.data[1]
		self.got_vel_cmd = True
		self.last_subscribed_time = time.perf_counter()

	"""
	###############################
	## rpm_cmd_callback function ##
	###############################
	Callback function to subscribe for "/zlac8015d/vel/cmd_rpm".
	"""
	def rpm_cmd_callback(self, msg):
		self.left_rpm_cmd = msg.data[0]
		self.right_rpm_cmd = -msg.data[1]
		self.got_vel_rpm_cmd = True
		self.last_subscribed_time = time.perf_counter()

	"""
	###############################
	## deg_cmd_callback function ##
	###############################
	Callback function to subscribe for "/zlac8015d/pos/cmd_deg".
	"""
	def deg_cmd_callback(self, msg):
		self.left_pos_deg_cmd = msg.data[0]
		self.right_pos_deg_cmd = -msg.data[1]
		self.got_pos_deg_cmd = True

	"""
	################################
	## dist_cmd_callback function ##
	################################
	Callback function to subscribe for "/zlac8015d/pos/cmd_dist".
	"""
	def dist_cmd_callback(self, msg):
		self.left_pos_dist_cmd = msg.data[0]
		self.right_pos_dist_cmd = msg.data[1]
		self.got_pos_dist_cmd = True
    
	"""
	##############################
	## estop_callback function ##
	##############################
	Callback function to subscribe for "/estop".
	"""
	def estop_callback(self, msg):
		self.estop = msg.data

	"""
	###########################
	## twist_to_rpm function ##
	###########################
	Convert from twist to RPM.
	"""
	def twist_to_rpm(self, linear_vel, angular_vel):
		left_vel = linear_vel - self.wheels_base_width / 2 * angular_vel
		right_vel = linear_vel + self.wheels_base_width / 2 * angular_vel
		left_rpm, right_rpm = self.vel_to_rpm(left_vel, -right_vel)
		return left_rpm, right_rpm

	"""
	#########################
	## vel_to_rpm function ##
	#########################
	Convert from speed to RPM.
	"""
	def vel_to_rpm(self, left_vel, right_vel):
		left_rpm = 60 * left_vel / (2 * np.pi * self.left_wheel_radius)
		right_rpm = 60 * right_vel / (2 * np.pi * self.right_wheel_radius)
		return left_rpm, right_rpm
    
	"""
	#####################################
	## dist_to_relative_angle function ##
	#####################################
	Convert from distance to relative angle.
	"""
	def dist_to_relative_angle(self, left_dist, right_dist):
		left_circumference = self.left_wheel_radius * 2 * np.pi
		right_circumference = self.right_wheel_radius * 2 * np.pi
		left_relative_deg = (left_dist * 360.0) / left_circumference
		right_relative_deg = (-right_dist * 360.0) / right_circumference
		return left_relative_deg, right_relative_deg

	"""
	###################################
	## int16Dec_to_int16Hex function ##
	###################################
	Convert from int16Dec to int16Hex.
	"""
	def int16Dec_to_int16Hex(self, int16):
		lo_byte = (int16 & 0x00FF)
		hi_byte = (int16 & 0xFF00) >> 8
		all_bytes = (hi_byte << 8) | lo_byte
		return all_bytes

	"""
	#################################
	## deg_to_32bitArray function ##
	#################################
	Convert from degree to 32bitArray.
	"""
	def deg_to_32bitArray(self, deg):
		dec = int((deg + 1440) * (65536 + 65536) / (1440 + 1440) - 65536)
		HI_WORD = (dec & 0xFFFF0000) >> 16
		LO_WORD = dec & 0x0000FFFF
		return [HI_WORD, LO_WORD]

	"""
	###################################
	## get_wheels_travelled function ##
	###################################
	Get distance traveled by left and right wheels.
	"""
	def get_wheels_travelled(self):
		registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
		l_pul_hi = registers[0]
		l_pul_lo = registers[1]
		r_pul_hi = registers[2]
		r_pul_lo = registers[3]
		l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
		r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
		l_travelled = (float(l_pulse) / self.cpr) * self.computation_left_wheel_radius * np.pi * 8  # unit in meter
		r_travelled = (float(r_pulse) / self.cpr) * self.computation_right_wheel_radius * np.pi * 8  # unit in meter
		return l_travelled, r_travelled
    
	"""
	#######################################
	## modbus_fail_read_handler function ##
	#######################################
	Get data from registers.
	"""
	def modbus_fail_read_handler(self, ADDR, WORD):
		read_success = False
		reg = [None] * WORD
		while not read_success:
			result = self.client.read_holding_registers(ADDR, WORD, unit = self.ID)
			try:
				for i in range(WORD):
					reg[i] = result.registers[i]
				read_success = True
			except AttributeError as e:
				print(e)
				pass
		return reg

	"""
	#################################
	## set_rpm_with_limit function ##
	#################################
	Set RPM with limit.
	"""
	def set_rpm_with_limit(self, left_rpm, right_rpm):
		if self.left_rpm_lim < left_rpm:
			left_rpm = self.left_rpm_lim
			rospy.logwarn("RPM reach the limit.")
		elif left_rpm < -self.left_rpm_lim:
			left_rpm = -self.left_rpm_lim
			rospy.logwarn("RPM reach the limit.")
		elif -self.deadband_rpm < left_rpm < self.deadband_rpm:
			left_rpm = 0

		if self.right_rpm_lim < right_rpm:
			right_rpm = self.right_rpm_lim
			rospy.logwarn("RPM reach the limit.")
		elif right_rpm < -self.right_rpm_lim:
			right_rpm = -self.right_rpm_lim    
			rospy.logwarn("RPM reach the limit.")        
		elif -self.deadband_rpm < right_rpm < self.deadband_rpm:
			right_rpm = 0

		left_bytes = self.int16Dec_to_int16Hex(int(left_rpm))
		right_bytes = self.int16Dec_to_int16Hex(int(right_rpm))
		result = self.client.write_registers(self.L_CMD_RPM, [left_bytes, right_bytes], unit = self.ID)

	"""
	#################################
	## set_relative_angle function ##
	#################################
	Set relative angle.
	"""
	def set_relative_angle(self, ang_L, ang_R):
		L_array = self.deg_to_32bitArray(ang_L / 4)
		R_array = self.deg_to_32bitArray(ang_R / 4)
		all_cmds_array = L_array + R_array
		result = self.client.write_registers(self.L_CMD_REL_POS_HI, all_cmds_array, unit = self.ID)

	"""
	#################################
	## calculate_odometry function ##
	#################################
	Odometry computation.
	"""
	def calculate_odometry(self):
		self.l_meter, self.r_meter = self.get_wheels_travelled()
		self.l_meter = self.l_meter - self.l_meter_init
		self.r_meter = (-1 * self.r_meter) - (-1 * self.r_meter_init)
		vl = (self.l_meter - self.prev_l_meter) / self.period
		vr = (self.r_meter - self.prev_r_meter) / self.period
		Wl = vl / self.computation_left_wheel_radius
		Wr = vr / self.computation_right_wheel_radius
		matrix = np.array([[self.computation_right_wheel_radius / 2, self.computation_left_wheel_radius / 2], [self.computation_right_wheel_radius / self.wheels_base_width, -self.computation_left_wheel_radius / self.wheels_base_width]])
		vector = np.array([[Wr], [Wl]])
		input_vector = np.dot(matrix, vector)
		V = input_vector[0, 0]
		Wz = input_vector[1, 0]
		out_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
		dirc_matrix = np.array([[np.cos(self.state_vector[2, 0]) * self.period, 0], [np.sin(self.state_vector[2, 0]) * self.period, 0], [0, self.period]])
		out_vector = np.dot(out_matrix, self.state_vector) + np.dot(dirc_matrix, input_vector)
		x = out_vector[0, 0]
		y = out_vector[1, 0]
		theta = out_vector[2, 0]
		
		# -----Construct TF-----
		if(self.publish_TF):
			self.t.header.stamp = rospy.Time.now()
			self.t.header.frame_id = self.TF_header_frame
			self.t.child_frame_id = self.TF_child_frame
			self.t.transform.translation.x = x
			self.t.transform.translation.y = y
			self.t.transform.translation.z = 0.0
			rotation = quaternion_from_euler(0, 0, theta)
			self.t.transform.rotation.x = rotation[0]
			self.t.transform.rotation.y = rotation[1]
			self.t.transform.rotation.z = rotation[2]
			self.t.transform.rotation.w = rotation[3]
			self.br.sendTransform(self.t)
			
		# -----Construct Odom Message-----
		if(self.publish_odom):
			self.odom_msg.header.stamp = rospy.Time.now()
			self.odom_msg.header.frame_id = self.odom_header_frame
			self.odom_msg.child_frame_id = self.odom_child_frame
			self.odom_msg.pose.pose.position.x = x
			self.odom_msg.pose.pose.position.y = y
			self.odom_msg.pose.pose.position.z = 0.0
			orientation = quaternion_from_euler(0, 0, theta)
			self.odom_msg.pose.pose.orientation.x = orientation[0]
			self.odom_msg.pose.pose.orientation.y = orientation[1]
			self.odom_msg.pose.pose.orientation.z = orientation[2]
			self.odom_msg.pose.pose.orientation.w = orientation[3]
			self.odom_msg.pose.covariance[0] = 0.0001
			self.odom_msg.pose.covariance[7] = 0.0001
			self.odom_msg.pose.covariance[14] = 0.000001
			self.odom_msg.pose.covariance[21] = 0.000001
			self.odom_msg.pose.covariance[28] = 0.000001
			self.odom_msg.pose.covariance[35] = 0.0001
			self.odom_msg.twist.twist.linear.x = V
			self.odom_msg.twist.twist.linear.y = 0.0
			self.odom_msg.twist.twist.angular.z = Wz
			self.odom_pub.publish(self.odom_msg)

		self.state_vector[0, 0] = x
		self.state_vector[1, 0] = y
		self.state_vector[2, 0] = theta
		return x, y, theta

	"""
	##############################
	## control_loop function ##
	##############################
	Control loop.
	"""
	def control_loop(self):
		rate = rospy.Rate(1 / self.period)
		estop_reset_flag = False
        
		while True:
			if rospy.is_shutdown():
				# -----Disable Motor-----
				result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit = self.ID)
				break

			start_time = time.perf_counter()
			if self.estop:
				# -----Emergency Stop-----
				if self.control_mode == 3:
					self.set_rpm_with_limit(0, 0)
				elif self.control_mode == 1:
					result = self.client.write_register(self.CONTROL_REG, self.EMER_STOP, unit=self.ID)
					
				if not estop_reset_flag:
					#result = self.client.write_register(self.CONTROL_REG, self.EMER_STOP, unit=self.ID)
					
					rospy.logwarn("####################")
					rospy.logwarn("---EMERGENCY STOP---")
					rospy.logwarn("####################")
				estop_reset_flag = True
			elif not self.estop:
				if estop_reset_flag:
					# -----Clear Alarm-----
					result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR, unit=self.ID)

					# -----Disable Motor-----
					result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit = self.ID)

					# -----Enable Motor-----
					result = self.client.write_register(self.CONTROL_REG, self.ENABLE, unit = self.ID)

					estop_reset_flag = False
                
				# -----Speed RPM Control-----
				if self.control_mode == 3:
					if self.got_twist_cmd:
						self.left_rpm_cmd, self.right_rpm_cmd = self.twist_to_rpm(self.linear_vel_cmd, self.angular_vel_cmd)
						self.set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
						self.got_twist_cmd = False
					elif self.got_vel_cmd:
						self.left_rpm_cmd, self.right_rpm_cmd = self.vel_to_rpm(self.left_vel_cmd, self.right_vel_cmd)
						self.set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
						self.got_vel_cmd = False
					elif self.got_vel_rpm_cmd:
						self.set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
						self.got_vel_rpm_cmd = False
					elif (time.perf_counter() - self.last_subscribed_time) > self.callback_timeout:
						self.left_rpm_cmd = 0.0
						self.right_rpm_cmd = 0.0
						self.set_rpm_with_limit(self.left_rpm_cmd, self.right_rpm_cmd)
                        
				# -----Position Control-----
				elif self.control_mode == 1:
					if self.got_pos_deg_cmd:
						self.set_relative_angle(self.left_pos_deg_cmd ,self.right_pos_deg_cmd)
						# -----Move Left Wheel-----
						result = self.client.write_register(self.CONTROL_REG, self.POS_L_START, unit=self.ID)

						# -----Move Right Wheel-----
						result = self.client.write_register(self.CONTROL_REG, self.POS_R_START, unit=self.ID)

						self.got_pos_deg_cmd = False
					elif self.got_pos_dist_cmd:
						self.left_pos_deg_cmd, self.right_pos_deg_cmd = self.dist_to_relative_angle(self.left_pos_dist_cmd, self.right_pos_dist_cmd)
						self.set_relative_angle(self.left_pos_deg_cmd ,self.right_pos_deg_cmd)
						# -----Move Left Wheel-----
						result = self.client.write_register(self.CONTROL_REG, self.POS_L_START, unit=self.ID)

						# -----Move Right Wheel-----
						result = self.client.write_register(self.CONTROL_REG, self.POS_R_START, unit=self.ID)

						self.got_pos_dist_cmd = False

			#-----Odometry computation-----
			x, y, theta = self.calculate_odometry()
			self.period = time.perf_counter() - start_time
			self.prev_l_meter = self.l_meter
			self.prev_r_meter = self.r_meter
			
			#-----Debugging Feature-----
			if (self.debug):
				print("\033[32m" + "x: {:f} | y: {:f} | yaw: {:f}".format(x, y, np.rad2deg(theta)) + "\033[0m")
			else:
				pass

			rate.sleep()
			
"""
###################
## Main function ##
###################
Main.
"""
def main():
	rospy.init_node("motor_driver_node")
	MDN = MotorDriverNode()
	MDN.control_loop()
	rospy.spin()

if __name__ == "__main__":
	main()
