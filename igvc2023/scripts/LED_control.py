#!/usr/bin/env python3
# maintainer:kbkn/mori
"""
******************************************
This node changes led to navigation mode 
when no_map_navigation.launch is executed.
******************************************
"""
import rospy
from std_msgs.msg import Bool
import time

class LED_control:
	def __init__(self):
		self.navigation_start_pub = rospy.Publisher("/navBool", Bool, queue_size = 10)
		self.navigation_end_pub = rospy.Publisher("/navBool", Bool, queue_size = 10)
		self.navBool_msg = Bool()

	def main(self):
		self.navBool_msg = True
		#self.navigation_start_pub.publish(self.navBool_msg)
		while not rospy.is_shutdown():
			self.navigation_start_pub.publish(self.navBool_msg)
			node_flag = rospy.get_param("node_flag", 1)
			time.sleep(0.5)
			if node_flag == 4:
				self.navBool_msg = False
				self.navigation_end_pub.publish(self.navBool_msg)

if __name__ == "__main__":
	#init node
	rospy.init_node("LED_control")
	LEDc = LED_control()
	LEDc.main()
