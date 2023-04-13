#!/usr/bin/env python3
# maintainer:kbkn/mori
"""
******************************************
This node gives the goal to "move_base" 
that the robot should head for based on 
"qualification_gps_waypoints.yaml".
******************************************
"""
import rospy
import tf2_ros
import math
import sys
import ruamel.yaml
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction

class GPSGoalSetter:
  def __init__(self):
    self.waypoints_file = rospy.get_param("waypoints_file")
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    # true for the first time in loop processing, False for the second and subsequent times
    self.flag = True
    self.first_flag = True
    self.now = rospy.Time()
    self.goal_message = PoseStamped()
    self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

  """
  ++++++++++++++++++++++++++++++++++
      goal_arrival_flag function
  ++++++++++++++++++++++++++++++++++
  True when within 1.0m of the goal waypoint.
  """
  def goal_arrival_flag(self, waypoints, num):
    coordinates = self.get_coordinate_from_tf(self.now)
    dist = math.sqrt((coordinates["x"] - waypoints["waypoints"][num-1]["point"]["x"]) ** 2 + (coordinates["y"] - waypoints["waypoints"][num-1]["point"]["y"]) ** 2)
    print("\r" + "\033[34m" + "Distance from waypoint" + str(num) + ": " + str(dist) + "\033[0m", end = "")
    if dist < 0.5: # 1.0m
      print("")
      print("\033[32m" + "Waypoint" + str(num) + " reached!!" + "\033[0m")
      return True
    else:
      return False


  """
  ++++++++++++++++++++++++++++++++++
       load_waypoint function
  ++++++++++++++++++++++++++++++++++
  Get gps waypoints from yaml file.
  """
  def load_waypoint(self, file):
    yaml = ruamel.yaml.YAML()
    with open(file) as file:
      waypoints = yaml.load(file)
    return waypoints
  
  """
  ++++++++++++++++++++++++++++++++++
   get_coordinate_from_tf function
  ++++++++++++++++++++++++++++++++++
  Get (x, y)coordinates of the igvc robot.
  """
  def get_coordinate_from_tf(self, stamp):
    # if failed, retry up to 3 times
    for i in range(3):
      try:
        tf = self.tfBuffer.lookup_transform("map", "base_link", stamp, rospy.Duration(1.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
        if not i == 2:
          rospy.logwarn(err)
          print("Retrying...")
        pass
      else:
        break
    else:
      rospy.logerr("Can't access TF. Node is shutting down...")
      sys.exit()
    coordinates = {"x": tf.transform.translation.x, "y": tf.transform.translation.y, "rx": tf.transform.rotation.x, "ry": tf.transform.rotation.y, "rz": tf.transform.rotation.z, "rw": tf.transform.rotation.w}
    print(coordinates)
    return coordinates
      
  """
  ++++++++++++++++++++++++++++++++++
          pub_goal function
  ++++++++++++++++++++++++++++++++++
  Publish goal to move_base.
  The robot's finish pose will match the robot's pose when the goal is published
  """
  def pub_goal(self, waypoints, num):
    if self.first_flag:
      coordinates = self.get_coordinate_from_tf(self.now)
      goal_msg = PoseStamped()
      goal_msg.header.stamp = rospy.Time()
      goal_msg.header.frame_id = "map"
      goal_msg.pose.position.x = waypoints["waypoints"][num-1]["point"]["x"]
      goal_msg.pose.position.y = waypoints["waypoints"][num-1]["point"]["y"]
      goal_msg.pose.position.z = 0.0
      goal_msg.pose.orientation.x = 0.0
      goal_msg.pose.orientation.y = 0.0
      goal_msg.pose.orientation.z = 0.0
      goal_msg.pose.orientation.w = 1.0
      self.goal_message = goal_msg
      self.goal_pub.publish(goal_msg)
      self.first_flag = False
    else:
      #print("\033[32m" + "Go to waypoint" + "\033[0m")
      self.goal_pub.publish(self.goal_message)
  """
  ++++++++++++++++++++++++++++++++++
      goal_arrival_flag function
  ++++++++++++++++++++++++++++++++++
  True when within 1.0m of the goal waypoint.
  """
  def goal_arrival_flag(self, waypoints, num, d):
    coordinates = self.get_coordinate_from_tf(self.now)
    dist = math.sqrt((coordinates["x"] - waypoints["waypoints"][num-1]["point"]["x"]) ** 2 + (coordinates["y"] - waypoints["waypoints"][num-1]["point"]["y"]) ** 2)
    print("\r" + "\033[34m" + "Distance from waypoint: " + str(dist) + "\033[0m", end = "")
    if dist < d: # 1.0m
      print("")
      print("\033[32m" + "Waypoint" + str(num) + " reached!!" + "\033[0m")
      self.client.wait_for_server()
      self.client.cancel_all_goals()
      return True
    else:
      return False
      
  
  """
  ++++++++++++++++++++++++++++++++++
            main function
  ++++++++++++++++++++++++++++++++++
  """ 
  def main(self):
    # get gps waypoints from yaml file
    waypoints = self.load_waypoint(self.waypoints_file)
    rate = rospy.Rate(10.0)
    start_msg = """
###################
     started!!
###################
"""
    print("\033[35m" + start_msg + "\033[0m")
    while not self.goal_arrival_flag(waypoints, 1, 2):
      print("ok")
      self.pub_goal(waypoints, 1)
      rospy.sleep(1)
    self.first_flag = True
    while not self.goal_arrival_flag(waypoints, 2, 0.7):
      print("ok")
      self.pub_goal(waypoints, 2)
      rospy.sleep(1)
    #while not self.goal_arrival_flag(waypoints, 1):
      #rate.sleep()
"""
++++++++++++++++++++++++++++++++++
              main
++++++++++++++++++++++++++++++++++
"""
if  __name__ == "__main__":
    # init node
    rospy.init_node("qualification_gps_data_acquisition")
    ggs = GPSGoalSetter()
    ggs.main()
    # spin
    rospy.spin()
