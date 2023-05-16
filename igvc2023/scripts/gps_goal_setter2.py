#!/usr/bin/env python3
# maintainer:kbkn/mori
"""
******************************************
This node gives the goal to "move_base" 
that the robot should head for based on 
"gps_waypoints.yaml".
******************************************
"""
import rospy
import tf2_ros
import math
import sys
import ruamel.yaml
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class GPSGoalSetter:
  def __init__(self):
    self.waypoints_file = rospy.get_param("waypoints_file")
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
    # true for the first time in loop processing, False for the second and subsequent times
    self.flag = True
    self.now = rospy.Time()

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
    # IGVC2022 has 4 pre-defined gps waypoint
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
        tf = self.tfBuffer.lookup_transform("map", "base_footprint", stamp, rospy.Duration(1.0))
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
    return coordinates

  """
  ++++++++++++++++++++++++++++++++++
        active_flag function
  ++++++++++++++++++++++++++++++++++
  Execute this node if return value is True.
  """
  def active_flag(self):
    # node_flag == 1 then activate local_goal_setter node
    # node_flag == 2 then activate this node
    # node_flag == 3 then activate local_goal_setter node for line detection at ramp entries
    # node_flag == 4 then active ramp detect node
    # node_flag == 5 then disable all node
    node_array = str(rospy.get_param("node_array",1)) #+
    node_index = rospy.get_param("node_index",1) #+
    node_flag = int(node_array[node_index-1]) #+
    #node_flag = rospy.get_param("node_flag", 1)
    if node_flag == 2 and self.flag == True:
      self.flag = False
      return True
    elif node_flag == 2 and self.flag == False:
      return False
    else:
      self.flag = True
      return False

  """
  ++++++++++++++++++++++++++++++++++
          pub_goal function
  ++++++++++++++++++++++++++++++++++
  Publish goal to move_base.
  The robot's finish pose will match the robot's pose when the goal is published
  """
  def pub_goal(self, waypoints, num):
    coordinates = self.get_coordinate_from_tf(self.now)
    (roll, pitch, yaw) = euler_from_quaternion([coordinates["rx"], coordinates["ry"], coordinates["rz"], coordinates["rw"]])
    roll = pitch = 0.0
    q = quaternion_from_euler(roll, pitch, yaw)
    goal_msg = PoseStamped()
    goal_msg.header.seq = 1
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    goal_msg.pose.position.x = waypoints["waypoints"][num-1]["point"]["x"]
    goal_msg.pose.position.y = waypoints["waypoints"][num-1]["point"]["y"]
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.x = q[0]
    goal_msg.pose.orientation.y = q[1]
    goal_msg.pose.orientation.z = q[2]
    goal_msg.pose.orientation.w = q[3]
    print("\033[32m" + "Go to waypoint" + str(num) + "..." + "\033[0m")
    for _ in range(3):
      self.goal_pub.publish(goal_msg)
      rospy.sleep(1)

  """
  ++++++++++++++++++++++++++++++++++
      select_first_goal function
  ++++++++++++++++++++++++++++++++++
  Calculate the first goal waypoint.
  """
  def select_first_goal(self, waypoints):
    coordinates = self.get_coordinate_from_tf(self.now)
    dist = []
    for i in range(len(waypoints["waypoints"])):
      dist.append(math.sqrt((coordinates["x"] - waypoints["waypoints"][i]["point"]["x"]) ** 2 + (coordinates["y"] - waypoints["waypoints"][i]["point"]["y"]) ** 2))
    if dist.index(min(dist)) == 0:
      waypoints_goal_num = 1
      return waypoints_goal_num
    elif dist.index(min(dist)) == 1:
      waypoints_goal_num = 2
      return waypoints_goal_num
    elif dist.index(min(dist)) == 2:
      waypoints_goal_num = 3
      return waypoints_goal_num
    else:
      waypoints_goal_num = 4
      return waypoints_goal_num

  """
  ++++++++++++++++++++++++++++++++++
      select_second_goal function
  ++++++++++++++++++++++++++++++++++
  Calculate the second goal waypoint.
  """
  def select_second_goal(self, waypoints):
    coordinates = self.get_coordinate_from_tf(self.now)
    dist = []
    for i in range(len(waypoints["waypoints"])):
      dist.append(math.sqrt((coordinates["x"] - waypoints["waypoints"][i]["point"]["x"]) ** 2 + (coordinates["y"] - waypoints["waypoints"][i]["point"]["y"]) ** 2))
    if dist.index(min(dist)) == 0:
      waypoints_goal_num = 2
      return waypoints_goal_num
    elif dist.index(min(dist)) == 1:
      waypoints_goal_num = 1
      return waypoints_goal_num
    elif dist.index(min(dist)) == 2:
      waypoints_goal_num = 4
      return waypoints_goal_num
    else:
      waypoints_goal_num = 3
      return waypoints_goal_num

  """
  ++++++++++++++++++++++++++++++++++
      goal_arrival_flag function
  ++++++++++++++++++++++++++++++++++
  True when within 1.0m of the goal waypoint.
  """
  def goal_arrival_flag(self, waypoints, num, distance):
    coordinates = self.get_coordinate_from_tf(self.now)
    dist = math.sqrt((coordinates["x"] - waypoints["waypoints"][num-1]["point"]["x"]) ** 2 + (coordinates["y"] - waypoints["waypoints"][num-1]["point"]["y"]) ** 2)
    print("\r" + "\033[34m" + "Distance from waypoint" + str(num) + ": " + str(dist) + "\033[0m", end = "")
    if dist < distance:
      print("")
      print("\033[32m" + "Waypoint" + str(num) + " reached!!" + "\033[0m")
      return True
    else:
      return False

  """
  ++++++++++++++++++++++++++++++++++
      update_node_index function
  ++++++++++++++++++++++++++++++++++
  """
  def update_node_index(self):
    node_index = rospy.get_param("node_index",1)
    node_index += 1 #+
    rospy.set_param("node_index", node_index) #+

  """
  ++++++++++++++++++++++++++++++++++
            main function
  ++++++++++++++++++++++++++++++++++
  """ 
  def main(self):
    # get gps waypoints from yaml file
    waypoints = self.load_waypoint(self.waypoints_file)
    while not rospy.is_shutdown():
      if self.active_flag():
        start_msg = """
###################
  Node2 started!!
###################
"""
        print("\033[35m" + start_msg + "\033[0m")
        waypoints_goal_num = self.select_first_goal(waypoints)
        self.pub_goal(waypoints, waypoints_goal_num)
        while not self.goal_arrival_flag(waypoints, waypoints_goal_num, 1.0):
          rospy.sleep(1)
        waypoints_goal_num = self.select_second_goal(waypoints)
        self.pub_goal(waypoints, waypoints_goal_num)
        while not self.goal_arrival_flag(waypoints, waypoints_goal_num,1.0):
          rospy.sleep(1)
        self.update_node_index()
      rospy.sleep(1)
"""
++++++++++++++++++++++++++++++++++
              main
++++++++++++++++++++++++++++++++++
"""
if  __name__ == "__main__":
    # init node
    rospy.init_node("gps_data_acquisition")
    ggs = GPSGoalSetter()
    ggs.main()
    # spin
    rospy.spin()
