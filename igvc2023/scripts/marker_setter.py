#!/usr/bin/env python3
# maintainer:kbkn/mori

import rospy
import ruamel.yaml
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

topic = "visualization_marker_array"
publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

rospy.init_node("marker_setter")
markerArray = MarkerArray()

count = 0
num = 0
MARKERS_MAX = 4
dist = []

yaml = ruamel.yaml.YAML()
with open(rospy.get_param("waypoints_file")) as file:
    waypoints = yaml.load(file)

while not rospy.is_shutdown():
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = waypoints["waypoints"][num]["point"]["x"]
    marker.pose.position.y = waypoints["waypoints"][num]["point"]["y"]
    marker.pose.position.z = 0.0

    if(count > MARKERS_MAX):
        markerArray.markers.pop(0)

    markerArray.markers.append(marker)

    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    publisher.publish(markerArray)

    count += 1

    if num == 3:
        num = 0
    else:
        num += 1

    rospy.sleep(0.01)
