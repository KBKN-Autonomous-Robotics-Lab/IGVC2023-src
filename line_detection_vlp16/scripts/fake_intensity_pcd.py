#!/usr/bin/env python3

import rospy
import numpy as np
import ros_numpy
import cv2
import math
import sensor_msgs.point_cloud2 as pc2
import random
from sensor_msgs.msg import PointCloud2, PointField
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion


NODE_NAME = "fake_intensity_pcd"


class FakePcdPublisher:

    def __init__(self):
        # Subscriber, Publisher
        self.pcd_sub = rospy.Subscriber("/points_in", PointCloud2, self.pcd_callback)
        self.pose_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.states_callback)
        self.pcd_pub = rospy.Publisher("/points_out", PointCloud2, queue_size=1)
        # Parameters
        map_image_path = rospy.get_param(NODE_NAME+"/map_image_path", "")
        map_size = rospy.get_param(NODE_NAME+"/map_size", "43 37") # "x y"
        origin = rospy.get_param(NODE_NAME+"/origin", "0 0") # "x y"
        self.model_name = rospy.get_param(NODE_NAME+"/urdf_model_name", "")
        self.sensor_h = rospy.get_param(NODE_NAME+"/sensor_height", 1.0)
        # Other variables
        self.map_image = cv2.imread(map_image_path)
        self.map_size = list(map(float, map_size.split()))
        self.origin = list(map(float, origin.split()))
        self.img_height, self.img_width = self.map_image.shape[0:2]
        self.scale = [self.img_width/self.map_size[0], self.img_height/self.map_size[1]]  # [x, y] (pix/m)
        self.img_origin = [self.img_width/2-self.origin[0]*self.scale[0], self.img_height/2+self.origin[1]*self.scale[1]] #[x, y] (pix)
        self.robot_pose = Pose()
        self.model_idx = -1
    


    def pcd_callback(self, msg: PointCloud2):
        # ----- Get original pointcloud data -----
        pcd_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        # pcd_array: 1D array: [(x1,y1,z1,i1,?,?), (x2,y2,z2,i2,?,?), ...]
        X = np.zeros(len(pcd_array), dtype=np.float32)
        Y = np.zeros_like(X)
        Z = np.zeros_like(X)
        for i, p in enumerate(pcd_array):
            X[i], Y[i], Z[i] = p[0], p[1], p[2]
        
        # ----- Add fake intensity data and publish -----
        img_X, img_Y = self.local_to_image(X, Y)
        R, C = np.round(img_Y).astype(np.uint16), np.round(img_X).astype(np.uint16)
        I = np.array(list(map(self.calc_intensity, R, C, Z)), dtype=np.float32)
        self.publish_fake_pcd(msg.header, X, Y, Z, I)
        return



    def states_callback(self, msg: ModelStates):
        # ----- Store the robot's position in the simulator -----
        if (self.model_idx == -1):
            self.model_idx = msg.name.index(self.model_name)
        self.robot_pose = msg.pose[self.model_idx]
        return
    


    def local_to_image(self, x, y):
        # local -> global
        p = self.robot_pose.position
        q = self.robot_pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        global_x = x*math.cos(yaw) - y*math.sin(yaw) + p.x
        global_y = x*math.sin(yaw) + y*math.cos(yaw) + p.y
        # global -> image
        img_x = self.img_origin[0] + global_x*self.scale[0]
        img_y = self.img_origin[1] - global_y*self.scale[1]
        return img_x, img_y
    


    def calc_intensity(self, r, c, z):
        # ----- Create fake intensity data from map image -----
        if (r < 0) or (self.img_height <= r) or (c < 0) or (self.img_width <= c):
            return 0
        elif (self.sensor_h + z) > 0.1:
            return random.uniform(0, 255)
        return np.mean(self.map_image[r, c, :])
    


    def publish_fake_pcd(self, header, X, Y, Z, I):
        # ----- Create pointcloud data and publish -----
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        points = np.zeros((X.shape[0], 4))
        for i in range(0, points.shape[0]):
            points[i, 0:4] = X[i], Y[i], Z[i], I[i]
        
        self.pcd_pub.publish(pc2.create_cloud(header, fields, points))
        return




if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)

    fake_pcd = FakePcdPublisher()

    rospy.sleep(0.1)

    rospy.spin()