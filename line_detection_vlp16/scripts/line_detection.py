#!/usr/bin/env python3

import rospy
import numpy as np
import math
import pcd_tools
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server
from line_detection_vlp16.cfg import reconfigConfig


NODE_NAME = "line_detection"




class LineDetector:
    
    def __init__(self):
        # Subscriber, Publisher
        self.pcd_sub = rospy.Subscriber("/points_in", PointCloud2, self.pcd_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.ground_pub = rospy.Publisher("/ground_points", PointCloud2, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacle_points", PointCloud2, queue_size=1)
        self.lane_pub = rospy.Publisher("/lane_points", PointCloud2, queue_size=1)
        self.lane_yaw_pub = rospy.Publisher("/lane_yaw", Float32, queue_size=1)
        # ROS Parameters
        self.sensor_h = rospy.get_param(NODE_NAME+"/sensor_height", 1.0)
        self.v_sample_num = rospy.get_param(NODE_NAME+"/v_sample_num", 3)
        self.angle_width = rospy.get_param(NODE_NAME+"/angle_width", math.pi/2)
        self.max_slope = rospy.get_param(NODE_NAME+"/max_slope", 0.1)
        self.max_height = rospy.get_param(NODE_NAME+"/max_height", 0.2)
        self.intensity_thresh = rospy.get_param(NODE_NAME+"/intensity_threshold", 200)
        self.predict = rospy.get_param(NODE_NAME+"/predition", False)
        # 3D-LiDAR parameters
        self.v_angle_partition = list(map(math.radians, range(-16, 16, 2)))[:self.v_sample_num+1]
        self.h_resolution = 512
        self.h_angle_partition = np.linspace(math.pi, -math.pi, self.h_resolution+1) # descend order (clock wise)
        self.sigma = 0.1
        # Others
        self.odom_yaw = [0, 0] # pre & current yaw
        self.lane_dir = 0.0
        self.lane_width_ang = np.zeros(self.v_sample_num, dtype=np.float16)
    


    def odom_callback(self, msg: Odometry):
        self.odom_yaw[0] = self.odom_yaw[1]
        q = msg.pose.pose.orientation
        (_, _, self.odom_yaw[1]) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    


    def pcd_callback(self, msg: PointCloud2):
        # ----- Read message and store x, y, z, intensity -----
        X, Y, Z, I = pcd_tools.read_from_msg(msg)
        
        # ----- Distance and Direction of each points -----
        ranges = np.array(list(map(lambda x,y: math.dist([x,y],[0,0]), X, Y)))
        pitch = np.array(list(map(lambda z,r: math.atan2(z,r), Z, ranges)))
        yaw = np.array(list(map(lambda x,y: math.atan2(y,x), X, Y)))
        
        # ----- Organize pointcloud & ground segmentation -----
        # matrices stored organized pointcloud
        X_mat = np.zeros((self.v_sample_num, self.h_resolution), dtype=np.float16)
        Y_mat = np.zeros_like(X_mat)
        Z_mat = np.zeros_like(X_mat)
        I_mat = np.zeros_like(X_mat)
        range_mat = np.zeros_like(X_mat)
        exist_mat = np.full_like(X_mat, True, dtype=np.bool)
        # horizontal angle to extract
        angle_max = min(self.lane_dir+self.angle_width/2, self.h_angle_partition[0])
        angle_min = max(self.lane_dir-self.angle_width/2, self.h_angle_partition[-1])
        max_idx = np.where(self.h_angle_partition < angle_max)[0][0]
        min_idx = np.where(self.h_angle_partition > angle_min)[0][-1]
        # ground segmentation flag
        ground_mat = np.full_like(X_mat, True, dtype=np.bool) # True:ground, False:obstacle

        # organize
        for hi in range(max_idx, min_idx+1):
            h_idx = (self.h_angle_partition[hi+1] < yaw) & (yaw < self.h_angle_partition[hi])
            for i, vi in enumerate(range(self.v_sample_num-1, -1, -1)):
                idx = h_idx & (self.v_angle_partition[i] < pitch) & (pitch < self.v_angle_partition[i+1])
                if np.any(idx):
                    X_mat[vi, hi] = X[idx][0]
                    Y_mat[vi, hi] = Y[idx][0]
                    Z_mat[vi, hi] = Z[idx][0]
                    I_mat[vi, hi] = I[idx][0]
                    range_mat[vi, hi] = ranges[idx][0]
                else:
                    exist_mat[vi, hi] = False
                    continue
                # ground segmentaion
                # by height
                z = Z_mat[vi, hi]
                ground_mat[vi, hi] = (self.sensor_h + z) < self.max_height
                # by difference in distance from neighboring pointcloud
                r = range_mat[vi, hi]
                if (hi > 0):
                    if (range_mat[vi, hi-1] - r) > self.sigma*2:
                        ground_mat[vi, hi] = False
                    if (r - range_mat[vi, hi-1]) > self.sigma*2:
                        ground_mat[vi, hi-1] = False
                # by slope angle
                if (vi != (self.v_sample_num-1)) and exist_mat[vi+1, hi]:
                    slope = math.atan2(z-Z_mat[vi+1, hi], r-range_mat[vi+1, hi])
                    # distance from previous vertical angle point
                    ground_mat[vi+1, hi] = ground_mat[vi+1, hi] & (abs(range_mat[vi+1,hi] - r) > self.sigma)
                else:
                    slope = math.atan2(self.sensor_h+z, r)
                ground_mat[vi, hi] =  ground_mat[vi, hi] & (abs(slope) < self.max_slope)
        

        # ----- Detect lane -----
        obs_mat = np.logical_not(ground_mat)
        I_mat[obs_mat] = -1
        roi_X_mat = X_mat[:, max_idx:min_idx+1]
        roi_Y_mat = Y_mat[:, max_idx:min_idx+1]
        roi_Z_mat = Z_mat[:, max_idx:min_idx+1]
        roi_I_mat = I_mat[:, max_idx:min_idx+1]
        roi_r_mat = range_mat[:, max_idx:min_idx+1]
        roi_yaw = self.h_angle_partition[max_idx:min_idx+1]
        detect_mat = roi_I_mat > self.intensity_thresh
        center_dir = np.zeros(self.v_sample_num)
        dir = self.lane_dir
        l_line, r_line = 0, 0


        lane_pcd = np.zeros((self.v_sample_num*2, 4))

        for vi in range(self.v_sample_num-1, -1, -1):
            # detect LR line
            center_idx = np.where(roi_yaw <= dir)[0][0]
            l_line = np.where(detect_mat[vi, :center_idx])[0]
            r_line = np.where(detect_mat[vi, center_idx:])[0]
            on_left = (len(l_line) > 0)
            on_right = (len(r_line) > 0)
            if on_left and on_right:
                left = roi_yaw[:center_idx][l_line[-1]]
                right = roi_yaw[center_idx:][r_line[0]]
                center_dir[vi] = (left + right)/2
                lane_width = left - right
                self.lane_width_ang[vi] = lane_width
                # lane pcd
                lane_pcd[(vi+1)*2-1, 0] = roi_X_mat[vi, :center_idx][l_line[-1]]
                lane_pcd[(vi+1)*2-1, 1] = roi_Y_mat[vi, :center_idx][l_line[-1]]
                lane_pcd[(vi+1)*2-1, 2] = roi_Z_mat[vi, :center_idx][l_line[-1]]
                lane_pcd[(vi+1)*2-1, 3] = 0
                lane_pcd[vi*2, 0] = roi_X_mat[vi, center_idx:][r_line[0]]
                lane_pcd[vi*2, 1] = roi_Y_mat[vi, center_idx:][r_line[0]]
                lane_pcd[vi*2, 2] = roi_Z_mat[vi, center_idx:][r_line[0]]
                lane_pcd[vi*2, 3] = 255

            elif on_left:
                left = roi_yaw[:center_idx][l_line[-1]]
                right = left - self.lane_width_ang[vi]
                center_dir[vi] = (left + right)/2
                # lane pcd
                lane_pcd[(vi+1)*2-1, 0] = roi_X_mat[vi, :center_idx][l_line[-1]]
                lane_pcd[(vi+1)*2-1, 1] = roi_Y_mat[vi, :center_idx][l_line[-1]]
                lane_pcd[(vi+1)*2-1, 2] = roi_Z_mat[vi, :center_idx][l_line[-1]]
                lane_pcd[(vi+1)*2-1, 3] = 0
                if self.predict:
                    r = roi_r_mat[vi, :center_idx][l_line[-1]]
                    lane_pcd[vi*2, :2] = r*math.cos(right), r*math.sin(right)
                    lane_pcd[vi*2, 2] = lane_pcd[(vi+1)*2-1, 2]
                    lane_pcd[vi*2, 3] = 255

            elif on_right:
                right = roi_yaw[center_idx:][r_line[0]]
                left = right + self.lane_width_ang[vi]
                center_dir[vi] = (left + right)/2
                # lane pcd
                lane_pcd[vi*2, 0] = roi_X_mat[vi, center_idx:][r_line[0]]
                lane_pcd[vi*2, 1] = roi_Y_mat[vi, center_idx:][r_line[0]]
                lane_pcd[vi*2, 2] = roi_Z_mat[vi, center_idx:][r_line[0]]
                lane_pcd[vi*2, 3] = 255
                if self.predict:
                    r = roi_r_mat[vi, center_idx:][r_line[0]]
                    lane_pcd[(vi+1)*2-1, :2] = r*math.cos(left), r*math.sin(left)
                    lane_pcd[(vi+1)*2-1, 2] = lane_pcd[vi*2, 2]
                    lane_pcd[(vi+1)*2-1, 3] = 0

            else:
                center_dir[vi] = dir
            dir = center_dir[vi]

        # lane direction
        dir1 = np.mean(center_dir) # detect from lane
        dir2 = self.lane_dir - (self.odom_yaw[1] - self.odom_yaw[0]) # detect from odom change
        lane_dir = (dir1 + dir2)/2
        if abs(self.lane_dir - lane_dir) < 0.1:
            self.lane_dir = lane_dir
        

        # ----- Publish -----
        pub_idx = exist_mat & ground_mat
        X, Y, Z, I = X_mat[pub_idx], Y_mat[pub_idx], Z_mat[pub_idx], I_mat[pub_idx]
        self.ground_pub.publish(pcd_tools.create_pcd_msg(msg.header, X, Y, Z, I))

        pub_idx = exist_mat & obs_mat
        X, Y, Z, I = X_mat[pub_idx], Y_mat[pub_idx], Z_mat[pub_idx], I_mat[pub_idx]
        self.obstacle_pub.publish(pcd_tools.create_pcd_msg(msg.header, X, Y, Z, I))

        pub_idx = np.logical_not(np.all(lane_pcd==0, axis=1))
        X, Y, Z, I = lane_pcd[pub_idx,0], lane_pcd[pub_idx,1], lane_pcd[pub_idx,2], lane_pcd[pub_idx,3]
        self.lane_pub.publish(pcd_tools.create_pcd_msg(msg.header, X, Y, Z, I))
        return
    

    def reconfigCallback(self, config, level):
        self.angle_width = config.angle_width
        self.intensity_thresh = config.intensity_threshold
        self.max_slope = config.max_slope
        self.max_height = config.max_height
        return config





if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)

    detector = LineDetector()
    recfg_srv = Server(reconfigConfig, detector.reconfigCallback)
    rospy.sleep(0.5)
    rospy.spin()