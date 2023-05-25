#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import actionlib
import math
import cv2
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction
from cv_bridge import CvBridge



NODE_NAME = "lane_follower"


class LaneFollower:

    def __init__(self):
        # Subscriber, Publisher
        self.costmap_sub = rospy.Subscriber("move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback)
        self.lane_yaw_sub = rospy.Subscriber("/lane_yaw", Float32, self.lane_yaw_callback)
        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        self.image_pub = rospy.Publisher("costmap_iamge", Image, queue_size=1)
        # ROS Parameters
        self.min_angle = rospy.get_param(NODE_NAME+"/min_angle", -math.pi/2)
        self.max_angle = rospy.get_param(NODE_NAME+"/max_angle", math.pi/2)
        self.angle_num = rospy.get_param(NODE_NAME+"/search_angle_num", 9)
        self.window = rospy.get_param(NODE_NAME+"/search_window", 3)
        self.max_range = rospy.get_param(NODE_NAME+"/search_max_range", 3)
        self.init_goal_x = rospy.get_param(NODE_NAME+"/init_goal_x", 3) # Initial goal's coordinate from robot frame (m)
        self.goal_tolerance = rospy.get_param(NODE_NAME+"/goal_tolerance", 2) # (m)
        # Other variables
        self.current_goal = [self.init_goal_x, 0]
        self.next_goal = []
        self.costmap_info = MapMetaData()
        self.init_flag = True
        self.refer_idx = ()
        self.search_angles = []
        self.lane_yaw = 0.0
        self.stack_flag = False
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.cv_bridge = CvBridge()
        self.t_counter = TimeCounter(10)
    


    ##########  Costmap callback function  ##########
    def costmap_callback(self, msg: OccupancyGrid):
        # ----- Listen tf map -> robot_frame -----
        try:
            stamp = msg.header.stamp
            tf = self.tfBuffer.lookup_transform("map", "base_footprint", stamp, rospy.Duration(1.0))
            (_, _, robot_yaw) = euler_from_quaternion(
                [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
            robot_pose = [tf.transform.translation.x, tf.transform.translation.y, robot_yaw]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            return
        
        # ----- Only first callback -----
        if self.init_flag:
            # Go to initial goal
            goal_th = math.atan2(self.current_goal[1], self.current_goal[0])
            goal_msg = self.new_goal_msg(self.current_goal[0], self.current_goal[1], goal_th, robot_pose)
            self.goal_pub.publish(goal_msg)
            # Initialize
            self.costmap_info = msg.info
            self.refer_idx, self.search_angles = self.get_reference_idx()
            self.init_flag = False

        # ----- Convert costmap msg to image format -----
        width, height = msg.info.width, msg.info.height
        costmap = np.zeros((width,height), dtype=np.int8)
        for r in range(0, height):
            costmap[width-r-1,:] = msg.data[r*width:(r+1)*width]

        np.place(costmap, costmap<0, 0)
        costmap = costmap.astype(np.uint8)

        # ----- Rotate costmap image to match robot orientation -----
        rpx, rpy, rpth = robot_pose
        rot_ang = 90 - math.degrees(rpth)
        rot_mat = cv2.getRotationMatrix2D(center=(width/2, height/2), angle=rot_ang, scale=1)
        rot_costmap = cv2.warpAffine(costmap, rot_mat, (width, height))
        rgb_costmap = cv2.cvtColor(rot_costmap, cv2.COLOR_GRAY2RGB)
        
        # ----- Search next goal positions -----
        goal_img_rc = self.global_to_image(self.current_goal[0], self.current_goal[1], robot_pose)
        scores = [0] * len(self.search_angles)
        for i, th in enumerate(self.search_angles):
            scores[i] = self.calc_score(goal_img_rc, i, rot_costmap)
            # For visualize
            x = np.round(goal_img_rc[1] - math.sin(th)*scores[i]).astype(np.uint16)
            y = np.round(goal_img_rc[0] - math.cos(th)*scores[i]).astype(np.uint16)
            rgb_costmap = cv2.line(rgb_costmap, (goal_img_rc[1],goal_img_rc[0]), (x,y), (0,100,0), 1)
        
        # ----- Get next goal -----
        max_arg = np.argmax(scores)
        angle = self.search_angles[max_arg]
        reduction = 0.7
        score = scores[max_arg] * reduction
        if score > 3:
            resolution = self.costmap_info.resolution
            local_goal_x = (height/2 - goal_img_rc[0] + score*math.cos(angle)) * resolution
            local_goal_y = (width/2 - goal_img_rc[1] + score*math.sin(angle)) * resolution
            r = math.dist([0, 0], [local_goal_x, local_goal_y])
            if r > height/2:
                local_goal_x = local_goal_x/r * height/2
                local_goal_y = local_goal_y/r * height/2
            next_goal_x = local_goal_x*math.cos(rpth) - local_goal_y*math.sin(rpth) + rpx
            next_goal_y = local_goal_x*math.sin(rpth) + local_goal_y*math.cos(rpth) + rpy
            if len(self.next_goal) == 0:
                self.next_goal = [next_goal_x, next_goal_y]
            else:
                self.next_goal = [(self.next_goal[0]+next_goal_x)/2, (self.next_goal[1]+next_goal_y)/2]
            # ----- When distance to current goal is less than goal_torelance -----
            if (math.dist(robot_pose[:2], self.current_goal) < self.goal_tolerance) or self.stack_flag:
                goal_msg = PoseStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = self.next_goal[0]
                goal_msg.pose.position.y = self.next_goal[1]
                goal_msg.pose.position.z = 0
                q = quaternion_from_euler(0, 0, math.atan2((self.next_goal[1]-rpy), (self.next_goal[0]-rpx)))
                goal_msg.pose.orientation.x = q[0]
                goal_msg.pose.orientation.y = q[1]
                goal_msg.pose.orientation.z = q[2]
                goal_msg.pose.orientation.w = q[3]
                self.client.wait_for_server()
                self.goal_pub.publish(goal_msg)
                self.current_goal = self.next_goal
                self.stack_flag = False

        else:
            print("All scores are zero.")
            gx, gy = self.current_goal
            lgx = (gx-rpx)*math.cos(-rpth) - (gy-rpy)*math.sin(-rpth)
            lgy = (gx-rpx)*math.sin(-rpth) + (gy-rpy)*math.cos(-rpth)
            lgx, lgy = lgx*0.8, lgy*0.8
            self.current_goal[0] = lgx*math.cos(rpth) - lgy*math.sin(rpth) + rpx
            self.current_goal[1] = lgx*math.sin(rpth) + lgy*math.cos(rpth) + rpy
            self.client.wait_for_server()
            self.client.cancel_all_goals()
            self.stack_flag = True
            

        # ----- Publish Image -----
        rgb_costmap = cv2.circle(rgb_costmap, (goal_img_rc[1], goal_img_rc[0]), 3, (0,0,100), thickness=-1)
        n_goal_img_rc = self.global_to_image(self.next_goal[0], self.next_goal[1], robot_pose)
        rgb_costmap = cv2.circle(rgb_costmap, (n_goal_img_rc[1], n_goal_img_rc[0]), 3, (100,0,0), thickness=-1)
        self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg((rgb_costmap/100*255).astype(np.uint8)))
        return
    


    ##########  Caluculate scores for how far robot can move in seach_angles  ##########
    def calc_score(self, goal_img_rc, angle_idx, image):
        ref_idx_r = goal_img_rc[0] + self.refer_idx[0][angle_idx]
        ref_idx_c = goal_img_rc[1] + self.refer_idx[1][angle_idx]
        window = 3
        image[goal_img_rc[0]-window:goal_img_rc[0]+window, goal_img_rc[1]-window:goal_img_rc[1]+window] = 0
        score = self.max_range/self.costmap_info.resolution
        for i in range(0, len(ref_idx_r)):
            r, c = ref_idx_r[i], ref_idx_c[i]
            if ((r-window < 0) or (c-window < 0) or (c+window >= self.costmap_info.width)
                or (np.sum(image[r-window:r+window, c-window:c+window]) > 0)):
                score = min([score, math.dist(goal_img_rc, [r, c])])
                score -= abs(self.lane_yaw - self.search_angles[angle_idx])*3
        return max([score, 0])



    ##########  Get search line index in costmap image  ##########
    def get_reference_idx(self):
        increment = (self.max_angle-self.min_angle)/(self.angle_num-1)
        search_angles = [self.min_angle+increment*i for i in range(0, self.angle_num)]
        width, height = self.costmap_info.width, self.costmap_info.height
        center = (round(width/2), round(height/2))
        max_pix_range = self.max_range / self.costmap_info.resolution

        ref_idx_r, ref_idx_c = [], []
        for th in search_angles:
            x = center[0] - round(max_pix_range * math.sin(th))
            y = center[1] - round(max_pix_range * math.cos(th))
            img = np.zeros((width, height), np.uint8)
            line_img = cv2.line(img, center, (x,y), 255, 1)
            idx = np.where(line_img==255)
            ref_idx_r.append(idx[0] - center[0])
            ref_idx_c.append(idx[1] - center[1])

        return (ref_idx_r, ref_idx_c), search_angles



    ##########  Make goal message for move_base  ##########
    def new_goal_msg(self, lgx, lgy, lgth, rp):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        # ----- local goal xy => global xy -----
        rpx, rpy, rpth = rp
        goal_msg.pose.position.x = lgx*math.cos(rpth) - lgy*math.sin(rpth) + rpx
        goal_msg.pose.position.y = lgx*math.sin(rpth) + lgy*math.cos(rpth) + rpy
        goal_msg.pose.position.z = 0
        q = quaternion_from_euler(0, 0, lgth+rpth)
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]
        return goal_msg
    


    ##########  Convert global xy to image rc  ##########
    def global_to_image(self, gx, gy, rp):
        # ----- global => local -----
        rpx, rpy, rpth = rp
        lgx = (gx-rpx)*math.cos(-rpth) - (gy-rpy)*math.sin(-rpth)
        lgy= (gx-rpx)*math.sin(-rpth) + (gy-rpy)*math.cos(-rpth)
        # ----- local => cv image xy -----
        width, height = self.costmap_info.width, self.costmap_info.height
        resolution = self.costmap_info.resolution
        x = width/2 - lgy/resolution
        y = height/2 - lgx/resolution
        return round(y), round(x)
    


    def lane_yaw_callback(self, msg):
        self.lane_yaw = msg.data





import time

class TimeCounter:
    def __init__(self, count_time) -> None:
        self.counter = 0
        self.start_time = 0
        self.sum_time = 0
        self.count_time = count_time
    
    def start(self):
        self.start_time = time.perf_counter()
    
    def end(self):
        end_time = time.perf_counter()
        self.sum_time += (end_time - self.start_time)
        self.counter += 1
        if self.counter == self.count_time:
            t = self.sum_time/self.counter
            print(self.counter, "Times average:", '{:.5f}'.format(t), "(secs)")
            self.sum_time = 0
            self.counter = 0





if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=True)

    follower = LaneFollower()

    rospy.sleep(0.5)

    rospy.spin()