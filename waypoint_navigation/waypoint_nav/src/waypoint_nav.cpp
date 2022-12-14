#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>

#include <yaml-cpp/yaml.h>
#include <waypoint_saver.h>  // include Waypoint class

#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <math.h>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif


class SwitchRunningStatus : public std::exception {
public:
    SwitchRunningStatus() : std::exception() { }
};



class WaypointsNavigation{
public:
    /*
    ++++++++++ Constructer ++++++++++
    */
    WaypointsNavigation() :
        has_activate_(false),
        stopped_(false),
        move_base_action_("move_base", true),
        rate_(10),
        last_moved_time_(0),
        dist_err_(1.0),
        min_dist_err_(0.3),
        min_yaw_err_(0.3)
    {
        while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
        {
            ROS_INFO("Waiting...");
        }
        ros::NodeHandle private_nh("~");
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("world_frame", world_frame_, std::string("map"));
        
        double max_update_rate;
        private_nh.param("max_update_rate", max_update_rate, 10.0);
        rate_ = ros::Rate(max_update_rate);
        std::string filename = "";
        private_nh.param("filename", filename, filename);
        if (filename != "") {
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            if (!readFile(filename)) {
                ROS_ERROR("Failed loading waypoints file");
            } else {
                finish_pose_ = waypoints_.poses.end()-1;
                computeWpOrientation();
            }
            current_waypoint_ = waypoints_.poses.begin();
            wp_num_.data = 1;
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }
        
        //Scripts of "start from the middle"
        //Edited mori 2022/10/19
        //#############################################################
        StartFromTheMiddle = false;
        private_nh.getParam("StartFromTheMiddle", StartFromTheMiddle);
        //#############################################################
        
        private_nh.param("min_dist_err", min_dist_err_, min_dist_err_);
        private_nh.param("min_yaw_err", min_yaw_err_, min_yaw_err_);

        ros::NodeHandle nh;
        start_server_ = nh.advertiseService("start_wp_nav", &WaypointsNavigation::startNavigationCallback, this);
        stop_server_ = nh.advertiseService("stop_wp_nav", &WaypointsNavigation::stopNavigationCallback, this);
        resume_server_ = nh.advertiseService("resume_nav", &WaypointsNavigation::resumeNavigationCallback, this);
        cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &WaypointsNavigation::cmdVelCallback, this);
        scan_sub_ = nh.subscribe("scan", 1, &WaypointsNavigation::laserscan_callback, this);
        wp_pub_ = nh.advertise<geometry_msgs::PoseArray>("waypoints", 10);
        max_vel_pub_ = nh.advertise<std_msgs::Float32>("max_vel", 5);
        wp_num_pub_ = nh.advertise<std_msgs::UInt16>("waypoint_num", 5);
        clear_costmaps_srv_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    }



    /*
    ++++++++++ Read waypoints file ++++++++++
    */
    bool readFile(const std::string &filename)
    {
        waypoints_.poses.clear();
        try {
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if (ifs.good() == false) return false;

            YAML::Node node;
            #ifdef NEW_YAMLCPP
                node = YAML::Load(ifs);
            #else
                YAML::Parser parser(ifs);
                parser.GetNextDocument(node);
            #endif

            // Read waypoints from yaml file
            #ifdef NEW_YAMLCPP
                const YAML::Node &wp_node_tmp = node["waypoints"];
                const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
            #else
                const YAML::Node *wp_node = node.FindValue("waypoints");
            #endif
            geometry_msgs::Pose pose;
            Waypoint point;
            if (wp_node != NULL) {
                for(int i=0; i < wp_node->size(); i++) {
                    (*wp_node)[i]["point"]["x"] >> pose.position.x;
                    (*wp_node)[i]["point"]["y"] >> pose.position.y;
                    (*wp_node)[i]["point"]["z"] >> pose.position.z;
                    waypoints_.poses.push_back(pose);
                    point.x = pose.position.x;
                    point.y = pose.position.y;
                    point.z = pose.position.z;
                    try {
                        (*wp_node)[i]["point"]["vel"]  >> point.vel;
                    } catch(...) {} // Use default value
                    try {
                        (*wp_node)[i]["point"]["rad"]  >> point.rad;
                    } catch(...) {}
                    try {
                        (*wp_node)[i]["point"]["stop"] >> point.stop;
                    } catch(...) {}
                    waypoint_list_.push_back(point);
                }
            } else {
                return false;
            }
            
            // Read finish_pose
            #ifdef NEW_YAMLCPP
                const YAML::Node &fp_node_tmp = node["finish_pose"];
                const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
            #else
                const YAML::Node *fp_node = node.FindValue("finish_pose");
            #endif
            if (fp_node != NULL) {
                (*fp_node)["pose"]["position"]["x"] >> pose.position.x;
                (*fp_node)["pose"]["position"]["y"] >> pose.position.y;
                (*fp_node)["pose"]["position"]["z"] >> pose.position.z;
                (*fp_node)["pose"]["orientation"]["x"] >> pose.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> pose.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> pose.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> pose.orientation.w;
                waypoints_.poses.push_back(pose);
                point.x = pose.position.x;
                point.y = pose.position.y;
                point.z = pose.position.z;
                point.stop = true;
                waypoint_list_.push_back(point);
            } else {
                return false;
            }

        } catch(YAML::ParserException &e) {
            return false;

        } catch(YAML::RepresentationException &e) {
            return false;
        }

        return true;
    }



    /*
    ++++++++++ Compute target orientation for each waypoint ++++++++++
    */
    void computeWpOrientation()
    {
        for(std::vector<geometry_msgs::Pose>::iterator it = waypoints_.poses.begin(); it != finish_pose_; it++) {
            double goal_direction = atan2((it+1)->position.y - (it)->position.y,
                                          (it+1)->position.x - (it)->position.x);
            (it)->orientation = tf::createQuaternionMsgFromYaw(goal_direction);
        }
        waypoints_.header.frame_id = world_frame_;
    }



    /*
    ++++++++++ Callback function for "StartWaypointsNavigation" button on rviz  ++++++++++
    */
    bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        if ((has_activate_) || (current_waypoint_ != waypoints_.poses.begin())) {
            ROS_WARN("Waypoint navigation is already started");
            response.success = false;
            return false;
        }
        //Scripts of "start from the middle"
        //Edited mori 2022/10/19
        //#############################################################
        wp_num_.data = 1;
        init_waypoint_ = waypoints_.poses.begin();
        compare_waypoint_ = waypoints_.poses.begin();
        double min_dist_w = std::numeric_limits<int>::max();
        if(StartFromTheMiddle){
            ROS_INFO("StartFromTheMiddle => True");
            tf::StampedTransform robot_init_pos = getRobotPosGL();
            double init_x = robot_init_pos.getOrigin().x();
            double init_y = robot_init_pos.getOrigin().y();
            double pre_waypoint_x;
            double pre_waypoint_y;
            double convert_x;
            double init_yaw = tf::getYaw(robot_init_pos.getRotation());
            double square_x;
            double square_y;
            double dist_w;
            double dir_w;
            int min_waypoint = 0;
            int i = 0;
            while(compare_waypoint_ != waypoints_.poses.end() - 1){
                i++;
                square_x = (compare_waypoint_ -> position.x - init_x) * (compare_waypoint_ -> position.x - init_x);
                square_y = (compare_waypoint_ -> position.y - init_y) * (compare_waypoint_ -> position.y - init_y);
                dist_w = sqrt(square_x + square_y);
                if(i == 1){
                    dir_w = atan2(compare_waypoint_ -> position.y, compare_waypoint_ -> position.x);
                    convert_x = (compare_waypoint_ -> position.x - init_x) * cos(-init_yaw) - (compare_waypoint_ -> position.y - init_y) * sin(-init_yaw);
                    if(abs(dir_w - init_yaw) < (90 / 180.0 * M_PI) && convert_x > 0){
                        min_dist_w = dist_w;
                        init_waypoint_ = compare_waypoint_;
                        min_waypoint = 1;
                    }
                }else if(min_dist_w > dist_w){
                    dir_w = atan2(compare_waypoint_ -> position.y - pre_waypoint_y, compare_waypoint_ -> position.x- pre_waypoint_x);
                    convert_x = (compare_waypoint_ -> position.x - init_x) * cos(-init_yaw) - (compare_waypoint_ -> position.y - init_y) * sin(-init_yaw);
                    if(abs(dir_w - init_yaw) < (90 / 180.0 * M_PI) && convert_x > 0){
                        min_dist_w = dist_w;
                        init_waypoint_ = compare_waypoint_;
                        min_waypoint = i;
                    }
                }
                pre_waypoint_x = compare_waypoint_ -> position.x;
                pre_waypoint_y = compare_waypoint_ -> position.y;
                compare_waypoint_++;
            }
            wp_num_.data = min_waypoint;
        }
        std_srvs::Empty empty;
        while(!clear_costmaps_srv_.call(empty)) {
            ROS_WARN("Resend clear costmap service");
            sleep();
        }
        if(min_dist_w == std::numeric_limits<int>::max()){
            ROS_WARN("Could not find the closest appropriate waypoint. Please check the direction of the robot again.");
            response.success = false;
            return false;
        }else{
            current_waypoint_ = init_waypoint_;
            has_activate_ = true;
            response.success = true;
            return true;
        }
        //#############################################################
    }



    /*
    ++++++++++ Callback function for "ResumeWaypointsNavigation" button on rviz  ++++++++++
    */
    bool resumeNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        if (has_activate_) {
            ROS_WARN("Navigation is already active");
            response.success = false;
        } else {
            std_srvs::Empty empty;
            clear_costmaps_srv_.call(empty);
            ROS_INFO("Navigation has resumed");
            has_activate_ = true;
            response.success = true;
        }
        return true;
    }


    /*
    ++++++++++ Callback function for service of "stop_wp_nav"  ++++++++++
    */
    bool stopNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
    {
        if (has_activate_) {
            ROS_INFO("Waypoint navigation has been stopped");
            has_activate_ = false;
            response.success = true;
        } else {
            ROS_WARN("Waypoint navigation is already stopped");
            response.success = false;
        }
        return true;
    }



    /*
    ++++++++++ Callback function for /cmd_vel topic  ++++++++++
    */
    void cmdVelCallback(const geometry_msgs::Twist &msg)
    {
        if (has_activate_ &&
            msg.linear.x > -0.001 && msg.linear.x < 0.001 &&
            msg.linear.y > -0.001 && msg.linear.y < 0.001 &&
            msg.linear.z > -0.001 && msg.linear.z < 0.001 &&
            msg.angular.x > -0.001 && msg.angular.x < 0.001 &&
            msg.angular.y > -0.001 && msg.angular.y < 0.001 &&
            msg.angular.z > -0.001 && msg.angular.z < 0.001)
        {
            // ROS_INFO("command velocity all zero");
        } else {
            last_moved_time_ = ros::Time::now().toSec();
        }
    }



    /*
    ++++++++++ Callback function for /scan topic  ++++++++++
    */
    void laserscan_callback(const sensor_msgs::LaserScan &msg)
    {
        laserscan = msg;
    }



    /*
    ++++++++++ Search for a goal without obstacles  ++++++++++
    */
    void sendNoObstacleGoal(float first_thresh, float after_second_thresh, float goal_thresh, const geometry_msgs::Pose &dest)
    {
        if(laserscan.ranges.size() > 0){
            move_base_msgs::MoveBaseGoal move_base_goal;
            tf::StampedTransform robot_pos = getRobotPosGL();
            geometry_msgs::Pose mid_waypoint;
            double start_nav_time, time;
            float sum_x, sum_y;
            float x[laserscan.ranges.size()];
            float y[laserscan.ranges.size()];
            float waypoint_x = dest.position.x - robot_pos.getOrigin().x();
            float waypoint_y = dest.position.y - robot_pos.getOrigin().y();
            float robot_yaw = tf::getYaw(robot_pos.getRotation());
            float waypoint_x_update = dest.position.x - robot_pos.getOrigin().x();
            float waypoint_y_update = dest.position.y - robot_pos.getOrigin().y();
            float r = std::sqrt(std::pow(dest.position.x - robot_pos.getOrigin().x(), 2) + std::pow(dest.position.y - robot_pos.getOrigin().y(), 2));
            int j = 0;
            int sum_num = 0;
            bool pass_bool = false;
            for(int i = 0; i < laserscan.ranges.size(); i++){
                if(!(laserscan.ranges[i] < laserscan.range_min || laserscan.ranges[i] > laserscan.range_max || std::isnan(laserscan.ranges[i]))){
                    float theta = laserscan.angle_min + i * laserscan.angle_increment + robot_yaw;
                    x[j] = laserscan.ranges[i] * cosf(theta);
                    y[j] = laserscan.ranges[i] * sinf(theta);
                    j++;
                }
            }
            float dist_waypoint[j];
            for(int i = 0;;i++){
                sum_x = 0;
                sum_y = 0;
                sum_num = 0;
                for(int ii = 0; ii < j; ii++){
                    dist_waypoint[ii] = std::sqrt(std::pow(x[ii] - waypoint_x_update, 2) + std::pow(y[ii] - waypoint_y_update, 2));
                    if(i == 0 && dist_waypoint[ii] < first_thresh){
                        sum_x += x[ii];
                        sum_y += y[ii];
                        sum_num++;
                    }
                }
                int size = sizeof(dist_waypoint) / sizeof(*dist_waypoint);
                float min = *std::min_element(dist_waypoint, dist_waypoint + size);
                if((i == 0 && min > first_thresh) || (i == 0 && std::sqrt(std::pow(waypoint_x_update, 2) + std::pow(waypoint_y_update, 2)) < first_thresh)){
                    pass_bool = true;
                    ROS_WARN("Min dist_waypoint: %f", min);
                    ROS_WARN("Pass");
                    break;
                }else if(i == 0 && min < first_thresh){
                    ROS_WARN("There is an obstacle near waypoint.");
                    ROS_WARN("Waypoint x: %f, y: %f", waypoint_x_update, waypoint_y_update);
                    ROS_WARN("Obstacle x: %f, y: %f", sum_x / sum_num, sum_y / sum_num);
                }else if(waypoint_x * waypoint_x_update < 0){
                    waypoint_x_update = waypoint_x;
                    waypoint_y_update = waypoint_y;
                    break;
                }else if(i > 0 && min > after_second_thresh){
                    break;
                }
                waypoint_x = waypoint_x_update;
                waypoint_y = waypoint_y_update;
                waypoint_x_update = (dest.position.x - robot_pos.getOrigin().x()) * (r - 0.5 * (i + 1)) / r;
                waypoint_y_update = (dest.position.y - robot_pos.getOrigin().y()) * (r - 0.5 * (i + 1)) / r;
            }
            if(!pass_bool){
                std_srvs::Empty empty;
                mid_waypoint.position.x = waypoint_x_update + robot_pos.getOrigin().x();
                mid_waypoint.position.y = waypoint_y_update + robot_pos.getOrigin().y();
                mid_waypoint.orientation = dest.orientation;
                move_base_goal.target_pose.header.stamp = ros::Time::now();
                move_base_goal.target_pose.header.frame_id = world_frame_;
                move_base_goal.target_pose.pose.orientation = mid_waypoint.orientation;
                move_base_goal.target_pose.pose.position = mid_waypoint.position;
                ROS_WARN("Send goal x: %f, y: %f", waypoint_x_update, waypoint_y_update);
                move_base_action_.sendGoal(move_base_goal);
                start_nav_time = ros::Time::now().toSec();
                while(!onNavigationPoint(mid_waypoint.position, goal_thresh)){
                    time = ros::Time::now().toSec();
                    if(time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                        ROS_WARN("Resend the navigation goal.");
                        clear_costmaps_srv_.call(empty);
                        move_base_goal.target_pose.header.stamp = ros::Time::now();
                        move_base_action_.sendGoal(move_base_goal);
                        start_nav_time = time;
                    }
                    sleep();
                }
                ROS_WARN("Reach the goal without obstacle.");
            }
        }
    }



    /*
    ++++++++++ Check if robot reached the goal sent to move_base ++++++++++
    */
    bool navigationFinished()
    {
        if (move_base_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Move base state is Goal reached");
            return true;
        }
        return false;
    }



    /*
    ++++++++++ Check if robot reached current waypoint ++++++++++
    */
    bool onNavigationPoint(const geometry_msgs::Point &dest, double dist_err=1.0)
    {
        tf::StampedTransform robot_gl = getRobotPosGL();
        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));
        if (waypoint_list_[wp_num_.data-1].stop) {
            tf::Quaternion q = robot_gl.getRotation();
            tf::Matrix3x3 m(q);
            double r, p ,y;
            m.getRPY(r, p, y);
            double diff_yaw = std::abs(y - target_yaw_);
            if ((dist < min_dist_err_) && (diff_yaw < min_yaw_err_)) return true;
            else if ((dist < min_dist_err_ * 2) && navigationFinished()) return true;
            return false;
        }
        return dist < dist_err;
    }



    /*
    ++++++++++ Get gloabl position of robot ++++++++++
    */
    tf::StampedTransform getRobotPosGL()
    {
        tf::StampedTransform robot_gl;
        try {
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
        } catch(tf::TransformException &e) {
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }
        return robot_gl;
    }



    /*
    ++++++++++ Sleep function used in main loop function ++++++++++
    */
    void sleep()
    {
        rate_.sleep();
        ros::spinOnce();
        publishPoseArray();
    }



    /*
    ++++++++++ Publish waypoints to be displayed as arrows on rviz ++++++++++
    */
    void publishPoseArray()
    {
        waypoints_.header.stamp = ros::Time::now();
        wp_pub_.publish(waypoints_);
    }



    /*
    ++++++++++ Send goal to move_base ++++++++++
    */
    void startNavigationGL(const geometry_msgs::Pose &dest)
    {
        setMaxVel();
        bool FindGoalWithoutObstacle = true;
        float first_thresh = 1.0;
        float after_second_thresh = 1.0;
        float goal_thresh = 0.5;
        if(FindGoalWithoutObstacle){
            sendNoObstacleGoal(first_thresh, after_second_thresh, goal_thresh, dest);
        }
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.header.frame_id = world_frame_;
        move_base_goal.target_pose.pose.position = dest.position;
        move_base_goal.target_pose.pose.orientation = dest.orientation;
        move_base_action_.sendGoal(move_base_goal);
        sleep();
        if (waypoint_list_[wp_num_.data-1].stop) {
            tf::Quaternion q(dest.orientation.x, dest.orientation.y, dest.orientation.z, dest.orientation.w);
            tf::Matrix3x3 m(q);
            double r, p ,y;
            m.getRPY(r, p, y);
            target_yaw_ = y;
        }
    }



    /*
    ++++++++++ Publish ratio to maximum speed ++++++++++
    */
    void setMaxVel()
    {
        float max_vel_rate = waypoint_list_[wp_num_.data-1].vel;
        max_vel_msg_.data = max_vel_rate;
        max_vel_pub_.publish(max_vel_msg_);
    }



    /*
    ++++++++++ Main loop function ++++++++++
    */
    void run()
    {
        ROS_INFO("Waiting for waypoint navigation to start.");
        int resend_goal;
        double start_nav_time, time;
        std_srvs::Empty empty;
        while(ros::ok()) {
            // has_activate_ is false, nothing to do
            if (!has_activate_) {
                sleep();
                continue;
            }
            // go to fianal goal and finish process
            if (current_waypoint_ == finish_pose_) {
                ROS_INFO_STREAM("Go to final goal");
                startNavigationGL(*current_waypoint_);
                start_nav_time = ros::Time::now().toSec();
                while(!onNavigationPoint(finish_pose_->position) && ros::ok()) {
                    time = ros::Time::now().toSec();
                    if (time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                        ROS_WARN("Resend the navigation goal.");
                        clear_costmaps_srv_.call(empty);
                        startNavigationGL(*current_waypoint_);
                        start_nav_time = time;
                    }
                    sleep();
                }
                ROS_INFO("Final goal reached!!");
                has_activate_ = false;
                continue;
            }
            // go to current waypoint
            ROS_INFO_STREAM("Go to waypoint " << wp_num_.data);
            ROS_INFO_STREAM("x: " << waypoint_list_[wp_num_.data-1].x << ", y: " << waypoint_list_[wp_num_.data-1].y);
            startNavigationGL(*current_waypoint_);
            resend_goal = 0;
            start_nav_time = ros::Time::now().toSec();
            dist_err_ = waypoint_list_[wp_num_.data-1].rad;
            try {
                // loop until reach waypoint
                while(!onNavigationPoint(current_waypoint_->position, dist_err_)) {
                    if (!has_activate_) throw SwitchRunningStatus();
                    time = ros::Time::now().toSec();
                    if (time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                        ROS_WARN("Resend the navigation goal.");
                        clear_costmaps_srv_.call(empty);
                        startNavigationGL(*current_waypoint_);
                        resend_goal++;
                        if (!waypoint_list_[wp_num_.data-1].stop && resend_goal == 3) {
                            ROS_WARN("Skip waypoint.");
                            break;
                        }
                        start_nav_time = time;
                    }
                    wp_num_pub_.publish(wp_num_);
                    sleep();
                }
                // if current waypoint is stop point
                if (waypoint_list_[wp_num_.data-1].stop) {
                    has_activate_ = false;
                    move_base_action_.cancelAllGoals();
                    ROS_INFO("Waiting for navigation to resume...");
                }
                // update current waypoint
                current_waypoint_++;
                wp_num_.data++;
                wp_num_pub_.publish(wp_num_);

            } catch(const SwitchRunningStatus &e) {
                move_base_action_.cancelAllGoals();
                ROS_INFO_STREAM("Running status switched");
            }
            sleep();
        }
    }




private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    geometry_msgs::PoseArray waypoints_;
    visualization_msgs::MarkerArray marker_;
    std::vector<Waypoint> waypoint_list_;
    
    //Scripts of "start from the middle"
    //Edited mori 2022/10/19
    //#############################################################
    bool StartFromTheMiddle;
    std::vector<geometry_msgs::Pose>::iterator init_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator compare_waypoint_;
    //#############################################################
    
    std::vector<geometry_msgs::Pose>::iterator current_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator finish_pose_;
    bool has_activate_, stopped_;
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::ServiceServer start_server_, stop_server_, resume_server_;
    ros::Subscriber cmd_vel_sub_,scan_sub_, move_base_status_sub_;
    ros::Publisher wp_pub_, max_vel_pub_, wp_num_pub_;
    ros::ServiceClient clear_costmaps_srv_;
    double last_moved_time_, dist_err_, min_dist_err_, target_yaw_, min_yaw_err_;
    std_msgs::UInt16 wp_num_;
    std_msgs::Float32 max_vel_msg_;
    sensor_msgs::LaserScan laserscan;
};






int main(int argc, char *argv[]) {
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;
    w_nav.run();

    return 0;
}
