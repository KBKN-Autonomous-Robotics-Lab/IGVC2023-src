#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <fulanghua_srvs/Pose.h>
 
#include <yaml-cpp/yaml.h>
 
#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <math.h>
#include <limits>
 
//KBKN Original
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <angles/angles.h>
 
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
    WaypointsNavigation() :
        has_activate_(false),
        move_base_action_("move_base", true),
        rate_(10),
        last_moved_time_(0),
        dist_err_(0.8)
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
        if(filename != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename);
            if(!readFile(filename)) {
                ROS_ERROR("Failed loading waypoints file");
            } else {
                last_waypoint_ = waypoints_.poses.end()-2;
                finish_pose_ = waypoints_.poses.end()-1;
                computeWpOrientation();
            }
            current_waypoint_ = waypoints_.poses.begin();
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }
 
        private_nh.param("dist_err", dist_err_, dist_err_);
        
        private_nh.param("tandem_area_start1", tandem_area_start1, 1000);
        private_nh.param("tandem_area_start2", tandem_area_start2, 1000);
        private_nh.param("tandem_area_start3", tandem_area_start3, 1000);
        private_nh.param("tandem_area_start4", tandem_area_start4, 1000);
        private_nh.param("tandem_area_start5", tandem_area_start5, 1000);
        private_nh.param("tandem_area_start6", tandem_area_start6, 1000);
 
        private_nh.param("tandem_area_end1", tandem_area_end1, 1000);
        private_nh.param("tandem_area_end2", tandem_area_end2, 1000);
        private_nh.param("tandem_area_end3", tandem_area_end3, 1000);
        private_nh.param("tandem_area_end4", tandem_area_end4, 1000);
        private_nh.param("tandem_area_end5", tandem_area_end5, 1000);
        private_nh.param("tandem_area_end6", tandem_area_end6, 1000);
 
        private_nh.param("stop_waypoint1", stop_waypoint1, 1000);
        private_nh.param("stop_waypoint2", stop_waypoint2, 1000);
        private_nh.param("stop_waypoint3", stop_waypoint3, 1000);
        private_nh.param("stop_waypoint4", stop_waypoint4, 1000);
        private_nh.param("stop_waypoint5", stop_waypoint5, 1000);
        private_nh.param("stop_waypoint6", stop_waypoint6, 1000);
        private_nh.param("stop_waypoint7", stop_waypoint7, 1000);
        private_nh.param("stop_waypoint8", stop_waypoint8, 1000);
        private_nh.param("stop_waypoint9", stop_waypoint9, 1000);
        private_nh.param("stop_waypoint10", stop_waypoint10, 1000);
        private_nh.param("stop_waypoint11", stop_waypoint11, 1000);
        private_nh.param("stop_waypoint12", stop_waypoint12, 1000);
 
        private_nh.param("fast_area_start1", fast_area_start1, 1000);
        private_nh.param("fast_area_start2", fast_area_start2, 1000);
        private_nh.param("fast_area_start3", fast_area_start3, 1000);
        private_nh.param("fast_area_start4", fast_area_start4, 1000);
 
        private_nh.param("fast_area_end1", fast_area_end1, 1000);
        private_nh.param("fast_area_end2", fast_area_end2, 1000);
        private_nh.param("fast_area_end3", fast_area_end3, 1000);
        private_nh.param("fast_area_end4", fast_area_end4, 1000);
 
        private_nh.param("slow_area_start1", slow_area_start1, 1000);
        private_nh.param("slow_area_start2", slow_area_start2, 1000);
        private_nh.param("slow_area_start3", slow_area_start3, 1000);
   
        private_nh.param("slow_area_end1", slow_area_end1, 1000);
        private_nh.param("slow_area_end2", slow_area_end2, 1000);
        private_nh.param("slow_area_end3", slow_area_end3, 1000);
        
        ros::NodeHandle nh;
        start_server_ = nh.advertiseService("start_wp_nav", &WaypointsNavigation::startNavigationCallback, this);
        pause_server_ = nh.advertiseService("pause_wp_nav", &WaypointsNavigation::pauseNavigationCallback,this);
        unpause_server_ = nh.advertiseService("unpause_wp_nav", &WaypointsNavigation::unpauseNavigationCallback,this);
        stop_server_ = nh.advertiseService("stop_wp_nav", &WaypointsNavigation::pauseNavigationCallback,this);
        suspend_server_ = nh.advertiseService("suspend_wp_pose", &WaypointsNavigation::suspendPoseCallback, this);
        resume_server_ = nh.advertiseService("resume_wp_pose", &WaypointsNavigation::resumePoseCallback, this);
        search_server_ = nh.advertiseService("near_wp_nav",&WaypointsNavigation::searchPoseCallback, this);
        cmd_vel_sub_ = nh.subscribe("/igvc_robot/cmd_vel", 1, &WaypointsNavigation::cmdVelCallback, this);
        wp_pub_ = nh.advertise<geometry_msgs::PoseArray>("waypoints", 10);
        clear_costmaps_srv_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
        
        //KBKN Original
        waypoint_num_pub = nh.advertise<std_msgs::UInt16>("waypoint_num",1);
        zero_vel_flag_pub = nh.advertise<std_msgs::Bool>("zero_vel_flag",1);
        back_vel_flag_pub = nh.advertise<std_msgs::Bool>("back_vel_flag",1);
        fast_vel_flag_pub = nh.advertise<std_msgs::Bool>("fast_vel_flag",1);
        slow_vel_flag_pub = nh.advertise<std_msgs::Bool>("slow_vel_flag",1);
        pose_sub = nh.subscribe("global_human_position", 1, &WaypointsNavigation::poseCallback, this);
        human_detected_flag_sub = nh.subscribe("human_detected_flag", 1, &WaypointsNavigation::humanDetectedflagCallback, this);
        
        /* ++++++++++++++++++++++++++++++++++++++*/
        // This section is about IGVC2022 LEDs
        navigation_start_pub = nh.advertise<std_msgs::Bool>("/navBool", 10); //Publisher for navBool
        navigation_end_pub = nh.advertise<std_msgs::Bool>("/navBool", 10);
        /* ++++++++++++++++++++++++++++++++++++++*/
        
    }
 
    void humanDetectedflagCallback(const std_msgs::Bool &msg){
        human_detected_flag = msg.data;
    }
    
    void poseCallback(const geometry_msgs::Pose &msg) {
        human_pos = msg;
    }
 
    bool startNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
        if(has_activate_) {
            response.success = false;
            return false;
        }
        
        std_srvs::Empty empty;
        while(!clear_costmaps_srv_.call(empty)) {
            ROS_WARN("Resend clear costmap service");
            sleep();
        }
 
        current_waypoint_ = waypoints_.poses.begin();
        has_activate_ = true;
        response.success = true;
        return true;
    }
 
    bool pauseNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
         if(!has_activate_) {
            ROS_WARN("Navigation is already pause");
            response.success = false;
            return false;
        }
        
        has_activate_ = false;
        response.success = true;
        return true;
    }
 
    bool unpauseNavigationCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
        if(has_activate_){
            ROS_WARN("Navigation is already active");
            response.success = false;
        }
 
        has_activate_ = true;
        response.success = true;
        return true;
    }
 
    void stopNavigationCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response){
        has_activate_ = false;
        move_base_action_.cancelAllGoals();
    }
 
    bool resumePoseCallback(fulanghua_srvs::Pose::Request &request, fulanghua_srvs::Pose::Response &response) {
        if(has_activate_) {
            response.status = false;
            return false;
        }
        
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);
        //move_base_action_.cancelAllGoals();
        
        ///< @todo calculating metric with request orientation
        double min_dist = std::numeric_limits<double>::max();
        for(std::vector<geometry_msgs::Pose>::iterator it = current_waypoint_; it != finish_pose_; it++) {
            double dist = hypot(it->position.x - request.pose.position.x, it->position.y - request.pose.position.y);
            if(dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
            }
        }
        
        response.status = true;
        has_activate_ = true;
 
        return true;
    }
 
    bool suspendPoseCallback(fulanghua_srvs::Pose::Request &request, fulanghua_srvs::Pose::Response &response) {
        if(!has_activate_) {
            response.status = false;
            return false;
        }
        
        //move_base_action_.cancelAllGoals();
        startNavigationGL(request.pose);
        bool isNavigationFinished = false;
        while(!isNavigationFinished && ros::ok()) {
            actionlib::SimpleClientGoalState state = move_base_action_.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED){
                isNavigationFinished = true;
                response.status = true;
            }else if(state == actionlib::SimpleClientGoalState::ABORTED){
                isNavigationFinished = true;
                response.status = false;
            }
            sleep();
        }
        has_activate_ = false;
 
        return true;
    }
 
    bool searchPoseCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response){
        
        if(has_activate_) {
            response.success = false;
            return false;
        }
        
        tf::StampedTransform robot_gl = getRobotPosGL();
        std_srvs::Empty empty;
        clear_costmaps_srv_.call(empty);
 
        double min_dist = std::numeric_limits<double>::max();
        for(std::vector<geometry_msgs::Pose>::iterator it = current_waypoint_; it != finish_pose_; it++) {
            double dist = hypot(it->position.x - robot_gl.getOrigin().x(), it->position.y - robot_gl.getOrigin().y());
            if(dist < min_dist) {
                min_dist = dist;
                current_waypoint_ = it;
            }
        }
        
        response.success = true;
        has_activate_ = true;
 
        return true;
    }
    
    void cmdVelCallback(const geometry_msgs::Twist &msg){
        if(msg.linear.x > -0.001 && msg.linear.x < 0.001   &&
           msg.linear.y > -0.001 && msg.linear.y < 0.001   &&
           msg.linear.z > -0.001 && msg.linear.z < 0.001   &&
           msg.angular.x > -0.001 && msg.angular.x < 0.001 &&
           msg.angular.y > -0.001 && msg.angular.y < 0.001 &&
           msg.angular.z > -0.001 && msg.angular.z < 0.001){
            
           //ROS_INFO("command velocity all zero");
        }else{
           last_moved_time_ = ros::Time::now().toSec();
        }
        cmd_vel = msg;
    }
 
    bool readFile(const std::string &filename){
        waypoints_.poses.clear();
        try{
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if(ifs.good() == false){
                return false;
            }
 
            YAML::Node node;
            
            #ifdef NEW_YAMLCPP
                node = YAML::Load(ifs);
            #else
                YAML::Parser parser(ifs);
                parser.GetNextDocument(node);
            #endif
 
            #ifdef NEW_YAMLCPP
                const YAML::Node &wp_node_tmp = node["waypoints"];
                const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
            #else
                const YAML::Node *wp_node = node.FindValue("waypoints");
            #endif
 
            geometry_msgs::Pose pose;
            if(wp_node != NULL){
                for(int i=0; i < wp_node->size(); i++){
 
                    (*wp_node)[i]["point"]["x"] >> pose.position.x;
                    (*wp_node)[i]["point"]["y"] >> pose.position.y;
                    (*wp_node)[i]["point"]["z"] >> pose.position.z;
 
                    waypoints_.poses.push_back(pose);
 
                }
            }else{
                return false;
            }
            
            #ifdef NEW_YAMLCPP
                const YAML::Node &fp_node_tmp = node["finish_pose"];
                const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
            #else
                const YAML::Node *fp_node = node.FindValue("finish_pose");
            #endif
 
            if(fp_node != NULL){
                (*fp_node)["pose"]["position"]["x"] >> pose.position.x;
                (*fp_node)["pose"]["position"]["y"] >> pose.position.y;
                (*fp_node)["pose"]["position"]["z"] >> pose.position.z;
 
                (*fp_node)["pose"]["orientation"]["x"] >> pose.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> pose.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> pose.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> pose.orientation.w;
 
                waypoints_.poses.push_back(pose);
 
            }else{
                return false;
            }
 
        }catch(YAML::ParserException &e){
            return false;
 
        }catch(YAML::RepresentationException &e){
            return false;
        }
 
        return true;
    }
 
   void computeWpOrientation(){
        for(std::vector<geometry_msgs::Pose>::iterator it = waypoints_.poses.begin(); it != finish_pose_; it++) {
            double goal_direction = atan2((it+1)->position.y - (it)->position.y,
                                          (it+1)->position.x - (it)->position.x);
            (it)->orientation = tf::createQuaternionMsgFromYaw(goal_direction);
        }
        waypoints_.header.frame_id = world_frame_;
    }
 
    bool shouldSendGoal(){
        bool ret = true;
        actionlib::SimpleClientGoalState state = move_base_action_.getState();
        if((state != actionlib::SimpleClientGoalState::ACTIVE) &&
           (state != actionlib::SimpleClientGoalState::PENDING) && 
           (state != actionlib::SimpleClientGoalState::RECALLED) &&
           (state != actionlib::SimpleClientGoalState::PREEMPTED))
        {
            ret = false;
        }
 
        if(waypoints_.poses.empty()){
            ret = false;
        }
 
        return ret;
    }
 
    bool navigationFinished(){
        return move_base_action_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }
 
    bool onNavigationPoint(const geometry_msgs::Point &dest, double dist_err = 0.8){
        tf::StampedTransform robot_gl = getRobotPosGL();
 
        const double wx = dest.x;
        const double wy = dest.y;
        const double rx = robot_gl.getOrigin().x();
        const double ry = robot_gl.getOrigin().y();
        const double dist = std::sqrt(std::pow(wx - rx, 2) + std::pow(wy - ry, 2));
 
        return dist < dist_err;
    }
 
    tf::StampedTransform getRobotPosGL(){
        tf::StampedTransform robot_gl;
        try{
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
        }catch(tf::TransformException &e){
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }
 
        return robot_gl;
    }
 
    void sleep(){
        rate_.sleep();
        ros::spinOnce();
        publishPoseArray();
    }
 
    void startNavigationGL(const geometry_msgs::Point &dest){
        geometry_msgs::Pose pose;
        pose.position = dest;
        pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        startNavigationGL(pose);
    }
 
    void startNavigationGL(const geometry_msgs::Pose &dest){
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.header.frame_id = world_frame_;
        move_base_goal.target_pose.pose.position = dest.position;
        move_base_goal.target_pose.pose.orientation = dest.orientation;
        
        move_base_action_.sendGoal(move_base_goal);
    }
 
    
    void publishPoseArray(){
        waypoints_.header.stamp = ros::Time::now();
        wp_pub_.publish(waypoints_);
    }
 
    //KBKN Original
    bool stop_area(const int waypoint_num){
        if((waypoint_num == stop_waypoint1)||
           (waypoint_num == stop_waypoint2)||
           (waypoint_num == stop_waypoint3)||
           (waypoint_num == stop_waypoint4)||
           (waypoint_num == stop_waypoint5)||
           (waypoint_num == stop_waypoint6)||
           (waypoint_num == stop_waypoint7)||
           (waypoint_num == stop_waypoint8)||
           (waypoint_num == stop_waypoint9)||
           (waypoint_num == stop_waypoint10)||
           (waypoint_num == stop_waypoint11)||
           (waypoint_num == stop_waypoint12))
 
           // (waypoint_num == traffic_waypoint1)||
           // (waypoint_num == traffic_waypoint2))
 
           return true;
        else return false;
    }
 
    bool tandem_area(const std_msgs::UInt16 waypoint_num){
        if(((tandem_area_start1 <= waypoint_num.data) && (waypoint_num.data <= tandem_area_end1))||
           ((tandem_area_start2 <= waypoint_num.data) && (waypoint_num.data <= tandem_area_end2))||
           ((tandem_area_start3 <= waypoint_num.data) && (waypoint_num.data <= tandem_area_end3))||
           ((tandem_area_start4 <= waypoint_num.data) && (waypoint_num.data <= tandem_area_end4))||
           ((tandem_area_start5 <= waypoint_num.data) && (waypoint_num.data <= tandem_area_end5))||
           ((tandem_area_start6 <= waypoint_num.data) && (waypoint_num.data <= tandem_area_end6))) return true;
        else return false;
    }
 
    bool fast_area(const std_msgs::UInt16 waypoint_num){
        if(((fast_area_start1 < waypoint_num.data) && (waypoint_num.data <= fast_area_end1))||
           ((fast_area_start2 < waypoint_num.data) && (waypoint_num.data <= fast_area_end2))||
           ((fast_area_start3 < waypoint_num.data) && (waypoint_num.data <= fast_area_end3))||
           ((fast_area_start4 < waypoint_num.data) && (waypoint_num.data <= fast_area_end4))) return true;
        else return false;
    }
 
    bool slow_area(const std_msgs::UInt16 waypoint_num){
        if(((slow_area_start1 < waypoint_num.data) && (waypoint_num.data <= slow_area_end1))||
           ((slow_area_start2 < waypoint_num.data) && (waypoint_num.data <= slow_area_end2))||
           ((slow_area_start3 < waypoint_num.data) && (waypoint_num.data <= slow_area_end3))) return true;
        else return false;
    }
 
    void normalize(const geometry_msgs::Pose &msg){
        double normalize_quaternion = std::sqrt(std::pow(msg.orientation.x,2)+std::pow(msg.orientation.y,2)+std::pow(msg.orientation.z,2)+std::pow(msg.orientation.w,2));
        const_cast<geometry_msgs::Pose&>(msg).orientation.x=msg.orientation.x/normalize_quaternion;
        const_cast<geometry_msgs::Pose&>(msg).orientation.y=msg.orientation.y/normalize_quaternion;
        const_cast<geometry_msgs::Pose&>(msg).orientation.z=msg.orientation.z/normalize_quaternion;
        const_cast<geometry_msgs::Pose&>(msg).orientation.w=msg.orientation.w/normalize_quaternion;
    }
 
    bool human_approach_success(const geometry_msgs::Pose &dest){
        resend_goal = 0;
        start_nav_time = ros::Time::now().toSec();
        while(!navigationFinished()) {
            if(!has_activate_)
                throw SwitchRunningStatus();
            time = ros::Time::now().toSec();
            if(time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                std_srvs::Empty empty;
                clear_costmaps_srv_.call(empty);
                ++resend_goal;
                std::cout<<"approach times:"<<resend_goal<<std::endl;
                if(resend_goal >= 3){
                    startNavigationGL(dest);
                    return false;
                }
                start_nav_time = time;
            }
            sleep();
        }
        return true;
    }
 
    void run(){
        //KBKN Original
        std_msgs::UInt16 waypoint_num;
        std_msgs::Bool detection_stop_flag, back_vel_flag, zero_vel_flag;
        
        /* ++++++++++++++++++++++++++++++++++++++*/
		  // This section is about IGVC2022 LEDs
        std_msgs::Bool navBool;
        /* ++++++++++++++++++++++++++++++++++++++*/
        
        waypoint_num.data = 0;
        detection_stop_flag.data = false;
        back_vel_flag.data = false;
        zero_vel_flag.data = false;
        human_detected_flag = false;
        
        while(ros::ok()){
            try {
                if(has_activate_) {
                
                    /* ++++++++++++++++++++++++++++++++++++++*/
                    // This section is about IGVC2022 LEDs
                    navBool.data = true; //Publish "true" when navigation is started
    	              navigation_start_pub.publish(navBool);
                    ROS_INFO("navBool:true published!");
                    /* ++++++++++++++++++++++++++++++++++++++*/
                    
                    waypoint_num_pub.publish(waypoint_num);
                    ROS_INFO_STREAM(waypoint_num.data);
                    if(current_waypoint_ == last_waypoint_) {
                        ROS_INFO("prepare finish pose");
                    } else {
                        ROS_INFO("calculate waypoint direction");
                        //ROS_INFO_STREAM("goal_direction = " << current_waypoint_->orientation);
                        //ROS_INFO_STREAM("current_waypoint_+1 " << (current_waypoint_+1)->position.y);
                        //ROS_INFO_STREAM("current_waypoint_" << current_waypoint_->position.y);
                    }
                    normalize(*current_waypoint_);
                    startNavigationGL(*current_waypoint_);
                    int resend_goal = 0;
                    double start_nav_time = ros::Time::now().toSec();
                    double detect_action_start_time, detect_action_finish_time;
                    if(tandem_area(waypoint_num)){
                        std_srvs::Empty empty;
                        clear_costmaps_srv_.call(empty);
                    }
                    
                    std_msgs::Bool fast_vel_flag;
                    if(fast_area(waypoint_num)) {fast_vel_flag.data = true;}
                    else {fast_vel_flag.data = false;}
                    fast_vel_flag_pub.publish(fast_vel_flag);
 
                    std_msgs::Bool slow_vel_flag;
                    if(slow_area(waypoint_num)) {slow_vel_flag.data = true;}
                    else {slow_vel_flag.data = false;}
                    slow_vel_flag_pub.publish(slow_vel_flag);
 
                    if(stop_area(waypoint_num.data)){
                        //while(!onNavigationPoint(current_waypoint_->position, dist_err_)) {
                        while(!navigationFinished()) {
                            if(!has_activate_)
                                throw SwitchRunningStatus();
                        
                            double time = ros::Time::now().toSec();
                            if(time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                                ROS_WARN("Resend the navigation goal.");
                                std_srvs::Empty empty;
                                clear_costmaps_srv_.call(empty);
                                startNavigationGL(*current_waypoint_);
                                //resend_goal++;
                                //if(resend_goal == 3) {
                                //    ROS_WARN("Skip waypoint.");
                                //    current_waypoint_++;
                                //    startNavigationGL(*current_waypoint_);
                                //}
                                start_nav_time = time;
                            }
                            sleep();
                        }
                    } 
                    else {
                        while(!onNavigationPoint(current_waypoint_->position, dist_err_)) {
                            // while(!doOnce) ros::spinOnce();
                            if(human_detected_flag) {//detected human
                                // while(!doOnce2) ros::spinOnce();
                                detection_stop_flag.data = true;
                                detection_stop_flag_pub.publish(detection_stop_flag);
                                std::cout<<"--------------------approach to human----------------"<<std::endl;
                                normalize(human_pos);
                                startNavigationGL(human_pos);
                                if(human_approach_success(*current_waypoint_)){
                                    zero_vel_flag.data = true;
                                    zero_vel_flag_pub.publish(zero_vel_flag);
                                    startNavigationGL(*current_waypoint_);
                                    detect_action_start_time = ros::Time::now().toSec();
                                    detect_action_finish_time = ros::Time::now().toSec();
                                    while(detect_action_finish_time-detect_action_start_time<5){
                                        detect_action_finish_time = ros::Time::now().toSec();
                                    }
                                    zero_vel_flag.data = false;
                                    zero_vel_flag_pub.publish(zero_vel_flag);
                                    back_vel_flag.data = true;
                                    back_vel_flag_pub.publish(back_vel_flag);
                                    while(cmd_vel.linear.x!=-0.2) ros::spinOnce();
                                    std::cout<<"back start\n";
                                    detect_action_start_time = ros::Time::now().toSec();
                                    detect_action_finish_time = ros::Time::now().toSec();
                                    while(detect_action_finish_time-detect_action_start_time<2){
                                        detect_action_finish_time = ros::Time::now().toSec();
                                    }
                                    back_vel_flag.data = false;
                                    back_vel_flag_pub.publish(back_vel_flag);
                                    while(cmd_vel.linear.x==-0.2) ros::spinOnce();
                                    std::cout<<"back end\n";
                                    // doOnce2 = false;
                                }
                                detection_stop_flag.data = false;
                                detection_stop_flag_pub.publish(detection_stop_flag);
                                resend_goal = 0;
                                start_nav_time = ros::Time::now().toSec();
                            }
                            if(!has_activate_)
                                throw SwitchRunningStatus();
                            time = ros::Time::now().toSec();
                            if(time - start_nav_time > 10.0 && time - last_moved_time_ > 10.0) {
                                ROS_WARN("Resend the navigation goal.");
                                std_srvs::Empty empty;
                                clear_costmaps_srv_.call(empty);
                                startNavigationGL(*current_waypoint_);
                                ++resend_goal;
                                if((resend_goal >= 3)&&(!stop_area(waypoint_num.data-1))&&(!tandem_area(waypoint_num))) break;
                                start_nav_time = time;
                            }
                            sleep();
                        }
                        // doOnce = false;
                    }
 
                    if(stop_area(waypoint_num.data)){
                        std_msgs::Bool zero_vel_flag;
                        zero_vel_flag.data = true;
                        zero_vel_flag_pub.publish(zero_vel_flag);
                    }
 
                    current_waypoint_++;
                    ++waypoint_num.data;
                    if(current_waypoint_ == finish_pose_) {
                        startNavigationGL(*current_waypoint_);
                        while(!navigationFinished() && ros::ok()) sleep();
                        has_activate_ = false;
                        waypoint_num.data = 0;
                        
                        /* ++++++++++++++++++++++++++++++++++++++*/
                        // This section is about IGVC2022 LEDs
                        navBool.data = false; //Publish "false" when navigation is ended
    	                  navigation_end_pub.publish(navBool);
                        ROS_INFO("navBool:false published!");
                        /* ++++++++++++++++++++++++++++++++++++++*/
                        
                        ROS_INFO_STREAM("FINISH WAYPOINT NAVIGATION !!");
                    }
                }
            } catch(const SwitchRunningStatus &e) {
                ROS_INFO_STREAM("running status switched");
            }
 
            sleep();
        }
    }
 
private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    geometry_msgs::PoseArray waypoints_;
    geometry_msgs::Twist cmd_vel;
    visualization_msgs::MarkerArray marker_;
    std::vector<geometry_msgs::Pose>::iterator current_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator last_waypoint_;
    std::vector<geometry_msgs::Pose>::iterator finish_pose_;
    bool has_activate_;
    std::string robot_frame_, world_frame_;
    tf::TransformListener tf_listener_;
    ros::Rate rate_;
    ros::ServiceServer start_server_, pause_server_, unpause_server_, stop_server_, suspend_server_, resume_server_ ,search_server_;
    ros::Subscriber cmd_vel_sub_, pose_sub, human_detected_flag_sub;
    ros::Publisher wp_pub_, waypoint_num_pub, zero_vel_flag_pub, back_vel_flag_pub, fast_vel_flag_pub, slow_vel_flag_pub, detection_stop_flag_pub;
    
    /* ++++++++++++++++++++++++++++++++++++++*/
    // This section is about IGVC2022 LEDs
    ros::Publisher navigation_start_pub; //Publisher for navBool
    ros::Publisher navigation_end_pub;
    /* ++++++++++++++++++++++++++++++++++++++*/
    
    ros::ServiceClient clear_costmaps_srv_;
    
    int stop_waypoint1, stop_waypoint2, stop_waypoint3, stop_waypoint4, stop_waypoint5, stop_waypoint6, stop_waypoint7, stop_waypoint8, stop_waypoint9, stop_waypoint10, stop_waypoint11, stop_waypoint12; //traffic_waypoint1, traffic_waypoint2;
    int tandem_area_start1, tandem_area_start2, tandem_area_start3, tandem_area_end1, tandem_area_end2, tandem_area_end3, tandem_area_start4, tandem_area_start5, tandem_area_start6, tandem_area_end4, tandem_area_end5, tandem_area_end6;
    int fast_area_start1, fast_area_start2, fast_area_start3, fast_area_start4, fast_area_end1, fast_area_end2, fast_area_end3, fast_area_end4;
    int slow_area_start1, slow_area_start2, slow_area_start3, slow_area_end1, slow_area_end2, slow_area_end3;
    int resend_goal;
    double time, start_nav_time, last_moved_time_, dist_err_;
    geometry_msgs::Pose human_pos;
    bool human_detected_flag;
 
};
 
int main(int argc, char *argv[]){
    ros::init(argc, argv, ROS_PACKAGE_NAME);
    WaypointsNavigation w_nav;
    w_nav.run();
    return 0;
}
