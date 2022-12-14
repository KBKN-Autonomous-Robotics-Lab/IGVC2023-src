#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>


class VelocityController {
public:
    VelocityController() :
        move_base_action_("move_base", true),
        standard_vel_(1.0),
        current_max_vel_(1.0),
        min_vel_(0.0)
    {
        while((move_base_action_.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true)) {
            ROS_INFO("Waiting...");
        }

        ros::NodeHandle private_nh("~");
        std::string max_vel_param = "/move_base/TrajectoryPlannerROS/max_vel_x";
        std::string min_vel_param = "/move_base/TrajectoryPlannerROS/min_vel_x";
        private_nh.param("max_vel_param", max_vel_param, max_vel_param);
        private_nh.param("min_vel_param", min_vel_param, min_vel_param);

        ros::NodeHandle nh;
        max_vel_sub_ = nh.subscribe("/max_vel", 1, &VelocityController::maxVelCallback, this);
        cmd_vel_sub_ = nh.subscribe("/cmd_vel_in", 100, &VelocityController::cmdVelCallback, this);
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_out", 100);

        nh.param(max_vel_param, standard_vel_, standard_vel_);
        nh.param(min_vel_param, min_vel_, min_vel_);
        ROS_INFO_STREAM("Standard max vel: " << standard_vel_);
    }



    void maxVelCallback(const std_msgs::Float32 &msg)
    {
        current_max_vel_ = standard_vel_ * msg.data;
        current_max_vel_ = std::max(current_max_vel_, min_vel_);
        ROS_INFO_STREAM("Set max vel: " << current_max_vel_);
    }



    void cmdVelCallback(const geometry_msgs::Twist &msg)
    {
        pub_msg_.linear.x = std::min(float(msg.linear.x), current_max_vel_);
        pub_msg_.angular = msg.angular;
        cmd_vel_pub_.publish(pub_msg_);
    }



private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_;
    ros::Subscriber max_vel_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher cmd_vel_pub_;
    float standard_vel_, current_max_vel_, min_vel_;
    geometry_msgs::Twist pub_msg_;
};







int main(int argc, char *argv[]) {
    ros::init(argc, argv, "velocity_controller");
    VelocityController vc;

    ros::spin();
    return 0;
}
