#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <fstream>
#include <string>

#include <waypoint_saver.h>  // include Waypoint class



class WaypointsSaver{
public:
    WaypointsSaver() : 
        filename_("waypoints.yaml"), 
        default_rad_(0.8)
    {
        waypoints_viz_sub_ = nh_.subscribe("waypoints_viz", 1, &WaypointsSaver::waypointsVizCallback, this);
        waypoints_joy_sub_ = nh_.subscribe("waypoints_joy", 1, &WaypointsSaver::waypointsJoyCallback, this);
        finish_pose_sub_ = nh_.subscribe("finish_pose", 1, &WaypointsSaver::finishPoseCallback, this);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("waypoints", 10);

        ros::NodeHandle private_nh("~");
        private_nh.param("filename", filename_, filename_);
        private_nh.param("save_joy_button", save_joy_button_, 0);
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("world_frame", world_frame_, std::string("map"));
        private_nh.param("default_rad", default_rad_, default_rad_);
    }
    
    

    /*
    ++++++++++ Save waypoint by joy stick button ++++++++++
    */
    void waypointsJoyCallback(const sensor_msgs::Joy &msg)
    {
        static ros::Time saved_time(0.0);
        if (msg.buttons[save_joy_button_] == 1 && (ros::Time::now() - saved_time).toSec() > 3.0) {
            pushbackWaypoint();
            saved_time = ros::Time::now();
        }
        publishMarkerArray();
    }
    
    

    /*
    ++++++++++ Save waypoint by "Publish Point" on rviz ++++++++++
    */
    void waypointsVizCallback(const geometry_msgs::PointStamped &msg)
    {
        Waypoint point;
        point.x = msg.point.x;
        point.y = msg.point.y;
        point.z = msg.point.z;
        point.rad = default_rad_;
        waypoints_.push_back(point);
        addWaypointMarker(point);
        publishMarkerArray();
    }
    
    

    /*
    ++++++++++ Save finish pose by "2D Nav Goal" on rviz ++++++++++
    */
    void finishPoseCallback(const geometry_msgs::PoseStamped &msg)
    {
        finish_pose_ = msg;
        save();
        waypoints_.clear();
    }



    /*
    ++++++++++ Get robot's pose and add waypoint ++++++++++
    */
    void pushbackWaypoint()
    {
        tf::StampedTransform robot_gl;
        try {
            tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
            Waypoint point;
            point.x = robot_gl.getOrigin().x();
            point.y = robot_gl.getOrigin().y();
            point.z = robot_gl.getOrigin().z();
            point.rad = default_rad_;
            waypoints_.push_back(point);
            addWaypointMarker(point);
        }
        catch(tf::TransformException &e) {
            ROS_WARN_STREAM("tf::TransformException: " << e.what());
        }
    }
    
    
    
    /*
    ++++++++++ Add marker to display on rviz ++++++++++
    */
    void addWaypointMarker(Waypoint point)
    {
        double scale = 0.2;
        int number = waypoints_.size();
        std::stringstream name;
        name << "Waypoint " << number;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = world_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = name.str();
        marker.id = number;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = scale/2;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r=0.8f;
        marker.color.g=0.2f;
        marker.color.b=0.2f;
        marker.color.a = 1.0f;
        
        markers_.push_back(marker);
    }



    /*
    ++++++++++ Publish markerArray to display waypoints on rviz ++++++++++
    */
    void publishMarkerArray()
    {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers = markers_;
        markers_pub_.publish(marker_array);
    }
    


    /*
    ++++++++++ Save waypoints as yaml file ++++++++++
    */
    void save()
    {
        std::ofstream ofs(filename_.c_str(), std::ios::out);
        /*
        waypoints:
        - point: {x: *, y: *, z: *, ...}
        */
        ofs << "waypoints:" << std::endl;
        for(int i=0; i < waypoints_.size(); i++) {
            ofs << "- point: " << waypoints_[i].getYAMLstr() << std::endl;
        }
        /*
        finish_pose:
          header: {seq: *, stamp: *, frame_id: *}
          pose:
            positioin: {x: *, y: *, z: *}
            orientation: {x: *, y: *, z: *, w: *}
        */
        ofs << "finish_pose:" << std::endl;
        ofs << "  header: {" ;
        ofs << "seq: "      << finish_pose_.header.seq        << ", " ;
        ofs << "stamp: "    << finish_pose_.header.stamp      << ", " ;
        ofs << "frame_id: " << finish_pose_.header.frame_id   << "}"  << std::endl;
        ofs << "  pose: " << std::endl;
        ofs << "    position: {" ;
        ofs << "x: " << finish_pose_.pose.position.x << ", " ;
        ofs << "y: " << finish_pose_.pose.position.y << ", " ;
        ofs << "z: " << finish_pose_.pose.position.z << "}"  << std::endl;
        ofs << "    orientation: {" ;
        ofs << "x: " << finish_pose_.pose.orientation.x << ", " ;
        ofs << "y: " << finish_pose_.pose.orientation.y << ", " ;
        ofs << "z: " << finish_pose_.pose.orientation.z << ", " ;
        ofs << "w: " << finish_pose_.pose.orientation.w << "}"  << std::endl;

        ofs.close();
        ROS_INFO_STREAM("write success");
    }
    


    void run()
    {
        ros::spin();
    }



private:
    ros::Subscriber waypoints_viz_sub_;
    ros::Subscriber waypoints_joy_sub_;
    ros::Subscriber finish_pose_sub_;
    ros::Publisher markers_pub_;
    std::vector<Waypoint> waypoints_;
    std::vector<visualization_msgs::Marker> markers_;
    geometry_msgs::PoseStamped finish_pose_;
    tf::TransformListener tf_listener_;
    int save_joy_button_;
    ros::NodeHandle nh_;
    std::string filename_;
    std::string world_frame_;
    std::string robot_frame_;
    float default_rad_;
};





int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoints_saver");
    WaypointsSaver saver;
    saver.run();

    return 0;
}
