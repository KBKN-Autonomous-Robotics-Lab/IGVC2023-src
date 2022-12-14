// #include <geometry_msgs/PoseArray.h>
// #include <waypoint_saver.h> // include Waypoint class
// #include <vector>


// class WaypointArray : public geometry_msgs::PoseArray {
//     /*
//     std_msgs/Header header
//     geometry_msgs/Pose[] poses
//     */
// public:
//     std::vector<Waypoint> waypoint_list;


//     void add_waypoint(double x, double y, double z, float vel, float rad)
//     {
//         Waypoint point;
//         point.set_x(x);
//         point.set_y(y);
//         point.set_z(z);
//         point.set_vel(vel);
//         point.set_rad(rad);
//         waypoint_list.push_back(point);
//     }


//     geometry_msgs::PoseArray get_posearray_msg()
//     {
//         geometry_msgs::PoseArray posearray;
//         posearray.header = header;
//         posearray.poses = poses;
//         return posearray;
//     }
// };