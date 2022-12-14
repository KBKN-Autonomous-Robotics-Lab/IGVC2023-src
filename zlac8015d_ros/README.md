# ROS package for ZLAC8015D dual-channel servo driver  
[![](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)](https://github.com/Alpaca-zip/zlac8015d_ros)

![circuit_scheme](https://user-images.githubusercontent.com/84959376/191182537-47d49329-7e61-4c6c-b6a7-7eab8f530f07.png)

## 1 Installation
```
$ cd catkin_ws/src 

$ git clone https://github.com/Alpaca-zip/zlac8015d_ros.git

$ cd .. && catkin_make
```
## 2 Usage 
### Run motor_driver_node
```
$ roslaunch zlac8015d_ros motor_driver_node.launch
```
### Parameters
- `port`: Name of the zlac8015d port. Default is `dev/ttyUSB0`.
- `control_mode`: `1` is relative position control mode, `3` is speed rpm control mode. Default is `3`.
- `debug`: If `true`, odometry information is displayed. Default is `false`.
- `twist_cmd_vel_topic`: Topic name for Twist-type messages. Default is `/zlac8015d/twist/cmd_vel`.
- `cmd_vel_topic`: Topic name for cmd_vel[m/s] messages. Default is `/zlac8015d/vel/cmd_vel`.
- `cmd_rpm_topic`: Topic name for cmd_rpm messages. Default is `/zlac8015d/vel/cmd_rpm`.
- `cmd_deg_topic`: Topic name for cmd_deg[°] messages. Default is `/zlac8015d/pos/cmd_deg`.
- `cmd_dist_topic`: Topic name for cmd_dist[m] messages. Default is `/zlac8015d/pos/cmd_dist`.
- `publish_TF`: If `true`, TF is published. Default is `true`.
- `TF_header_frame`: Name of the TF header frame. Default is `odom`.
- `TF_child_frame`: Name of the TF child frame. Default is `base_link`.
- `publish_odom`: If `true`, `/odom` is published. Default is `true`.
- `odom_header_frame`: Name of the odometry header frame. Default is `odom`.
- `odom_child_frame`: Name of the odometry child frame. Default is `base_link`.
- `left_wheel_radius`: Left wheel radius. Default is `0.1015`[m].
- `right_wheel_radius`: Right wheel radius. Default is `0.1015`[m].
- `computation_left_wheel_radius`: Radius of left wheel used for odometry computation. Default is `0.1015`[m].
- `computation_right_wheel_radius`: Radius of right wheel used for odometry computation. Default is `0.1015`[m].
- `cpr`: CPR(Counts Per Revolution). Default is `16385`.
- `wheels_base_width`: Distance between tires. Default is `0.5668`[m].
- `callback_timeout`: Motor automatically stops if no topics are received for a certain period of time. Default is `0.5`[s].
- `set_accel_time_left`: Acceleration time for left tire. Default is `200`[ms].
- `set_accel_time_right`: Acceleration time for right tire. Default is `200`[ms].
- `set_decel_time_left`: Deceleration time for left tire. Default is `200`[ms].
- `set_decel_time_right`: Deceleration time for right tire. Default is `200`[ms].
- `max_left_rpm`: Maximum rpm of left tire. Default is `150`.
- `max_right_rpm`: Maximum rpm of right tire. Default is `150`.
- `deadband_rpm`: Width of rpm to be regarded as 0. If `3`, then -3 to 3 is considered rpm 0. Default is `3`.

### Topics
This node publishes the following topics.
- `/wheels_rpm`: The speed in RPM of each tire as [left, right].
- `/odom`: Odometry data for the robot. [More detail](http://docs.ros.org/en/diamondback/api/nav_msgs/html/msg/Odometry.html)

This node subscribes to the following topics.
- `/zlac8015d/twist/cmd_vel`: Send command as linear velocity and angular velocity in speed rpm control. [More detail](https://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/Twist.html)
- `/zlac8015d/vel/cmd_vel`: Send command as velocity in speed rpm control, e.g. [0.6, 0.5] 0.6[m/s] of left tire, 0.5[m/s] of right tire.
- `/zlac8015d/vel/cmd_rpm`: Send command as rpm in speed rpm control, e.g. [100, 50] 100 rpm of left wtire, 50 rpm of right tire.
- `/zlac8015d/pos/deg_cmd`: Send command as angle degree in position control, e.g. [90,70] 90 [deg] of left tire, 70 [deg] of right tire.
- `/zlac8015d/pos/dist_cmd`: Send command as desired travelling distance in position control, e.g. [1.0, 1.0] for 1[m] travelling distance of each tire.
- `/estop`: Send command as emergency stop signal, e.g. true for emergency stop is activated.

### Emergency stop Feature
The motor is locked when you publish a `true` message on the `/estop` topic.

This feature is not recommended. Emergency stop should be controlled by hardware.
