# orange_mapping
This package provides slam functionality for the orange robot.
## Setup
```
./setup.sh
```
## LIO-SAM
- From gazebo simulation
```
roslaunch orange_mapping lio_sam_slam.launch
roslaunch tsukuba2022 start_sim.launch use_ekf:=false
rosservice call /lio_sam/save_map 1.0 "/Downloads/LOAM/"
```
If the map drifts frequently, run the following command instead.
```
roslaunch orange_mapping lio_sam_slam.launch rm_imu_acc:=true
```
- From rosbag
```
roslaunch orange_mapping lio_sam_slam.launch create_map_from_rosbag:=true
rosbag play your_bag.bag -r 1 --clock
rosservice call /lio_sam/save_map 1.0 "/Downloads/LOAM/"
```
