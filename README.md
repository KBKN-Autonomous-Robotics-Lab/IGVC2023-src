# IGVC2023-src
This package provides for the development of an orange robot to compete in igvc2023.
## Setup
⚠️ The following command deletes everything in `src`
```
cd ~/catkin_ws/src
sudo rm -rf *
git clone https://github.com/KBKN-Autonomous-Robotics-Lab/IGVC2023-src.git .
sudo bash setup.sh
```
##使い方
0-1. 大会側から知らされるGPSウェイポイントの緯度経度を igvc2022/config/gps_parameters.yaml に記入
場所に応じてcountry_idとazimuthも変更する。

0-2. ロボットをスタート位置に置き、igvc2022/src/get_waypoints_from_gps.py を実行 ->  igvc2022/config/gps_waypoints/gps_waypoints.yamlにスタート位置からのXY座標に変換されたウェイポイントが保存される

1. 電源を入れ、igvc2022/launch/start.launch を実行
　・各デバイスを起動
　・白線認識用のノード起動
　・白線が認識されやすいパラメータに調整
　　guiが立ち上がって
　　上の調節は左から2番目を見て調整
　　下の調整は左から4番目を見て調整

2．igvc2022/launch/no_map_navigation.launch　を実行
.yamlの変更し忘れ注意！
 robot_localizationの場合は <arg name="odom_topic" default="/odometry/filtered"/>
 robot_pose_ekfの場合は <arg name="odom_topic" default="/combine_dr_measurements/odom_combined"/>
　・gmapping, move_base, rvizなどを起動
　・ナビゲーション用のノードを起動
　 →走行開始

<!-- Select node -->
<param name="node_array" value="121215"/>
<param name="node_index" value="1"/>
1→白線追従
2→ウェイポイントナビゲーション
3→ramp探索
4→rampゴール
5→ゴール
local_goal_setter2.pyの240行目distanceの所(ウェイポイントに何m近づいてゴール判定にするか)も変更

##その他
zlacのpymodbusのversion
pip install pymodbus==v2.5.3
