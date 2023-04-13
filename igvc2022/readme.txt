0-1. 大会側から知らされるGPSウェイポイントの緯度経度を igvc2022/config/gps_parameters.yaml に記入
0-2. ロボットをスタート位置に置き、igvc2022/src/get_waypoints_from_gps.py を実行 ->  igvc2022/config/gps_waypoints/gps_waypoints.yamlにスタート位置からのXY座標に変換されたウェイポイントが保存される

1. 電源を入れ、igvc2022/launch/start.launch を実行
　・各デバイスを起動
　・白線認識用のノード起動
　・白線が認識されやすいパラメータに調整

2．igvc2022/launch/no_map_navigation.launch　を実行
　・gmapping, move_base, rvizなどを起動
　・ナビゲーション用のノードを起動
　 →走行開始
