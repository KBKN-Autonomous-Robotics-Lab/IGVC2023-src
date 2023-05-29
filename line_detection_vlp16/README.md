# Line detection with velodyne (vlp16) intensity

## 1. パラメータの設定
まずは、velodyneとline_detectionを起動する。start.launchの引数 use_vlp16 をtrueに設定して実行する。

```
$ start use_vlp16:=true
($ start_sim use_vlp16:=true)
```

次に、rqt_reconfigureを起動する。

```
$ rosrun rqt_reconfigure rqt_reconfigure
```

データを確認するためにrvizを起動する・
```
$ roscd line_detection_vlp16/rviz
$ rviz -d rviz_cfg.rviz
```

rqt_reconfigureの左側のリストの中から line_detectioin をクリックし、パラメータを調整する。  
以下はパラメータの意味↓

- angle_width [rad] : 白線を認識するために使用する点群の、水平方向の角度範囲。左右の白線がある程度余裕をもってうつっていればよい。
- intensity_threshold : 白線とそれ以外の反射強度を区別するためのしきい値。ある程度大きい値から下げながら適切な値を探す。
- max_slope [rad] : 地面の最大傾斜角度。点群の中でこの値を超える傾斜になる箇所は障害物として認識する。
- max_height [m] : この値を超える高さにある点群は障害物として認識する。

rviz上には、障害物と認識した点群が白色、左右の白線がそれぞれ異なる色の大きな点として表示され、地面の点群の色は反射強度によって変わる。  
まずは、障害物のない場所でintensity_thresholdを調整して白線を認識できることを確認し、その後障害物のある状態でmax_slopeとmax_heightを調整するとよい。  

調整が終わったら、line_detection_vlp16/param/line_detection_params.yaml に値を書き込んでおく。

<br>

## 2. 自律走行
引数 use_vlp16 にtrueを設定して起動する。

```
$ start use_vlp16:=true
$ roslaunch igvc2023 no_map_navigation.launch use_vlp16:=true
```

どちらかを忘れても走り出してしまうので注意。
また、直接launchファイルの中の arg を書き換えてしまってもよい。