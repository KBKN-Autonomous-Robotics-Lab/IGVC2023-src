#!/bin/sh

MAPPATH="/home/ubuntu/catkin_ws/src/tsukuba2022/maps"
WPPATH="/home/ubuntu/catkin_ws/src/tsukuba2022/config/waypoints" 
index=`date +%Y%m%d%H%M`

cp "${MAPPATH}/mymap.pgm" "${MAPPATH}/mymap_${index}.pgm"
cp "${MAPPATH}/mymap.yaml" "${MAPPATH}/mymap_${index}.yaml"
cp "${WPPATH}/waypoints.yaml" "${WPPATH}/waypoints_${index}.yaml"