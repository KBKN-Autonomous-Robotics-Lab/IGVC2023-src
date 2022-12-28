#!/bin/bash

#IGVC2023-src dependency
python3 -m pip install --user --upgrade --no-cache-dir --no-warn-script-location \
pymodbus \
numpy-quaternion \
ruamel.yaml \
Pillow

apt-get install --no-install-recommends -y \
libpcap-dev \
libsdl-image1.2-dev \
libsdl1.2-dev \

#Another dependency
rosdep install --default-yes --from-paths . --ignore-src -r

#build
apt-get update && apt-get upgrade -y
catkin build -j2
