#!/bin/bash

SCRIPT_DIR="$(cd $(dirname ${BASH_SOURCE:-$0}) || return; pwd)"
SRC_DIR="$(dirname "$SCRIPT_DIR")"
ROOT_DIR="$(dirname "$SRC_DIR")"

#IGVC2023-src dependency
python3 -m pip install --user --upgrade --no-cache-dir --no-warn-script-location \
pymodbus \
numpy-quaternion \
ruamel.yaml \
Pillow

apt-get install --no-install-recommends -y \
libpcap-dev \
libsdl-image1.2-dev \
libsdl-dev \

#Another dependency
cd "$SRC_DIR" || return
rosdep install --default-yes --from-paths $ROOT_DIR/src --ignore-src -r

#build
apt-get update && apt-get upgrade -y
catkin build
