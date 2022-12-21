#!/bin/bash

SCRIPT_DIR="$(cd $(dirname ${BASH_SOURCE:-$0}) || return; pwd)"
SRC_DIR="$(dirname "$SCRIPT_DIR")"
ROOT_DIR="$(dirname "$SRC_DIR")"

#lio_sam dependency
sudo add-apt-repository --yes ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev

#Another dependency
cd "$SRC_DIR" || return
rosdep install --default-yes --from-paths $ROOT_DIR/src --ignore-src -r

#build
catkin build -j2
