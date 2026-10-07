#!/bin/bash

lock_file=/tmp/rb2301_gz_ca2.lock
exec 9>"${lock_file}"
if ! flock -n 9; then
    echo "gz_ca2 is already running; close the existing launch before starting another."
    exit 1
fi

source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="rb2301_ca2_$$"
exec ros2 launch rb2301_gz ca2_gazebo.launch.py x:=0.0 y:=0.0
