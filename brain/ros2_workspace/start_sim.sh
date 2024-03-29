#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/iron/setup.bash
source ./install/setup.bash
/opt/ros/iron/share/webots_ros2_driver/scripts/webots-controller --robot-name=Lawnny5 --ip-address=host.docker.internal --protocol=tcp --port=1234 ros2 --ros-args -p robot_description:=/root/ros2_ws/install/lawnny5/share/lawnny5/resource/lawnny5.urdf