#!/bin/bash

export LAWNNY5_ROOT=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source /opt/ros/iron/setup.bash
colcon build
source ./install/setup.bash
ros2 launch lawnny5 lawnny5_launch.yaml