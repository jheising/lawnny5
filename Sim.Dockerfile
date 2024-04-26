FROM ros:humble-ros-base-jammy

RUN sudo apt-get update -y # && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f ros-humble-rosbridge-suite \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-colcon-common-extensions \
    python3-virtualenv \
    ros-humble-webots-ros2 \
    iproute2 \
    ros-humble-webots-ros2

RUN mkdir -p /root/lawnny5/src

## Make sure ROS is always setup when loading our shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENV WEBOTS_SHARED_FOLDER=/Users/jheising/Documents/Development/personal/lawnny5/development/brain/ros2_workspace/shared:/root/lawnny5/src/shared
ENV LAWNNY5_ROOT="/root/lawnny5/src"

WORKDIR /root/lawnny5

#RUN virtualenv -p python3 ./.venv
#RUN source ./.venv/bin/activate
# We have to lock depthai to 2.20.2 because of this issue https://github.com/geaxgx/depthai_blazepose/issues/37
RUN python3 -m pip install pysabertooth depthai==2.20.2 opencv-python

# https://www.theconstruct.ai/ros2-how-to-install-third-party-python-packages-using-ros2-5/

# docker run -d -i -t -v /home/debian/lawnny5/src:/root/lawnny5/src -v /dev/bus/usb:/dev/bus/usb --name lawnny5-ros-sim --rm --network host --device-cgroup-rule='c 189:* rmw' --privileged lawnny5-ros-sim:latest
# docker run -d -i -t -v /home/lawnny5/src/brain/ros2_workspace:/root/lawnny5/src -v /dev/bus/usb:/dev/bus/usb --name lawnny5-ros-sim --rm --network host --device-cgroup-rule='c 189:* rmw' --privileged lawnny5-ros-sim:latest
# RUN colcon build && source install/local_setup.bash

# CMD . /opt/ros/humble/setup.bash && . install/local_setup.bash && ros2 launch lawnny5 simulate_launch.py

# source install/local_setup.bash && /opt/ros/humble/share/webots_ros2_driver/scripts/webots-controller --robot-name=Lawnny5 --ip-address=192.168.1.6 --protocol=tcp --port=1234 ros2 --ros-args -p robot_description:=/root/lawnny5/install/lawnny5/share/lawnny5/resource/lawnny5.urdf

# python3 /opt/ros/humble/share/webots_ros2_driver/scripts/webots_tcp_client.py --port=1234 --batch --mode=realtime tmp97a4gzwv_world_with_URDF_robot.wbt'

# ros2 launch webots_ros2_universal_robot multirobot_launch.py