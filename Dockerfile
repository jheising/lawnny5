FROM ros:iron

# For node
# RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash

RUN sudo apt-get update -y && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f ros-iron-rosbridge-suite python3-colcon-common-extensions python3-pip

RUN python3 -m pip install pysabertooth depthai

RUN mkdir -p /root/ros2_ws/src

## Make sure ROS is always setup when loading our shell
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Copy our ROS package source code
#COPY brain/ros2_workspace/src /root/ros2_ws/src

# Copy our web controller
# COPY controller/web-ui/dist /root/ros2_ws/www

#ENV LAWNNY5_ROOT="/root/ros2_ws"
WORKDIR /root/ros2_ws

# RUN colcon build

# colcon build && source install/local_setup.bash && ros2 launch lawnny5 lawnny5_launch.yaml

# CMD . /opt/ros/iron/setup.sh && . install/setup.sh && ros2 launch lawnny5 lawnny5_launch.yaml