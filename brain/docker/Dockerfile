FROM ros:iron

# For node
# RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash

RUN sudo apt-get update -y && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f ros-iron-rosbridge-suite python3-colcon-common-extensions python3-pip

RUN python3 -m pip install pysabertooth depthai

RUN mkdir -p /root/ros2_ws/src

## Make sure ROS is always setup when loading our shell
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# ADD ../ros2_workspace /root/ros2_ws/

WORKDIR /root/ros2_ws

# RUN colcon build

#CMD ros2 launch lawnny5 lawnny5_launch.yaml

# colcon build && source install/local_setup.bash && ros2 launch lawnny5 lawnny5_launch.yaml