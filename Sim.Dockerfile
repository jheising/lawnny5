FROM ros:iron

RUN sudo apt-get update -y && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f ros-iron-rosbridge-suite python3-colcon-common-extensions python3-pip wget

RUN python3 -m pip install pysabertooth depthai

RUN mkdir -p /root/ros2_ws/src

## Make sure ROS is always setup when loading our shell
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

ENV LAWNNY5_ROOT="/root/ros2_ws"
WORKDIR /root/ros2_ws

# Add webots
RUN sudo mkdir -p /etc/apt/keyrings && \
    cd /etc/apt/keyrings && \
    sudo wget -q https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
RUN sudo apt-get update -y && sudo apt-get install -y -f webots ros-iron-webots-ros2
ENV WEBOTS_HOME="/usr/local/webots"

# ENV WEBOTS_HOME=/home/ubuntu/.ros/webotsR2023b/webots
# /opt/ros/iron/share/webots_ros2_driver/scripts/webots-controller --robot-name=Lawnny5 --ip-address=host.docker.internal --protocol=tcp --port=1234 ros2 --ros-args -p robot_description:=/root/ros2_ws/install/lawnny5/share/lawnny5/resource/lawnny5.urdf