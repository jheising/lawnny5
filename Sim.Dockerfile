FROM ros:iron

RUN sudo apt-get update -y # && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f ros-iron-rosbridge-suite ros-iron-webots-ros2 python3-colcon-common-extensions python3-pip ros-iron-webots-ros2 wget

RUN python3 -m pip install pysabertooth depthai

RUN mkdir -p /root/ros2_ws/src

## Make sure ROS is always setup when loading our shell
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Add webots
RUN sudo mkdir -p /etc/apt/keyrings && \
    cd /etc/apt/keyrings && \
    sudo wget -q https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
RUN sudo apt-get update -y && sudo apt-get install -y -f webots

# Build Webots
#WORKDIR  /root
#RUN git clone --recurse-submodules -j8 https://github.com/cyberbotics/webots.git && \
#    cd webots && \
#    sudo scripts/install/linux_compilation_dependencies.sh && \
#    cat scripts/install/bashrc.linux >> ~/.bashrc && \
#    source ~/.bashrc && \
#    make -j12

ENV WEBOTS_HOME="/usr/local/webots"
ENV LAWNNY5_ROOT="/root/ros2_ws/src"

WORKDIR /root/ros2_ws

# ENV WEBOTS_HOME=/usr/local/webots
# source install/local_setup.bash && /opt/ros/iron/share/webots_ros2_driver/scripts/webots-controller --robot-name=Lawnny5 --ip-address=host.docker.internal --protocol=tcp --port=1234 ros2 --ros-args -p robot_description:=/root/ros2_ws/install/lawnny5/share/lawnny5/resource/lawnny5.urdf