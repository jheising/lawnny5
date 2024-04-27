FROM ros:humble-ros-base-jammy

RUN sudo apt-get update -y # && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f ros-humble-rosbridge-suite \
    python3-colcon-common-extensions \
    python3-pip \
    ffmpeg libsm6 libxext6

RUN mkdir -p /root/lawnny5/src

## Make sure ROS is always setup when loading our shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENV LAWNNY5_ROOT="/root/lawnny5/src"

WORKDIR /root/lawnny5

# We have to lock depthai to 2.20.2 because of this issue https://github.com/geaxgx/depthai_blazepose/issues/37
RUN python3 -m pip install pysabertooth depthai==2.20.2 opencv-python

COPY ./brain/* /root/lawnny5/src

RUN . /opt/ros/humble/setup.sh && colcon build

# CMD . install/local_setup.sh && ros2 launch lawnny5 robotulate_launch.yaml