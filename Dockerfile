FROM ros:humble-ros-base-jammy

WORKDIR /root

RUN sudo apt-get update -y && sudo apt-get upgrade -y -f

# Install Python
RUN sudo apt-get install -y -f python3-pip

#Install stuff for Ardupilot
RUN sudo apt-get install -y -f avahi-utils
# Note, we can move this to the official distribution once https://github.com/ArduPilot/MAVProxy/pull/1440 is merged
RUN git clone https://github.com/jheising/MAVProxy.git
RUN cd MAVProxy && python3 setup.py build install --user && cd .. && rm -R MAVProxy
COPY brain/scripts/mavinit.scr /root

#RUN sudo apt-get install -y -f  \
#    ros-humble-rosbridge-suite \
#    ros-humble-rclpy-message-converter \
#    python3-colcon-common-extensions \
#    python3-pip \
#    avahi-utils

# RUN python3 -m pip install pysabertooth depthai==2.20.2 opencv-python pytweening mpyg321 openai elevenlabs

#RUN mkdir -p /root/lawnny/cache

#ENV LAWNNY_ROOT="/root/lawnny/ros2_workspace"
#ENV LAWNNY_CACHE="/root/lawnny/cache"
#ENV LAWNNY_ASSETS="/root/lawnny/assets"

COPY brain/scripts/mavinit.scr /root/.mavinit.scr

CMD ["sudo", "/root/.local/bin/mavproxy.py", "--master=/dev/ttyAMA0", "--out=udpin:0.0.0.0:14550", "--daemon"]

#COPY ./brain/ros2_workspace /root/ros2_workspace
#COPY ./brain/assets /root/assets

#RUN . /opt/ros/humble/setup.sh && colcon build

#CMD . /opt/ros/humble/setup.sh && . install/local_setup.sh && ros2 launch lawnny5 robotulate_launch.yaml

# docker run -d -i -t -v /root/src/brain/ros2_workspace:/root/ros2_workspace -v /root/cache:/root/cache -v /root/src/brain/assets:/root/assets -v /dev/bus/usb:/dev/bus/usb -v /run/dbus:/run/dbus -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket --name lawnny5-ros --rm --network host --device-cgroup-rule='c 189:* rmw' --privileged lawnny5-ros:latest