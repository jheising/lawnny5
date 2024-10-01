FROM ros:humble-ros-base-jammy

WORKDIR /root

RUN sudo apt-get update -y && sudo apt-get upgrade -y -f

# Install Python
RUN sudo apt-get install -y -f python3-pip

#Install stuff for Ardupilot
RUN sudo apt-get install -y -f avahi-utils

# MAVPROXY stuff
## Note, we can move this to the official distribution once https://github.com/ArduPilot/MAVProxy/pull/1440 is released
##RUN git clone https://github.com/ArduPilot/MAVProxy.git
#RUN python3 -m pip install websockets
#ADD https://api.github.com/repos/jheising/MAVProxy/git/refs/heads/master version.json
#RUN git clone https://github.com/jheising/MAVProxy.git
#RUN cd MAVProxy && python3 setup.py build install --user && cd .. && rm -R MAVProxy
#COPY brain/scripts/mavinit.scr /root/.mavinit.scr
#CMD ["sudo", "/root/.local/bin/mavproxy.py", "--master=/dev/ttyAMA0,921600", "--out=udpin:0.0.0.0:14550", "--daemon", "--moddebug=3", "--show-errors"]

# mavp2p stuff
RUN sudo apt-get install -y -f curl npm
RUN npm install n -g && n lts
RUN curl -LO https://github.com/bluenviron/mavp2p/releases/download/v1.2.1/mavp2p_v1.2.1_linux_arm64v8.tar.gz && tar -xvzf mavp2p_v1.2.1_linux_arm64v8.tar.gz && rm mavp2p_v1.2.1_linux_arm64v8.tar.gz
COPY brain/kitelink /root/kitelink
RUN cd kitelink && npm install --omit=dev

CMD ./mavp2p serial:/dev/ttyAMA0:921600 tcps:0.0.0.0:5760 --idle-timeout=3600s & kitelink/node_modules/.bin/tsx kitelink/server.ts

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

#COPY ./brain/ros2_workspace /root/ros2_workspace
#COPY ./brain/assets /root/assets

#RUN . /opt/ros/humble/setup.sh && colcon build

#CMD . /opt/ros/humble/setup.sh && . install/local_setup.sh && ros2 launch lawnny5 robotulate_launch.yaml

# docker run -d -i -t -v /root/src/brain/ros2_workspace:/root/ros2_workspace -v /root/cache:/root/cache -v /root/src/brain/assets:/root/assets -v /dev/bus/usb:/dev/bus/usb -v /run/dbus:/run/dbus -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket --name lawnny5-ros --rm --network host --device-cgroup-rule='c 189:* rmw' --privileged lawnny5-ros:latest