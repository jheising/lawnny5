FROM ros:humble-ros-base-jammy

RUN sudo apt-get update -y && sudo apt-get upgrade -y -f
RUN sudo apt-get install -y -f  \
    ros-humble-rosbridge-suite \
    ros-humble-rclpy-message-converter \
    python3-colcon-common-extensions \
    python3-pip \
    bluetooth bluez-alsa-utils mpg123 mpv

RUN mkdir -p /root/lawnny5/cache

ENV LAWNNY5_ROOT="/root/lawnny5/ros2_workspace"
ENV LAWNNY5_CACHE="/root/lawnny5/cache"
ENV LAWNNY5_ASSETS="/root/lawnny5/assets"

WORKDIR /root/lawnny5

RUN python3 -m pip install pysabertooth depthai==2.20.2 opencv-python pytweening mpyg321 openai elevenlabs mavproxy

# CMD ["sudo", "mavproxy.py", "--master=/dev/ttyAMA0", "--out=udpin:0.0.0.0:14550", "--daemon"]

#COPY ./brain/ros2_workspace /root/lawnny5/ros2_workspace
#COPY ./brain/assets /root/lawnny5/assets

#RUN . /opt/ros/humble/setup.sh && colcon build

#CMD . /opt/ros/humble/setup.sh && . install/local_setup.sh && ros2 launch lawnny5 robotulate_launch.yaml

# docker run -d -i -t -v /home/lawnny5/cache:/root/lawnny5/cache -v /dev/bus/usb:/dev/bus/usb -v /run/dbus:/run/dbus --name lawnny5-ros --rm --network host --device-cgroup-rule='c 189:* rmw' --privileged -e OPENAI_API_KEY=... -e ELEVEN_LABS_API_KEY=... -e DEEPGRAM_API_KEY=... lawnny5-ros:latest