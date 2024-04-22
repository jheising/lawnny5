More documentation coming soon. These are just my personal post-it notes for now.

# Installing Docker on Raspberry Pi
https://docs.docker.com/engine/install/debian/

# Moving files from local to Rpi
`rsync -avz ./ jheising@lawnny5.local:~/`

# Building and running image

Build

`docker build -t lawnny5-ros .`

Run as interactive

`docker run --rm -it --privileged -p 8000:8000 -p 9090:9090 -v /dev/bus/usb:/dev/bus/usb -v ./ros2_workspace/src:/root/ros2_ws/src --device-cgroup-rule='c 189:* rmw' --name lawnny5-ros lawnny5-ros`

Run as background service at boot time

`docker run -it --privileged -p 8000:8000 -p 9090:9090 -v /dev/bus/usb:/dev/bus/usb -v ./ros2_workspace/src:/root/ros2_ws/src --device-cgroup-rule='c 189:* rmw' -d --name lawnny5-ros --restart always lawnny5-ros`

Create terminal into running container

`docker exec -it 56218417f957a86ecb9497fc252b1e9239023ebda1af49a5d72ac50178aa153a bash`

# Start wireless AP
sudo nmcli device wifi hotspot ssid lawnny5 password NeedMowInput ifname wlan0

### Stop
sudo nmcli device disconnect wlan0 && sudo nmcli device up wlan0

# Simulation
https://www.codeluge.com/post/setting-up-docker-on-macos-m1-arm64-to-use-debian-10.4-docker-engine/

On Mac:

`ssh debian@127.0.0.1 -p 22022 -N -L/tmp/docker-on-debian.sock:/var/run/docker.sock ssh://debian@127.0.0.1`


`gz sim -v 4 -s sim/simple_world.sdf`
`ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=sim/ros_gz_bridge_config.yaml`