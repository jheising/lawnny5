# Installing Docker on Raspberry Pi
https://docs.docker.com/engine/install/debian/

# Moving files from local to Rpi
`rsync -avz ./ jheising@lawnny5.local:~/`

# Building and running image
`docker build -t lawnny5-ros .`
`docker run --rm -it --privileged -p 8000:8000 -p 9090:9090 -v /dev/bus/usb:/dev/bus/usb -v ./ros2_workspace/src:/root/ros2_ws/src --device-cgroup-rule='c 189:* rmw' --name lawnny5-ros lawnny5-ros`
`docker run -it --privileged -p 8000:8000 -p 9090:9090 -v /dev/bus/usb:/dev/bus/usb -v ./ros2_workspace/src:/root/ros2_ws/src --device-cgroup-rule='c 189:* rmw' -d --name lawnny5-ros --restart always lawnny5-ros`


docker exec -it 56218417f957a86ecb9497fc252b1e9239023ebda1af49a5d72ac50178aa153a bash