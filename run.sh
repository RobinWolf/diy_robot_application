#!/bin/bash
##############################################################################
##                   Build the image, using dev.Dockerfile                  ##
##############################################################################
ROS_DISTRO=humble

uid=$(eval "id -u")
gid=$(eval "id -g")

#pass some arguments and settings to the dev.Dockerfile while building the image (dev.Dockerfile)
#name of the image builded here: diy-full-description/ros-render:"$ROS_DISTRO":"ROS-Distribution eg humble"
#dont use cached data to clone up-to date repos all the time
  #--no-cache \
docker build \
  --no-cache \
  --build-arg ROS_DISTRO="$ROS_DISTRO" \
  --build-arg UID="$uid" \
  --build-arg GID="$gid" \
  -f Dockerfile \
  -t diy-robot-application/ros-render:"$ROS_DISTRO" .

##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/hephaestus/ros2_ws/src
SRC_HOST="$(pwd)"/src

docker run \
  --name diy_robot_application\
  --rm \
  -it \
  --net=host \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -e DISPLAY="$DISPLAY" \
  diy-robot-application/ros-render:"$ROS_DISTRO" bash

# display and network access is already passed to the container