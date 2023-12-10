#!/bin/bash

set -e

export ROS_HOSTNAME=ur3robot.local
export ROS_MASTER_URI=http://ur3robot.local:11311
source /opt/ros/melodic/setup.bash

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/video0
sudo chmod 777 /dev/video1
sudo chmod 777 /dev/i2c-1
sudo chmod 777 -R /dev/bus/usb

catkin_make --only-pkg-with-deps ur3robot

echo "Provided arguments: $@"

exec $@
