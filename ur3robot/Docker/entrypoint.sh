#!/bin/bash

set -e

export ROS_HOSTNAME=ur3robot #.local
export ROS_MASTER_URI=http://ur3robot:11311 #was .local
source /opt/ros/melodic/setup.bash

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyACM1
sudo chmod 777 /dev/video0
sudo chmod 777 /dev/video1 #removing kinect for temporary testing
sudo chmod 777 /dev/i2c-1
sudo chmod 777 -R /dev/bus/usb

cd ~/catkin_ws
catkin_make --only-pkg-with-deps ur3robot
cd ~

echo "Provided arguments: $@"

exec $@
