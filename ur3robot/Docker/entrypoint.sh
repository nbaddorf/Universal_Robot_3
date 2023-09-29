#!/bin/bash

set -e

export ROS_HOSTNAME=ur3robot.local
export ROS_MASTER_URI=http://ur3robot.local:11311
source /opt/ros/melodic/setup.bash

sudo chmod 777 /dev/ttyUSB0

echo "Provided arguments: $@"

exec $@
