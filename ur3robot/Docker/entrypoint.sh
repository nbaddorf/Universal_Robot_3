#!/bin/bash

set -e

export ROS_HOSTNAME=ur3robot
source /opt/ros/melodic/setup.bash

sudo chmod 777 /dev/ttyUSB0

echo "Provided arguments: $@"

exec $@
