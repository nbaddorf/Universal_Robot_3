#!/bin/bash

set -e

export ROS_HOSTNAME=ur3robot
source /opt/ros/melodic/setup.bash

echo "Provided arguments: $@"

exec $@
