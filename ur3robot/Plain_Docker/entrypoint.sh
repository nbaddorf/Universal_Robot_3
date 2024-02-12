#!/bin/bash

set -e

export ROS_HOSTNAME=ur3robot #.local
export ROS_MASTER_URI=http://ur3robot:11311 #was .local
source /opt/ros/melodic/setup.bash

echo "Provided arguments: $@"

exec $@
