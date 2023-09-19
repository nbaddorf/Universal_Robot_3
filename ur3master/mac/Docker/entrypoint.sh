#!/bin/bash

set -e

source /opt/ros/melodic/setup.bash
export ROS_HOSTNAME=ur3master
export ROS_MASTER_URI=http://ur3robot:11311

echo "192.168.86.46   ur3robot >> /etc/hosts"

exec supervisord -c /supervisord.conf &

echo "Provided arguments: $@"

exec $@

echo "http://localhost:8080/vnc_auto.html"

