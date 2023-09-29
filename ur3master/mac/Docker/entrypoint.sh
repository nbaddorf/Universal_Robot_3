#!/bin/bash



set -e

source /opt/ros/melodic/setup.bash
export ROS_HOSTNAME=Nicks-MacBook-Pro.local
export ROS_MASTER_URI=http://ur3robot.local:11311

#echo 192.168.86.46   ur3robot >> /etc/hosts

exec supervisord -c /supervisord.conf &

echo "Provided arguments: $@"
echo "############################"
echo ""
echo "http://localhost:8080/vnc_auto.html"
echo ""
echo "Must start Rviz, just type rviz. "
echo "############################\n"


exec $@

#echo "http://localhost:8080/vnc_auto.html"

