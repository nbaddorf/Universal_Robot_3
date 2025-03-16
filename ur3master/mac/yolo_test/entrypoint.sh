#!/bin/bash

set -e

source /opt/ros/melodic/setup.bash

export ROS_HOSTNAME=nicks-mbp #For some reason the robot can see dns from router, but my docker cant see ur3robot dns.
export ROS_MASTER_URI=http://ur3robot.local:11311 # THIS WORKS, BUT YOU HAVE TO TURN OFF DNS ADDBLOCKER

export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}] [${logger}]: ${message}'
#export ROS_IP=192.168.86.25
exec supervisord -c /supervisord.conf &

#cd ~/catkin_ws
#catkin_make --only-pkg-with-deps ur3master
#cd ~

echo "Provided arguments: $@"
echo "############################"
echo ""
echo "http://localhost:8080/vnc_auto.html"
echo ""
echo "Must start Rviz, just type rviz. "
echo "############################\n"


exec $@

#echo "http://localhost:8080/vnc_auto.html"

