#!/bin/bash

set -e

source /opt/ros/melodic/setup.bash
source /home/ros/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=nick-ros-master #docker can resolve hosts from router, but not .local avahi
export ROS_MASTER_URI=http://ur3robot:11311

#exec supervisord -c /supervisord.conf &

#echo "Provided arguments: $@"
#echo "############################"
#echo ""
#echo "http://localhost:8080/vnc_auto.html"
#echo ""
#echo "Must start Rviz, just type rviz. "
#echo "############################\n"


exec $@

#echo "http://localhost:8080/vnc_auto.html"

