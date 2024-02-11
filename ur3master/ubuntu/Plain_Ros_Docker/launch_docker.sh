#!/bin/bash

docker run -it --user ros --net host --add-host nick-ros-master:127.0.0.1 -v /home/$USER/GitHub/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_plain_docker_image