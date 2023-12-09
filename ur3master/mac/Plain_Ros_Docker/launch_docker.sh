#!/bin/bash

docker run -it --user ros --net host --add-host Nicks-MBP:127.0.0.1 -v /Users/$USER/Documents/GitHub/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_plain_docker_image