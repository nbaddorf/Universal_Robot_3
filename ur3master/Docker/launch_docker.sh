#!/bin/bash

#docker run -it --user ros --network=host --ipc=host -v 
/home/$USER/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3robot ur3master_docker_image
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:rw  --env=DISPLAY --network=host --ipc=host ur3master_docker_image
