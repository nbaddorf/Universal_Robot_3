#!/bin/bash

#docker run -it --user ros --network=host --ipc=host ur3robot_docker_image
docker run -it --user ros --network=host --add-host ur3robot:ur3robot.local --ipc=host -v /home/$USER/Universal_Robot_3/ur3robot/ros_code:/home/ros/catkin_ws/src/ur3robot ur3robot_docker_image
