#!/bin/bash

docker run -it --user ros --network=host --ipc=host -v /home/$USER/Universal_Robot_3/ur3robot/ros_code:/ros_code ur3robot_docker_image
