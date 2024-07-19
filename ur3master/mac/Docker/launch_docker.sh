#!/bin/bash

#docker run -it --user ros --network=host --ipc=host -v /home/$USER/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3robot ur3master_docker_image
#docker run -it -p 8080:8080 --add-host ur3robot:ur3robot.local --user ros -v /Users/$USER/Documents/GitHub/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_docker_image
#host ur3robot gives address
docker run -it --net=host --user ros --add-host Nicks-MBP:127.0.0.1 -v /Users/$USER/Documents/GitHub/Universal_Robot_3/ur3master/ros_code/ur3master:/home/ros/catkin_ws/src/ur3master  -v /Users/$USER/Documents/GitHub/Universal_Robot_3/ur3master/ros_code/scara:/home/ros/catkin_ws/src/scara  -v /Users/$USER/Documents/GitHub/Universal_Robot_3/ur3master/ros_code/scara_scara_arm_ikfast_plugin:/home/ros/catkin_ws/src/scara_scara_arm_ikfast_plugin ur3master_docker_image