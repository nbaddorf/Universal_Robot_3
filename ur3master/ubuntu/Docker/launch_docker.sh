#!/bin/bash

#docker run -it --user ros --network=host --ipc=host -v /home/$USER/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3robot ur3master_docker_image
#docker run -it -p 8080:8080 --add-host ur3robot:ur3robot.local --user ros -v /Users/$USER/Documents/GitHub/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_docker_image
#docker run -it -p 8080:8080 --user ros --add-host nick-ros-master:127.0.0.1 -v /home/$USER/Github/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_docker_image
docker run -it --user ros --network=host --ipc=host --add-host nick-ros-master:127.0.0.1 -v /home/$USER/GitHub/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_docker_image
#docker run -it -p 8080:8080 --user ros --add-host nick-ros-master:127.0.0.1 -v /home/$USER/Github/Universal_Robot_3/ur3master/ros_code:/home/ros/catkin_ws/src/ur3master ur3master_docker_image