#!/bin/bash

#docker run -it --user ros --network=host --ipc=host ur3robot_docker_image
#docker run -it --user ros --network=host --add-host ur3robot:Nicks-MBP --ipc=host -v /home/$USER/Universal_Robot_3/ur3robot/ros_code:/home/ros/catkin_ws/src/ur3robot ur3robot_docker_image
docker run -it --user ros --network=host --add-host ur3robot.local:127.0.1.1  --device=/dev/ttyUSB0 --device=/dev/bus/usb --device=/dev/i2c-1 --device=/dev/video0 --device=/dev/video1 --device=/dev/ttyACM0 --ipc=host -v /home/$USER/Universal_Robot_3/ur3robot/ros_code:/home/ros/catkin_ws/src/ur3robot ur3robot_docker_image
#docker run -it --user ros --network=host --add-host ur3robot.local:127.0.1.1  --device=/dev/ttyUSB0 --device=/dev/bus/usb --device=/dev/i2c-1 --device=/dev/video0 --device=/dev/ttyACM0 --ipc=host -v /home/$USER/Universal_Robot_3/ur3robot/ros_code:/home/ros/catkin_ws/src/ur3robot ur3robot_docker_image