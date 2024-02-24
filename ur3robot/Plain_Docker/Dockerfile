FROM ros:melodic


RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

RUN apt-get update \
    && apt-get install -y \
    nano \
    iputils-ping \
    net-tools \
    cmake \
    build-essential \
    libusb-1.0-0-dev \
    ros-melodic-video-stream-opencv \
    ros-melodic-joint-state-publisher\
    ros-melodic-robot-state-publisher\
    ros-melodic-rosserial-python\
    ros-melodic-teleop-twist-keyboard\
    ros-melodic-amcl\
    ros-melodic-move-base\
    #libfreenect-dev \
    ros-melodic-freenect-launch\
    #ros-melodic-freenect-stack \
    ros-melodic-robot-localization\
    ros-melodic-slam-gmapping\
    ros-melodic-map-server\
    ros-melodic-pcl-ros\
    ros-melodic-pointcloud-to-laserscan \
    #ros-melodic-
    && rm -rf /var/lib/apt/lists/*

  RUN apt-get update \
    && apt-get install -y \
    ros-melodic-dwa-local-planner \
    ros-melodic-laser-scan-matcher \
    ros-melodic-rosserial-server \
    && rm -rf /var/lib/apt/lists/*


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

USER ros

#link to imu github: https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver This has the two repositories, i2c devices and mpu6050

#I think that the kinect doesnt work if the usb ports arnt being used sort of. It seems to work fine when running the rest of the robot, or even streeming the kinect camera, andthen the pointcloud.
#RUN /bin/bash -c 'cd /home/${USERNAME} \
#    && git clone https://github.com/OpenKinect/libfreenect \
#    && cd libfreenect \
#    && mkdir build \
#    &&  cd build \
#    && cmake -L .. \
#    && make'


#RUN apt-get install -y 


RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash &&\
    mkdir -p /home/${USERNAME}/catkin_ws/src &&\
    cd /home/${USERNAME}/catkin_ws/src &&\
    git clone https://github.com/Slamtec/rplidar_ros.git &&\
    git clone https://github.com/Brazilian-Institute-of-Robotics/i2c_device_ros.git &&\
    git clone https://github.com/mateusmenezes95/mpu6050_driver.git &&\
    #git clone https://github.com/muhrix/kinect_aux.git &&\
    #git clone https://github.com/corot/rail_mesh_icp.git &&\
    git clone https://github.com/GT-RAIL/rail_mesh_icp.git &&\
    #below is for xbox kinect, freenect requirements
    #git clone https://github.com/ros-drivers/freenect_stack.git &&\
    #git clone https://github.com/ros-drivers/rgbd_launch.git &&\
    #git clone https://github.com/ros-perception/image_common.git &&\
    #git clone https://github.com/ros-perception/vision_opencv.git &&\
    #git clone https://github.com/ros-perception/image_pipeline.git &&\
    #git clone https://github.com/ros/geometry2.git &&\
    cd /home/${USERNAME}/catkin_ws &&\
    catkin_make'

#RUN '
#    && rm -rf /var/lib/apt/lists/*'

USER root

#RUN 'chmod 777 /dev/USB0'

WORKDIR /home/${USERNAME}

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
