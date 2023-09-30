FROM ros:melodic


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    iputils-ping \
    net-tools \
    ros-melodic-video-stream-opencv \
    ros-melodic-joint-state-publisher\
    ros-melodic-robot-state-publisher\
    ros-melodic-rosserial-python\
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

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash &&\
    mkdir -p /home/${USERNAME}/catkin_ws/src &&\
    cd /home/${USERNAME}/catkin_ws/src &&\
    git clone https://github.com/Slamtec/rplidar_ros.git &&\
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
