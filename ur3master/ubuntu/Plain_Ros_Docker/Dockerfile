FROM osrf/ros:melodic-desktop-full

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    iputils-ping \
    net-tools \
    wget \
    novnc \
    supervisor \
    x11vnc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
    && apt-get install -y \
    ros-melodic-image-pipeline \
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
#COPY bashrc /home/${USERNAME}/.bashrc
#COPY supervisord.conf /supervisord.conf
#COPY conf.d /conf.d

USER ros


RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash &&\
    mkdir -p /home/${USERNAME}/catkin_ws/src &&\
    cd /home/${USERNAME}/catkin_ws/src &&\
    cd /home/${USERNAME}/catkin_ws &&\
    catkin_make'

#RUN '
#    && rm -rf /var/lib/apt/lists/*'



USER root

WORKDIR /home/${USERNAME}

#ENV DISPLAY=:0.0 \
#    DISPLAY_WIDTH=1280 \
#    DISPLAY_HEIGHT=850 
#EXPOSE 8080

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]