# FROM bramtoula/ros:melodic-trt-py3-tx2-jp430
FROM ros:melodic-perception
WORKDIR /root

# Change default shell for RUN per https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work/39777387#39777387
SHELL ["/bin/bash", "-c"]

### MAVROS ########################################
# https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html
# https://github.com/DiegoHerrera1890/Pixhawk-connected-to-Jetson-Tx2-devkit

# install tools needed for mavros install
RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    python-rosinstall-generator \
    wget \
    && rm -rf /var/lib/apt/lists/* 

# create catkin workspace for mavros
RUN source /ros_entrypoint.sh && \
    mkdir -p catkin_ws/src && \
    cd ~/catkin_ws && \
    catkin init && \
    wstool init src

# install mavlink
RUN cd ~/catkin_ws && \
    rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall && \
    wstool merge -t src /tmp/mavros.rosinstall && wstool update -t src -j4 && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/* 

# install other dependencies
RUN apt-get update && apt-get install -y \
    libgeographic-dev \
    geographiclib-tools \
    ros-melodic-tf2 \
    ros-melodic-tf2-eigen \
    ros-melodic-control-toolbox \
    && rm -rf /var/lib/apt/lists/* 

# Install required dataset
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod 754 install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh

# install mavros
RUN apt-get update && apt-get install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    && rm -rf /var/lib/apt/lists/* 

RUN source /ros_entrypoint.sh && \
    cd ~/catkin_ws && \
    catkin build
### END MAVROS ########################################


# Modify avahi-config (this has to be done)
RUN perl -p -i -e 's|#deny-interfaces=eth1|deny-interfaces=eth0,eth1|g' /etc/avahi/avahi-daemon.conf

COPY ./mslquad_base_entrypoint.sh /

ENTRYPOINT ["/mslquad_base_entrypoint.sh"]
CMD ["bash"]
