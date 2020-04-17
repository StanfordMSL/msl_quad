# FROM bramtoula/ros:melodic-trt-py3-tx2-jp430
FROM mslquad_base:latest
WORKDIR /root

# Change default shell for RUN per https://stackoverflow.com/questions/20635472/using-the-run-instruction-in-a-dockerfile-with-source-does-not-work/39777387#39777387
SHELL ["/bin/bash", "-c"]

# Clone OCAM github (and install dependences) --> this allows us to use our camera
RUN apt-get update && \
    apt-get install -y libudev-dev && apt-get install -y libv4l-dev && \
    cd ~/catkin_ws/src && \
    git clone https://github.com/StanfordMSL/ocam_msl.git && \
    rm -rf /var/lib/apt/lists/*

RUN source /ros_entrypoint.sh && \
    cd ~/catkin_ws && \
    catkin build

# Set Env Variables for NVIDIA (or else rviz wont work) - https://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY ./mslquad_base_ocam_entrypoint.sh /

ENTRYPOINT ["/mslquad_base_ocam_entrypoint.sh"]
CMD ["bash"]
