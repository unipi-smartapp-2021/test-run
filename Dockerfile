ARG CARLA_VERSION=0.9.11
FROM ghcr.io/unipi-smartapp-2021/carla-ros:noetic-carla${CARLA_VERSION}-nvidia-opengl
USER $USERNAME
ENV WORKSPACE $HOME/actuators_ws

RUN mkdir -p $WORKSPACE
COPY ./src $WORKSPACE/src
WORKDIR $WORKSPACE 

SHELL ["/bin/bash", "-ic"]
RUN catkin_make
RUN echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc

