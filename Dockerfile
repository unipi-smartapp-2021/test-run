ARG CARLA_VERSION=0.9.11
FROM ghcr.io/unipi-smartapp-2021/carla-ros:noetic-carla${CARLA_VERSION}
SHELL ["/bin/bash", "-c"]
USER $USERNAME
ARG WORKSPACE=$HOME/actuators_ws
ENV WORKSPACE $WORKSPACE
ARG ROS_VERSION=noetic
ARG SIM_WS=$HOME/etdv_simulator_ws

RUN mkdir -p $WORKSPACE
ADD src $WORKSPACE/src
WORKDIR $WORKSPACE 

RUN source /opt/ros/$ROS_VERSION/setup.bash && \
    source $HOME/carla-ros-bridge/catkin_ws/devel/setup.bash && \
    source $SIM_WS/devel/setup.bash && \
    catkin_make
RUN echo "source $WORKSPACE/devel/setup.bash" >> ~/.bashrc
# CMD ["/bin/bash", "-ic", "roscore"]
CMD ["bash", "-ic", "$HOME/run_all.sh ${CARLA_RENDER_OPTS} && tail -f /dev/null"]

