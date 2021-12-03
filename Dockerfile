ARG CARLA_VERSION=0.9.13
FROM ghcr.io/unipi-smartapp-2021/carla-ros:noetic-carla${CARLA_VERSION}-amd-vulkan
USER $USERNAME
SHELL ["/bin/bash", "-ic"]

# make planning workspace
ENV PLANNING_WS $HOME/planning_ws
RUN mkdir -p $PLANNING_WS/src
COPY ./planning $PLANNING_WS/src/

# build planning package
WORKDIR $PLANNING_WS
RUN catkin_make && \
    source $PLANNING_WS/devel/setup.bash && \
    rosdep update && \
    rosdep install -y planning
RUN echo "source $PLANNING_WS/devel/setup.bash" >> ~/.bashrc

COPY ./scripts/run_planner.sh $HOME/run_planner.sh

# make execution workspace
ENV EXECUTION_WS $HOME/actuators_ws
RUN mkdir -p $EXECUTION_WS
COPY ./src $EXECUTION_WS/src

# build execution package
WORKDIR $EXECUTION_WS
RUN catkin_make
RUN echo "source $EXECUTION_WS/devel/setup.bash" >> ~/.bashrc

WORKDIR $HOME

CMD ["/bin/bash", "-ic", "$HOME/run_planner.sh && tail -f /dev/null"]
