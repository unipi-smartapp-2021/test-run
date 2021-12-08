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
    apt-get update && \
    rosdep update && \
    rosdep install -y planning
RUN echo "source $PLANNING_WS/devel/setup.bash" >> ~/.bashrc

COPY ./scripts/run_planner.sh $HOME/run_planner.sh

RUN sudo apt-get install -y netcat

# make execution workspace
ENV EXECUTION_WS $HOME/actuators_ws
RUN mkdir -p $EXECUTION_WS
COPY ./src $EXECUTION_WS/src

# build execution package
WORKDIR $EXECUTION_WS
RUN catkin_make
RUN echo "source $EXECUTION_WS/devel/setup.bash" >> ~/.bashrc

WORKDIR $HOME
ENV CARLA_PORT 2000
ENV ROS_MASTER_PORT 11311

COPY ./scripts/change_car_location.sh $HOME/change_car_location.sh

ARG RIGHT_SIDE=0
# Move car to the right if specified
RUN if [ $RIGHT_SIDE -ne 0 ]; then \
      ./change_car_location.sh '-199.0'; \
    fi

ARG LEFT_SIDE=0
# Move car to the left if specified
RUN if [ $LEFT_SIDE -ne 0 ]; then \
      ./change_car_location.sh '-195.4'; \
    fi

CMD ["/bin/bash", "-ic", "$HOME/run_planner.sh && tail -f /dev/null"]
