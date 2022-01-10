ARG CARLA_VERSION=0.9.13
ARG ARCH=nvidia
ARG GAPI=vulkan
FROM ghcr.io/unipi-smartapp-2021/carla-ros:noetic-carla${CARLA_VERSION}-${ARCH}-${GAPI}-cuda
USER $USERNAME
SHELL ["/bin/bash", "-ic"]

RUN sudo apt-get update && sudo apt-get install -y netcat

# install sensors dependencies
ENV PATH=$PATH:$HOME/.local/bin
RUN python3 -m pip install \
    torch==1.10.0+cu113 \
    torchvision==0.11.1+cu113 \
    torchaudio==0.10.0+cu113 \
    -f https://download.pytorch.org/whl/cu113/torch_stable.html 

COPY ./sensory-cone-detection/requirements.txt $SENSORS_WS/src/sensory/requirements.txt
RUN cat $SENSORS_WS/src/sensory/requirements.txt | xargs -n 1 python3 -m pip install || true

# make sensors workspace
ENV SENSORS_WS $HOME/sensors_ws
RUN mkdir -p $SENSORS_WS/src
COPY ./sensory-cone-detection/sensory $SENSORS_WS/src/sensory

WORKDIR $SENSORS_WS

# build sensors workspace
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make && \
    source $SENSORS_WS/devel/setup.bash && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
RUN echo "source $SENSORS_WS/devel/setup.bash" >> ~/.bashrc


# make slam workspace
ENV SLAM_WS $HOME/slam_ws
RUN mkdir -p $SLAM_WS/src
COPY ./SLAM/ $SLAM_WS/

# build slam workspace
WORKDIR $SLAM_WS
RUN sudo cp utils/pointcloud_to_laserscan_nodelet.cpp src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_nodelet.cpp && \
    sudo cp utils/sample_node.launch src/pointcloud_to_laserscan/launch/sample_node.launch && \
    sudo cp utils/laser_scan_matcher.cpp src/scan_tools/laser_scan_matcher/src/laser_scan_matcher.cpp

RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make && \
    source $SLAM_WS/devel/setup.bash
RUN echo "source $SLAM_WS/devel/setup.bash" >> ~/.bashrc

# make execution workspace
ENV EXECUTION_WS $HOME/actuators_ws
RUN mkdir -p $EXECUTION_WS
COPY ./actuators-control/src $EXECUTION_WS/src

# build execution package
WORKDIR $EXECUTION_WS
RUN catkin_make
RUN echo "source $EXECUTION_WS/devel/setup.bash" >> ~/.bashrc

# make planning workspace
ENV PLANNING_WS $HOME/planning_ws
RUN mkdir -p $PLANNING_WS/src
COPY ./planning $PLANNING_WS/src/

# build planning package
WORKDIR $PLANNING_WS
RUN catkin_make && \
    source $PLANNING_WS/devel/setup.bash && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install -y planning
RUN echo "source $PLANNING_WS/devel/setup.bash" >> ~/.bashrc

COPY ./scripts/run_planner.sh $HOME/run_planner.sh

WORKDIR $HOME
ENV CARLA_PORT 2000
ENV ROS_MASTER_PORT 11311

COPY ./scripts/change_car_location.sh $HOME/change_car_location.sh
COPY ./scripts/run_system.sh $HOME/run_system.sh

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
COPY ./resources/ellipse.yaml /home/noetic/etdv_simulator_ws/src/etdv_simulator/resources/
COPY ./resources/empty_track.xodr /home/noetic/CARLA_0.9.13/CarlaUE4/Content/Carla/Maps/OpenDrive/empty.xodr
COPY ./resources/spawn_custom_track.launch /home/noetic/etdv_simulator_ws/src/etdv_simulator/launch/
COPY ./resources/spawn_custom_vehicle.launch /home/noetic/etdv_simulator_ws/src/etdv_simulator/launch

CMD ["/bin/bash", "-ic", "$HOME/run_system.sh && tail -f /dev/null"]
