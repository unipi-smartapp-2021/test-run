# TODO: add ARCH and GAPI build args
ARG CARLA_VERSION=0.9.13
ARG ARCH=nvidia
ARG GAPI=vulkan
FROM ghcr.io/unipi-smartapp-2021/carla-ros:noetic-carla${CARLA_VERSION}-${ARCH}-${GAPI}-cuda
USER $USERNAME
SHELL ["/bin/bash", "-ic"]

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

# make sensors workspace
ENV SENSORS_WS $HOME/sensors_ws
RUN mkdir -p $SENSORS_WS/src
# RUN sudo pip3 install --ignore-installed seaborn testresources opencv-python pandas numpy open3d Pillow
RUN sudo pip3 install torch==1.10.0+cu113 torchvision==0.11.1+cu113 torchaudio==0.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html 
# build sensors workspace
WORKDIR $SENSORS_WS

# ARG SENSORS_URL='https://mega.nz/file/W5kQyQoJ#1fUw4aVEO48Yec7tsK1ONYN8pE4sjMxL7IIMIVZnj20'
# ARG SENSORS_URL='https://mega.nz/file/Dt9hlQgb#mc8i95NwX115wvRFV1Z43zuHBwcZ9XoWXhjwoAn0gtE'
# ARG SENSORS_URL='https://mega.nz/file/CgMEEZYB#0AZAPnDxGYWGMhwFjQ08e1Evyv7pQFXEZ05g-Jvp144'
# ARG SENSORS_URL='https://mega.nz/file/bxVjQKxb#qP8iG55NnYnB_tljb_ji9xcRSoboRrkHLXISbFb7R5I'
# ARG SENSORS_URL='https://mega.nz/file/vktzwCQY#7EGiBNU8-CBy4as3CJ1mXHQl0uZNhzHJA1ROJryxGIM'
ARG SENSORS_URL='https://mega.nz/file/mw13zYxY#VDxfuSXjSwZMrGgCYHtTyygor6tPYUwkzo0rQe_QlWo'
# RUN mega-get $SENSORS_URL && \
#     unzip smartapp*.zip && \
#     rm smartapp*.zip && \
#     mv smartapp src/smartapp
RUN mega-get $SENSORS_URL && \
    unzip sensory.zip && \
    rm sensory.zip && \
    mv sensory src/smartapp

# RUN sudo pip3 install -r $SENSORS_WS/src/smartapp/requirements.txt
# uncomment this if pip3 is failing to install single packages
RUN cat $SENSORS_WS/src/smartapp/requirements.txt | xargs -n 1 sudo pip3 install || true
RUN sudo pip3 install open3d==0.13.0

# SENSORS DEPENDENCIES

# RUN sudo wget https://developer.download.nvidia.com/compute/cuda/11.3.0/local_installers/cuda_11.3.0_465.19.01_linux.run
# RUN sudo apt-get install nvidia-driver-470 -y
# RUN sudo sh cuda_11.3.0_465.19.01_linux.run --silent --toolkit --no-drm
# Set up the Conda environment
# ENV CONDA_AUTO_UPDATE_CONDA=false \
#     PATH=$HOME/miniconda/bin:$PATH
# COPY environment.yml ./environment.yml
# RUN curl -sLo $HOME/miniconda.sh https://repo.continuum.io/miniconda/Miniconda3-py39_4.10.3-Linux-x86_64.sh \
#  && chmod +x $HOME/miniconda.sh \ 
#  && $HOME/miniconda.sh -b -p $HOME/miniconda \
#  && rm $HOME/miniconda.sh \
#  && conda env update -n base -f ./environment.yml \
#  && rm ./environment.yml \
#  && conda clean -ya

RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make && \
    source $SENSORS_WS/devel/setup.bash
RUN echo "source $SENSORS_WS/devel/setup.bash" >> ~/.bashrc

CMD ["/bin/bash", "-ic", "$HOME/run_planner.sh && tail -f /dev/null"]
