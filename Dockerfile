FROM ros:noetic
SHELL ["/bin/bash", "-c"]
ARG WORKSPACE=/home/ws
ENV WORKSPACE $WORKSPACE
RUN apt-get -y update && apt-get -y install vim
RUN mkdir -p $WORKSPACE
ADD src $WORKSPACE/src
WORKDIR $WORKSPACE 

RUN source /ros_entrypoint.sh && catkin_make
RUN source devel/setup.sh
RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc
RUN echo "source $WORKSPACE/devel/setup.bash" >> /root/.bashrc
CMD ["roscore"]
