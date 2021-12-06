#!/bin/sh


USERNAME=noetic
BUILDARGS=""

options=$(getopt --longoptions right,left -n 'run_amd' --options '' -- "$@")
[ $? -eq 0 ] || { 
  echo "Usage: run_amd [--right | --left]"
  exit 1
}
eval set -- "$options"
while true; do
  case "$1" in
    --right ) BUILDARGS="$BUILDARGS --build-arg RIGHT_SIDE=1"; shift ;;
    --left ) BUILDARGS="$BUILDARGS --build-arg LEFT_SIDE=1"; shift ;;
    -- ) shift; break ;;
    * ) break ;;
  esac
done

# remove existing running ros container
docker stop ros 
docker rm ros

# build and run new ros container
docker build --rm $BUILDARGS -t ros . &&

# the -opengl is automatically discarted by CARLA 0.9.12+
RENDER_OPTS="-opengl -quality-level=Low -RenderOffScreen"
# This corresponds to the username you wish to opt-in with (bad naming, i know)
ROS_VERSION=noetic
# Enable X11 forwarding on the host
xhost +
# Run docker container
docker run -e DISPLAY=$DISPLAY \
  -e CARLA_RENDER_OPTS="$RENDER_OPTS" \
  --privileged --rm -it \
  --net=host \
  --gpus all \
  --runtime=nvidia \
  --env=TERM=xterm-256color \
  -u "$ROS_VERSION" \
  --name ros \
  ros \
  $@


