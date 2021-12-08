#!/bin/sh

CARLA_VERSION=0.9.13
ARCH=undefined
GAPI=vulkan
LOAD_RVIZ=1
BUILDARGS=""
CAR_X=''
CAR_Y=''
CAR_Z=''

options=$(getopt --longoptions nvidia,amd,opengl,vulkan,right,left,no-rviz,car-x:,car-y:,car-z: -n 'run' --options '' -- "$@")
[ $? -eq 0 ] || { 
  echo "Usage: run [--nvidia | --amd] [--opengl | --vulkan] [--right | --left] [--no-rviz] [--car-x=pos] [--car-y=pos] [--car-z=pos]"
  exit 1
}
eval set -- "$options"
while true; do
  case "$1" in
    --nvidia ) ARCH=nvidia; shift ;;
    --amd ) ARCH=amd; shift ;;
    --opengl ) GAPI=opengl; shift ;;
    --vulkan ) GAPI=vulkan; shift ;;
    --no-rviz ) LOAD_RVIZ=0; shift ;;
    --car-x ) shift; CAR_X=$1; shift ;;
    --car-y ) shift; CAR_Y=$1; shift ;;
    --car-z ) shift; CAR_Z=$1; shift ;;
    --right ) BUILDARGS="$BUILDARGS --build-arg RIGHT_SIDE=1"; shift ;;
    --left ) BUILDARGS="$BUILDARGS --build-arg LEFT_SIDE=1"; shift ;;
    -- ) shift; break ;;
    * ) break ;;
  esac
done

if [ $ARCH = "undefined" ]; then
  echo "Could not infer system's architecture"
  exit 1
fi

IMAGE=ghcr.io/unipi-smartapp-2021/carla-ros:noetic-carla${CARLA_VERSION}-${ARCH}-${GAPI}
# remove existing running ros container
docker stop kerubless
docker rm kerubless

docker build \
  --rm \
  $BUILDARGS \
  --build-arg CARLA_VERSION=$CARLA_VERSION \
  --build-arg GAPI=$GAPI \
  --build-arg ETEAM_ASSETS=$LOAD_ASSETS \
  --cache-from $IMAGE \
  -t $IMAGE \
  .

# the -opengl is automatically discarted by CARLA 0.9.12+
RENDER_OPTS="-opengl -quality-level=Low -RenderOffScreen"
# This corresponds to the username you wish to opt-in with (bad naming, i know)
ROS_VERSION=noetic
# Get the render group used by VA-API (for video acceleration)
RENDER_GROUP=$(grep -e 'render' /etc/group | cut -d ':' -f 3)

if [ -z "$RENDER_GROUP" ]; then
  echo "Rendering group not found: you might need to install libva for video acceleration"
  exit 1
fi

# Enable X11 forwarding on the host
xhost +
# Run docker container
# TODO: refactor this ugly conditional block
if [ $ARCH = "amd" ]; then
  docker run -e DISPLAY=$DISPLAY \
    -e CARLA_RENDER_OPTS="$RENDER_OPTS" \
    -e LOAD_RVIZ=$LOAD_RVIZ \
    -e KERUBLESS_SPAWN_X=$CAR_X \
    -e KERUBLESS_SPAWN_Y=$CAR_Y \
    -e KERUBLESS_SPAWN_Z=$CAR_Z \
    --privileged --rm -it \
    --net=host \
    --group-add=$RENDER_GROUP \
    --env=TERM=xterm-256color \
    -u "$ROS_VERSION" \
    --name kerubless \
    $IMAGE \
    $@

else
  docker run -e DISPLAY=$DISPLAY \
    -e CARLA_RENDER_OPTS="$RENDER_OPTS" \
    -e LOAD_RVIZ=$LOAD_RVIZ \
    -e KERUBLESS_SPAWN_X=$CAR_X \
    -e KERUBLESS_SPAWN_Y=$CAR_Y \
    -e KERUBLESS_SPAWN_Z=$CAR_Z \
    --privileged --rm -it \
    --net=host \
    --gpus all \
    --runtime=nvidia \
    --env=TERM=xterm-256color \
    -u "$ROS_VERSION" \
    --name kerubless \
    $IMAGE \
    $@
fi
