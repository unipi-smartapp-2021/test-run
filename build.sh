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

# remove existing running ros container
docker stop kerubless
docker rm kerubless

docker build \
  --rm \
  $BUILDARGS \
  --build-arg CARLA_VERSION=$CARLA_VERSION \
  --build-arg ARCH=$ARCH \
  --build-arg GAPI=$GAPI \
  --build-arg ETEAM_ASSETS=$LOAD_ASSETS \
  -t kerubless \
  .
