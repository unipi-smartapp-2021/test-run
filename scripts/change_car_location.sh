#!/bin/bash

LOCATION=-197.2

if [ $# -eq 1 ]; then
  LOCATION=$1
fi

sudo sed -i "s/\(\"spawn_point\" default=\"[^,]*,\)\([^,]*,\)/\1$LOCATION,/g" \
  $HOME/etdv_simulator_ws/src/etdv_simulator/launch/spawn_vehicle.launch
