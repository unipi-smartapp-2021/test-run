#!/bin/bash

$HOME/run_all.sh ${CARLA_RENDER_OPTS} > /dev/null &
sleep 30

cd $PLANNING_WS/src
rosrun planning ROS_Stp.py > /dev/null &
sleep 5
rosrun planning Main_LTP.py > /dev/null &
sleep 5
rosrun execution dispatcher.py &
sleep 5
