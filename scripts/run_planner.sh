#!/bin/bash

$HOME/run_all.sh ${CARLA_RENDER_OPTS} &
sleep 60

cd $PLANNING_WS/src
rosrun planning ROS_Stp.py &
sleep 5
rosrun planning Main_LTP.py &
sleep 5
rosrun execution dispatcher.py &
sleep 5
