#!/bin/bash

function wait_port () {
  nc -z 127.0.0.1 $1
  while [ $? -ne 0 ]; do
    nc -z 127.0.0.1 $1
    sleep 1
  done
}

function wait_node () {
  rosnode ping $1 | grep reply
  while [ $? -ne 0 ]; do
    rosnode ping -c 1 $1 | grep reply
    sleep 1
  done
}

# $HOME/run_all.sh ${CARLA_RENDER_OPTS} > /dev/null &
$HOME/run_carla.sh ${CARLA_RENDER_OPTS} > /dev/null &

echo "WAITING FOR CARLA..."
wait_port $CARLA_PORT
echo "CARLA IS READY!"

echo "WAITING FOR CARLA_ROS_BRIDGE..."
wait_node "carla_ros_bridge"
echo "CARLA_ROS_BRIDGE IS READY!"

echo "WAITING FOR ROS MASTER..."
wait_port $ROS_MASTER_PORT || echo "ROS_MASTER FAILED!"
wait_node "rosout"
echo "ROS MASTER IS READY!"

echo "WAITING FOR RVIZ..."
wait_node "rviz"
echo "RVIZ IS READY!"

echo "LAUNCHING CAR..."
$HOME/run_etdv_car.sh ${CARLA_RENDER_OPTS} > /dev/null &

echo "WAITING FOR VEHICLE..."
wait_node "carla_manual_control_ego_vehicle"
echo "VEHICLE IS READY!"

sleep 15

$HOME/run_etdv_track.sh ${CARLA_RENDER_OPTS} > /dev/null &

sleep 10
cd $PLANNING_WS/src
rosrun planning ROS_Stp.py > /dev/null &
sleep 1
rosrun planning Main_LTP.py > /dev/null &
sleep 1
rosrun execution dispatcher.py &
sleep 1
