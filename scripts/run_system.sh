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

function spawn_sensors () {
  rosrun sensory rgb_camera.py > /dev/null &
  sleep 10
  rosrun sensory lidar.py > /dev/null &
  sleep 20
  rosrun sensory output_fusion.py > /dev/null &
}

function spawn_slam () {
  rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node > /dev/null &
  sleep 10
  rosrun cone_mapping cone_mapping.py > /dev/null &
  sleep 10
  rosparam set use_sim_time true
  rosrun laser_scan_matcher laser_scan_matcher_node > /dev/null &
}

function signal_start () {
  rosrun sensory human_interaction.py start
}

function spawn_planning () {
  echo "LAUNCHING CAR..."
  $HOME/run_etdv_car.sh ${CARLA_RENDER_OPTS} > /dev/null &

  echo "WAITING FOR VEHICLE..."
  wait_node "carla_manual_control_ego_vehicle"
  echo "VEHICLE IS READY!"
  sleep 10

  $HOME/run_etdv_track.sh ${CARLA_RENDER_OPTS} > /dev/null &
  sleep 30
}

function spawn_track () {
  cd $PLANNING_WS/src
  rosrun planning ROS_Stp.py > /dev/null &
  sleep 5
  rosrun planning Main_LTP.py > /dev/null &
  sleep 5
  rosrun execution dispatcher.py &
}

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

spawn_track
sleep 20
spawn_sensors
sleep 20
spawn_slam
sleep 20
spawn_planning
sleep 20
signal_start
