# actuators-control

This repository holds the ROS package of the execution system.
The module is built on top of [carla-ros](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/tree/dockerized-install)
as a docker container.
It holds a reference to the planning module and a script to execute the Planning-Execution Driverless subsystem.

## Installation
Make sure you have *docker* installed. Then to build and run the docker container, run either:
```
run_nvidia.sh
```
or
```
run_amd.sh
```

## Execution
Once the container is running, to launch an interactive shell inside it run: </br>
```
docker exec -it ros /bin/bash
```
By default, when ran, the container spawns the CARLA simulator, the Dispatcher node of the
Execution module and the current configuration of STP and LTP nodes from the planning package.

To stop the container, run:
```
docker stop ros
```

## Implementing new Nodes
To implement new nodes as python script, touch a file under the **src/execution/scripts** directory.
Then add the node to the CMakeLists.txt under the `catkin_install_python` section.
