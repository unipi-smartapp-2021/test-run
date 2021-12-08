# actuators-control

This repository holds the ROS package of the execution system and a reference to the planning module and a script
to execute the Planning-Execution Driverless subsystem.
The module is built on top of [carla-ros](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/tree/dockerized-install)
as a docker container, please refer to the aforementioned documentation if you have any problem with the setup.
The repository holds 

## Installation
Make sure you have *docker* installed. Then to build and run the docker container, run either:
```
./run.sh --nvidia --vulkan
```
or
```
./run.sh --amd --vulkan
```
depending on your hardware. Currently, opengl support is not available, so don't bother running the above command
with the `--opengl` flag.

Other available options are:
- `--no-rviz`: disable rviz launch
- `--car-x=value`: set x component of the car's spawn location
- `--car-y=value`: set y component of the car's spawn location
- `--car-z=value`: set z component of the car's spawn location
- `--left`: spawn car on left side of the lane
- `--right`: spawn car on the right side of the lane

## Execution
Once the container is running, to launch an interactive shell inside it run:
```
docker exec -it kerubless /bin/bash
```
By default, when ran, the container spawns the CARLA simulator, the Dispatcher node of the
Execution module and the current configuration of STP and LTP nodes from the planning package.

To stop the container, run:
```
docker stop kerubless
```

## Implementing new Nodes
To implement new nodes as python script, touch a file under the **src/execution/scripts** directory.
Then add the node to the CMakeLists.txt under the `catkin_install_python` section.
After that, you can safely rerun the container (which will cause docker to update the image).
