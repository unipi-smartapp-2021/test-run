# actuators-control

This repository holds the ROS package of the execution system.
The package, for now, only contains a toy node that continously publishes a constant over a topic.

## Installation
Make sure you have *docker* installed. Then to build and run the docker container: </br>
`chmod +x ./run_ros.sh && ./run_ros.sh` </br>

## Execution
Once the container is running, to launch an interactive shell inside it run: </br>
`docker exec -it ros /bin/bash` </br>
You can then launch a toy ROS node by typing in the terminal: `rosrun execution dispatcher.py`.

## Implementing new Nodes
To implement new nodes as python script, touch a file under the **src/execution/scripts** directory.
Then add the node to the CMakeLists.txt under the `catkin_install_python` section.
