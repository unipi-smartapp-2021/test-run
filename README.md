# Integrated Driverless System
This repository holds the references to every submodule of the Driverless system, namely:
* [`KB-backbone`](https://github.com/unipi-smartapp-2021/KB-backbone)
* [`KB-logging`](https://github.com/unipi-smartapp-2021/KB-logging)
* [`SLAM`](https://github.com/unipi-smartapp-2021/SLAM)
* [`Planning`](https://github.com/unipi-smartapp-2021/planning)
* [`Execution`](https://github.com/unipi-smartapp-2021/actuators-control)

As of now, the `Sensory` module doesn not have a GitHub repository because of file system limitations. For this reason, the submodule is currently hosted on MEGA and (automatically) deployed on the fly.

The integrated system is built on top of [carla-ros](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/tree/dockerized-install)
as a docker container, please refer to the aforementioned docummentation for setup instructions.

## Hardware Requirements
As of now, the current system architecture has strict hardware requirements: to work, an Nvidia graphics card must be available with the latest drivers installed and it must have `CUDA` compatibility with at least version `11.3`. Make sure you have the CUDA runtime installed on your host machine and that it matches (at least) the version indicated above.

## Installation
If you havent, please clone the repository with:
```
git clone --recursive <url>
```
This will pull every submodule recursively from the repo. If you've already cloned the repository (withouth the `--recursive` flag) and only wish
to update the submodules, run:
```
git submodule update --init --recursive
```

## Running
Make sure you have *docker* installed. Then to build and run the docker container, run:
```
./run.sh --nvidia --vulkan
```
The above command will build the docker image by pulling `carla-ros` from the
[Container Registry](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/pkgs/container/carla-ros).
When done, the script will spawn the docker container and will launch the CARLA simulator along with the car and acceleration track.
For backward (and forward) compatibility, you need to indicate on which system you want the system to be deployed on, by specifying the ``--nvidia`` and ``--vulkan`` flags. Other flags, albeit compliant with the specification, won't work.

If you only want to log into the container with a shell, run:
```
./run.sh --nvidia --vulkan /bin/bash
```
From there you waill be able to inspect the container and launch other ros nodes.
**NOTE: you can only have one running instance of the container. Subsequent calls of run.sh will stop the container and spawn a new one.**
If you wish to connect to the container from a different shell, you can skip to Section **Execution**

Other available options for `run.sh` are:
- `--no-rviz`: disable rviz launch (*Not recommended, since needed by the Sensory nodes*)
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

To stop the container, run:
```
docker stop kerubless
```

## Updating Submodules
**TODO**
