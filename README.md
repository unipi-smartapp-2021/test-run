# Integrated Driverless System
This repository holds the references to every submodule of the Driverless system, namely:
* [`KB-backbone`](https://github.com/unipi-smartapp-2021/KB-backbone)
* [`KB-logging`](https://github.com/unipi-smartapp-2021/KB-logging)
* [`SLAM`](https://github.com/unipi-smartapp-2021/SLAM)
* [`Sensory`](https://github.com/unipi-smartapp-2021/sensory-cone-detection)
* [`Planning`](https://github.com/unipi-smartapp-2021/planning)
* [`Execution`](https://github.com/unipi-smartapp-2021/actuators-control)

The integrated system is built on top of [carla-ros](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/tree/dockerized-install)
as a docker container, please refer to the aforementioned docummentation for setup instructions. Make sure to
carefully follow Section [`Some dependencies`](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/tree/dockerized-install#some-dependencies) and have the `Nvidia container toolkit` installed on your system.


## System Requirements
As of now, the current system architecture has strict hardware requirements: to work, an Nvidia graphics card must be available with the latest drivers installed and it must have `CUDA` compatibility with at least version `11.3`. Make sure you have the CUDA runtime installed on your host machine and that it matches (at least) the version indicated above.

Having at least 12GB of RAM is recommended, although you can circumvent the lack of system memory by having a large swap file, provided it is installed on a fast drive.

## Installation
If you haven't, please clone the repository with:
```
git clone --recursive <url>
```
This will pull every submodule recursively from the repo. If you've already cloned the repository (withouth the `--recursive` flag) and only wish
to update the submodules, run:
```
git submodule update --init --recursive
```
It is recommended to also run the above command when checking out different remote branches.

## Running
Make sure you have *docker* installed. Then to build and run the docker container, run:
```
./run.sh --nvidia --vulkan
```
The above command will build the docker image by pulling `carla-ros` from the
[Container Registry](https://github.com/unipi-smartapp-2021/ETEAM-MIRROR-etdv_simulator/pkgs/container/carla-ros).
When done, the script will spawn the docker container and will launch the entire Driverless system, along with the CARLA simulator.
The manual `start` command will also be sent (for more information on timings and order of execution of nodes, please refer to `scripts/run_system.sh`).

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
To update single submodules to a different version, follow these steps:
It is **strongly** recommended to perform updates on a different branch and the open
a new Pull Request to review the changes through GitHub.

The following steps assume you're on a local branch other than `main`:
```
cd <submodule> 
git checkout <submodule_branch>
```
Then pull the branch updates and checkout the wanted commit hash or release tag.
You can also just pull and settle for the latest changes of your module, but using
a [Release Tag](https://docs.github.com/en/repositories/releasing-projects-on-github/managing-releases-in-a-repository)
is more manageable. Also make sure you indicate the new module version in your Pull Request.
```
git pull
git checkout <tag or commit>
```
You can then safely commit your changes and push them to your remote repository
```
cd .. 
git add <submodule>
git commit
git push --set-upstream origin <your_new_branch>
```

Lastly, open a Pull Request on GitHub on the `main` branch and have it reviewd by a mantainer of this repository.
