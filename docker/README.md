# Containers for running Docker images for development on the ARI

This repository contains some convenience scripts for running containers with the SDK made available by PAL for the ARI.

Each kind of container, under each of the folders `melodic` and `noetic`, has the SDK for the corresponding ROS version.
As of now, the ROS version running on the robot is `noetic`, so it is recommended to use that one. The simulation
environment in either of the images is currently not working and we've contacted PAL about it. (might work now, this was written a few months ago).

To be able to build the images you need access to the gitlab docker registry. Contact Ermanno about it.

After having gotten access, login to the docker registry:

```
docker login registry.gitlab.com
```

Then, build the images:

```
sh build_images.sh
```

## Running the containers

In order to run the desired containers, you must first start one container with:
```
cd <distro>
./run-docker.sh
```

Dated, we don't use ARI wifi anymore: In case you want to have the ROS master be ARI, you can start the container *after joining the ARI wifi network* with:
```
cd <distro>
./run-docker.sh real
```
This automatically attempts to set the necessary `ROS_MASTER_URI` and `ROS_IP`. You can always do it manually in any running
container.


Then, you can start more terminals in this container by running:
```
cd <distro>
./exec-container.sh
```

If you run into any problems with the scripts, contact me (Zé, jmdap@kth.se)!

