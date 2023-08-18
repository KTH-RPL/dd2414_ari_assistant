if [ "$1" = "real" ]; then
	DOCKER_ROS_ARGS="-e ROS_MASTER_URI=http://192.168.128.28:11311 -e ROS_IP=`hostname -I | cut -d \" \" -f1`"
else
	DOCKER_ROS_ARGS=""
fi

../pal_docker.sh $DOCKER_ROS_ARGS -v ./test_ws:/home/user/test_ws --rm --name ari-simulation-dev-docker-melodic -it ari-simulation-dev-docker-melodic:latest bash
