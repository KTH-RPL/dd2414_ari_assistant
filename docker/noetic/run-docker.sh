if [ "$1" = "real" ]; then
	DOCKER_ROS_ARGS="-e ROS_MASTER_URI=http://192.168.128.28:11311 -e ROS_IP=`hostname -I | cut -d \" \" -f1`"
else
	DOCKER_ROS_ARGS=""
fi

../pal_docker.sh $DOCKER_ROS_ARGS --rm --name ari-dev-docker-noetic -it ari-dev-docker-noetic:latest bash
