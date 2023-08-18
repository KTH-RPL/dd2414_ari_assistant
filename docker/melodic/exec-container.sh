# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

xhost +local:docker
docker exec $DOCKER_USER_ARGS --user user -it ari-dev-docker-melodic /bin/bash
