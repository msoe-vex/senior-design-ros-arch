# This script pulls down the docker contrainer for the dev environment, and starts it up

CONTAINER_IMAGE="raiderrobotics/container-registry:rr-foxy-base"
CONTAINER_NAME="rr-dev-container"
SOURCE_DIR=${PWD%/*}
DEST_DIR="/root/code"
docker pull $CONTAINER_IMAGE

if [ $? -ne 0 ]; then
   echo "Docker is not running. Please start/restart Docker and run again."
fi

if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
        # cleanup
        docker rm $CONTAINER_NAME
    fi
    # run your container
    echo "Mounting directory '$SOURCE_DIR' to '$DEST_DIR'"
    docker run -it -d --name $CONTAINER_NAME --mount source="/c/Users/dupontn/Documents/raider-robotics/senior-design-ros-arch",target="$DEST_DIR" $CONTAINER_IMAGE bash
fi

read -n 1 -p "Press any key to continue..."