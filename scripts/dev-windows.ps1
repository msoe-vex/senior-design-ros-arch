# This script pulls down the docker contrainer for the dev environment, and starts it up

$CONTAINER_IMAGE="raiderrobotics/container-registry:rr-foxy-base"
$CONTAINER_NAME="rr-dev-container"
$SOURCE_DIR=($PSScriptRoot -replace "[\\/]scripts", "")
$DEST_DIR="/root/code"
$MOUNT_CMD=-join(" -v ", $SOURCE_DIR, ":", $DEST_DIR, " ", $CONTAINER_IMAGE)
$STARTUP_CMD=-join("docker run -it -d --name ", $CONTAINER_NAME, $MOUNT_CMD)


Invoke-Expression -Command "docker pull $CONTAINER_IMAGE"

if (-not ($?)) {
    Write-Output "Docker is not running. Please start/restart Docker and run again."
}
   
if (-not (docker ps -q -f name=$CONTAINER_NAME)) {
    if (docker ps -aq -f status=exited -f name=$CONTAINER_NAME) {
        Invoke-Expression "docker rm $CONTAINER_NAME"
    }

    Write-Output "Mounting directory '$SOURCE_DIR' to '$DEST_DIR'"
    Invoke-Expression $STARTUP_CMD
}
