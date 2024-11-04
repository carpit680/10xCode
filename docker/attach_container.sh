#!/bin/bash
SHELL_DIR=$(cd $(dirname $0) && pwd)

cd $SHELL_DIR/dockerfiles

# Set a prefix for the image name
NAME_PREFIX='ros2-docker-'

# Retrieve the latest commit ID of the Git repository
LATEST_COMMIT_ID=$(git -C "$SHELL_DIR" rev-parse --short HEAD)

# Assign the image name dynamically based on the commit ID
NAME_IMAGE="${NAME_PREFIX}${LATEST_COMMIT_ID}"

if [ "$(docker ps -al | grep "${NAME_PREFIX}")" ]; then
    ./launch_container.sh
else
    echo "Please start container first!"
fi
