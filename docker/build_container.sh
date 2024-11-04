#!/bin/bash
SHELL_DIR=$(cd $(dirname $0) && pwd)

cd $SHELL_DIR/dockerfiles

# Set a prefix for the image name
NAME_PREFIX='ros2-docker-'

# Retrieve the latest commit ID of the Git repository
LATEST_COMMIT_ID=$(git -C "$SHELL_DIR" rev-parse --short HEAD)

# Assign the image name dynamically based on the commit ID
NAME_IMAGE="${NAME_PREFIX}${LATEST_COMMIT_ID}"

# Check if the Docker image already exists
if [ "$(docker image ls -q ${NAME_IMAGE})" ]; then
	echo "Docker image is already built!"
	exit
fi

# Build the Docker image
./launch_container.sh build

echo "Built Container Image: ${NAME_IMAGE}"
