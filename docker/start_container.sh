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
	echo "docker container restarting..."
	CONTAINER_ID=$(docker ps -a -f name="${NAME_PREFIX}" --format "{{.ID}}")
	
	sudo rm -rf /tmp/.docker.xauth
	XAUTH=/tmp/.docker.xauth
	touch $XAUTH
	xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
	if [ ! -z "$xauth_list" ]; then
		echo $xauth_list | xauth -f $XAUTH nmerge -
	fi
	chmod a+r $XAUTH

	docker start $CONTAINER_ID
	exit
fi

nohup ./launch_container.sh xrdp > /tmp/nohup.out &
