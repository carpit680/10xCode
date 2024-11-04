#!/bin/bash
SHELL_DIR=$(cd $(dirname $0) && pwd)

cd ./scripts

NAME_IMAGE='ros2-docker-apple-silicon'

if [ "$(docker image ls -q ${NAME_IMAGE})" ]; then
	echo "Docker image is already built!"
	exit
fi

echo "Build Container"

./launch_container.sh build US

echo "_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/"
echo "_/Building container image is finished!!_/"
echo "_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/"
