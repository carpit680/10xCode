#!/bin/bash
SHELL_DIR=$(cd $(dirname $0) && pwd)

cd $SHELL_DIR/scripts

if [ "$(docker ps -al | grep ros2-docker-apple-silicon-docker)" ]; then
    ./launch_container.sh
else
    echo "Please start container first!"
fi