#!/bin/bash

SHELL_DIR=$(cd $(dirname $0) && pwd)

cd $SHELL_DIR/dockerfiles
./launch_container.sh commit
