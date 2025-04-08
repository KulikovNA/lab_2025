#!/bin/bash

xhost +local:

docker run -it --rm \
    --network host \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    --privileged \
    base_docker_lab2