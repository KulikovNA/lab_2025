#!/bin/bash

xhost +local:

docker run --name lab2 \
    -it --rm \
    --env "QT_X11_NO_MITSHM=1" \
    --env DISPLAY=${DISPLAY} \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd)/../src:/workspace/lab_2025/src \
    --device /dev/dri:/dev/dri \
    --privileged \
    lab_2
