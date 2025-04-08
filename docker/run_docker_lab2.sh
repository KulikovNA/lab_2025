#!/bin/bash

xhost +local:

docker run -it --rm \
    --env "QT_X11_NO_MITSHM=1" \
    --env DISPLAY=${DISPLAY} \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    --privileged \
    lab_2