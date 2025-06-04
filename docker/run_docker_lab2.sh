#!/bin/bash

xhost +local:root

docker run -it --rm \
    --network host \
    --env "QT_X11_NO_MITSHM=1" \
    --env DISPLAY=${DISPLAY} \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --device /dev/dri:/dev/dri \
    --privileged \
    lab_2
