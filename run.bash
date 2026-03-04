#!/bin/bash

xhost +
docker run -it --rm \
    --gpus all \
    --network=host \
    --ipc=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./src/diff-gps:/home/dtc/ws/src/diff_gps/" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name dtc-dgps-dev \
    dgps:dev \
    bash
xhost -
