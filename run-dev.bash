#!/bin/bash

xhost +
docker run -it --rm \
    --gpus all \
    --network=host \
    --ipc=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./dgps-ros:/home/dtc/ws/src/dgps-ros/" \
    -v "./dgps-msgs:/home/dtc/ws/src/dgps-msgs/" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name dtc-dgps-dev \
    dgps:dev \
    bash
xhost -
