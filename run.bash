#!/bin/bash

xhost +
docker run -it --rm \
    --gpus all \
    --network=host \
    --ipc=host \
    --device=/dev/ttyACM0 \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name dtc-ros-jazzy-cuda \
    ros-jazzy:cuda \
    bash
xhost -