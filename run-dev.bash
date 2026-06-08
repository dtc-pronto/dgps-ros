#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="${SCRIPT_DIR}/bags"

mkdir -p "${BAG_DIR}"

xhost +
docker run -it --rm \
    --network=host \
    --ipc=host \
    --privileged \
    -v "/dev:/dev" \
    -v "${BAG_DIR}:/home/dtc/bags" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name dtc-dgps-dev \
    dgps:dev \
    bash
xhost -
