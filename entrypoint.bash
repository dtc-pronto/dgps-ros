#!/bin/bash
# Source ROS 2
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/dtc/ws/install/setup.bash

# if [ "$DIFF_GPS" = true ]; then
if [ false ]; then
    echo "[DIFF-GPS] Launching diff_GPS and rtk reciever"
    ros2 launch diff_gps diff_gps.launch.py
else
    echo "[DIFF-GPS] RUN set to false, not launching diff-gps"
    exec "$@"  
fi
