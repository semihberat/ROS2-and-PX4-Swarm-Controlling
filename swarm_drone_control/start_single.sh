#!/bin/bash

cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=aruco PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 1 &

sleep 5

MicroXRCEAgent udp4 -p 8888