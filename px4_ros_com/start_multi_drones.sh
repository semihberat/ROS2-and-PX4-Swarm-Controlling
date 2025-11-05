#!/bin/bash

cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=aruco PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 1 &

sleep 5

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=aruco PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 2 &

sleep 5

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=aruco PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,4" PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 3 &

sleep 5

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=aruco PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,6" PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 4 &

sleep 5

cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=aruco PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,8" PX4_SIM_MODEL=gz_x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 5 &

sleep 5

MicroXRCEAgent udp4 -p 8888