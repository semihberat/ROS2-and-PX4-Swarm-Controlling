# 🚁 PX4 ROS2 Offboard Control

Multi-vehicle offboard control system for PX4 autopilot using ROS2. Supports autonomous flight, swarm operations, and real-time neighbor communication.

[Screencast from 12-01-2025 08:38:23 AM.webm](https://github.com/user-attachments/assets/0f7b60f2-8eb0-4fc3-a765-8c1691f37ffc)

<img width="1390" height="728" alt="image" src="https://github.com/user-attachments/assets/6d6a2008-5f07-434c-8165-e77cdc7fe04e" />

## ✨ Features
- **Multi-Vehicle Support**: Control up to 5 drones simultaneously
- **Swarm Communication**: Custom neighbor GPS sharing via `NeighborsInfo` interface
- **ArUco World Integration**: Specialized environment for computer vision applications
- **Advanced Camera System**: Dual camera bridges (image + camera_info) for full sensor_msgs support
- **Automated Launch**: One-click system startup with complete camera integration
- **Real-time Monitoring**: GPS and local position tracking
- **Computer Vision Ready**: Full OpenCV/ROS2 camera integration with calibration data
- **Dynamic Configuration**: Runtime parameter adjustment via ROS2 parameters
- **Namespace Support**: Clean topic organization with `/px4_{id}/` namespaces

## 🚀 Quick Start

### Build
```bash
git clone https://github.com/semihberat/OffboardControl.git
cd OffboardControl
colcon build --packages-select px4_ros_com custom_interfaces
source install/setup.bash
```

### Launch System
```bash
./start_multi_drones.sh                      # Start PX4 instances in ArUco world
ros2 launch px4_ros_com multi_robot_start.py # Launch ROS2 nodes + Camera bridges
```

## 📁 Project Structure (project reorganized)

```
px4_ros_com/
├── src/
│   ├── controller/           # core controllers (uav_controller, offboard logic)
│   ├── formulations/         # math & algorithm helpers (CalculateCentralPoint, geometry utils)
│   ├── lib/                  # shared libraries (frame_transforms, helpers)
│   ├── object_detection/     # vision modules and scripts
│   └── path_planner/         # path planning algorithms and swarm coordination
├── launch/                   # launch files (multi_robot_start.py)
├── config/                   # configuration parameters (multi_robot_params.yaml)
└── start_multi_drones.sh     # PX4 SITL startup script
```

### 🧮 **Formulations Module**
The `formulations/` directory contains mathematical algorithms and geometric calculations essential for swarm operations:
- **CalculateCenterofGravity.hpp**: Compute center of gravity for GPS positions using template-based algorithms
- **CalculateOffsetsFromCenter.hpp**: Generate formation offsets from center point with GPS coordinate conversion
- **Mathematical utilities**: Precise GPS-to-meter conversions for formation control
- **Performance optimized**: Vector pre-allocation and efficient coordinate transformations
- **Future expansion**: Will include collision avoidance mathematics, formation patterns, and optimization algorithms

### 🛤️ **Path Planner Module** 
The `path_planner/` directory hosts intelligent navigation and coordination algorithms:
- **Swarm path planning**: Multi-drone trajectory generation and conflict resolution integrated with neighbor communication
- **Formation control**: Maintain desired geometric patterns using center of gravity calculations
- **Real-time coordination**: Integration with NeighborsInfo subscription for dynamic formation updates
- **Offset assignment**: Intelligent drone-to-position assignment for optimal formation flying
- **Integration ready**: Fully connected to controller via path_planner_callback() for seamless operation

Note: the package `main_class` executable is built from `src/controller/uav_controller.cpp` (see `CMakeLists.txt`).

## 📊 ROS2 Topic Architecture

### 🔍 **Topic Discovery**
```bash
# List all active topics
ros2 topic list

# Filter PX4 topics only
ros2 topic list | grep px4

# Show topic tree structure
ros2 topic list -t
```

### 📡 **Topic Hierarchy**

#### **Per-Drone Topics Structure** (`/px4_{1-5}/`)
```
/px4_1/fmu/in/                    # PX4 Command Input Topics
├── offboard_control_mode         # Control mode commands
├── trajectory_setpoint           # Position/velocity setpoints  
└── vehicle_command               # Vehicle control commands

/px4_1/fmu/out/                   # PX4 Data Output Topics
├── vehicle_global_position       # GPS coordinates
├── vehicle_local_position_v1     # Local NED position
└── vehicle_status_v1             # Flight status

/px4_1/neighbors_info             # Swarm Communication Topic
└── custom_interfaces/msg/NeighborsInfo  # Neighbor GPS sharing
```

#### **Camera Topics** (`/world/aruco/model/`)
```
/world/aruco/model/x500_mono_cam_down_{1-5}/link/camera_link/sensor/imager/
├── image                         # Camera image stream
└── camera_info                   # Camera calibration data
```

afdasdfasdfa
