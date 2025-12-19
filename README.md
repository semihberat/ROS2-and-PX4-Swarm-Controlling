# 🚁 PX4 ROS2 Offboard Control

Advanced multi-vehicle offboard control system for PX4 autopilot using ROS2. Production-ready architecture with lifecycle node management, swarm coordination, and gamepad control interface.

[Screencast from 12-01-2025 08:38:23 AM.webm](https://github.com/user-attachments/assets/0f7b60f2-8eb0-4fc3-a765-8c1691f37ffc)

## ✨ Key Features

### 🎯 Core Capabilities
- **Multi-Vehicle Swarm**: Coordinate up to 5 drones with intelligent formation control
- **Lifecycle Node Architecture**: State-managed nodes (configure → activate → deactivate → cleanup)
- **Gamepad Control**: Real-time manual control and formation triggering via joystick
- **Swarm Communication**: Custom neighbor GPS sharing via `NeighborsInfo` interface
- **Automated Path Planning**: Center-of-gravity based formation with collision avoidance

### 🎮 Ground Control System
- **Joystick Integration**: Configure and activate formations with button presses
- **Formation Management**: Lifecycle-based coordination across multiple drones
- **Real-time State Control**: Async service calls for responsive state transitions

### 📷 Computer Vision
- **ArUco World Integration**: Specialized Gazebo environment for marker detection
- **Dual Camera Bridges**: Synchronized image + camera_info publishing
- **Lifecycle Camera Nodes**: State-managed OpenCV processing pipeline
- **Multi-Camera Support**: Independent camera streams per drone

### 🔧 Advanced Features
- **Dynamic Configuration**: Runtime parameter adjustment via ROS2 parameters
- **Namespace Isolation**: Clean topic organization with `/px4_{id}/` namespaces
- **Modular Architecture**: Separated concerns (controller, formulations, ground_control)
- **Collision Avoidance**: Real-time distance checking and evasive maneuvers

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
# Terminal 1: Start PX4 SITL instances
./start_multi_drones.sh

# Terminal 2: Launch main system
source install/setup.bash
ros2 launch px4_ros_com multi_robot_start.py

# Terminal 3 (Optional): Ground control interface
ros2 run joy joy_node  # Start joystick node
ros2 launch px4_ros_com ground_control.py  # Formation control
```

### Gamepad Control
```bash
# Connect your gamepad/joystick first
ros2 run joy joy_node

# Launch formation control system
ros2 launch px4_ros_com ground_control.py

# Press Button 0: Configure → Activate all path planners
```

## 📁 Project Structure

```
px4_ros_com/
├── src/
│   ├── controller/           # Core UAV control & swarm coordination
│   │   ├── uav_controller.cpp            # Main offboard controller
│   │   ├── swarm_member_path_planner.cpp # Lifecycle path planner
│   │   └── swarm_communication.cpp       # Neighbor GPS sharing
│   ├── ground_control/       # Manual control interfaces
│   │   ├── formation_control.cpp         # Lifecycle manager via joystick
│   │   └── gamepad_control.py            # Joystick listener
│   ├── formulations/         # Mathematical algorithms
│   │   ├── geographic/       # GPS calculations
│   │   │   ├── calculate_center_of_gravity.hpp
│   │   │   ├── calculate_offset_from_center.hpp
│   │   │   └── calculate_distance.hpp
│   │   └── path_planning/    # Path planning math (future)
│   ├── interfaces/           # Custom data structures
│   │   ├── vehicle_positions.hpp
│   │   └── vectoral_distance.hpp
│   ├── object_detection/     # Computer vision modules
│   │   └── camera_bridge.py  # Lifecycle camera node
│   └── lib/                  # Shared libraries
│       └── frame_transforms/ # Coordinate transformations
├── launch/                   # Launch configurations
│   ├── multi_robot_start.py  # Main system launcher
│   └── ground_control.py     # Formation control launcher
├── config/                   # Parameters
│   └── multi_robot_params.yaml
└── start_multi_drones.sh     # PX4 SITL startup script
```

### 🧮 **Formulations Module**
Mathematical algorithms and geometric calculations for swarm operations:
- **CalculateCenterofGravity.hpp**: Template-based center of gravity computation for GPS positions
- **CalculateOffsetsFromCenter.hpp**: Formation offset generation with GPS ↔ meter conversion
- **CalculateDistance.hpp**: Precise distance calculations with vectoral components
- **Performance optimized**: Pre-allocated vectors, efficient transformations
- **Collision avoidance math**: Real-time distance checking for safe operation

### 🛤️ **Path Planner Module** 
Intelligent navigation and coordination as lifecycle nodes:
- **Lifecycle state management**: Configure → Activate → Deactivate when target reached
- **Formation control**: Center-of-gravity based positioning with dynamic offset assignment
- **Real-time coordination**: NeighborsInfo subscription for live formation updates
- **Collision avoidance**: Distance-based evasive maneuvering
- **Timer-based publishing**: 100ms interval target position updates

## 📊 ROS2 Architecture

### 🔍 **Topic Discovery**
```bash
# List all active topics
ros2 topic list

# Filter PX4 topics
ros2 topic list | grep px4

# Monitor a specific topic
ros2 topic echo /px4_1/neighbors_info

# Check topic info
ros2 topic info /px4_1/fmu/out/vehicle_global_position
```

### 📡 **Topic Hierarchy**

#### **Per-Drone Topics** (`/px4_{1-5}/`)
```
/px4_1/fmu/in/                    # Command Inputs
├── offboard_control_mode         # Control mode
├── trajectory_setpoint           # Position/velocity targets
└── vehicle_command               # Arm, takeoff, land

/px4_1/fmu/out/                   # Data Outputs
├── vehicle_global_position       # GPS (lat/lon/alt)
├── vehicle_local_position        # NED coordinates
└── vehicle_status                # Flight state

/px4_1/neighbors_info             # Swarm Communication
└── custom_interfaces/msg/NeighborsInfo

/px4_1/target_positions           # Path Planner Output
└── custom_interfaces/msg/TargetPositions
```

#### **Camera Topics**
```
/world/aruco/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/
├── image                         # sensor_msgs/Image
└── camera_info                   # sensor_msgs/CameraInfo
```

#### **Control Topics**
```
/joy                              # Joystick input
└── sensor_msgs/msg/Joy

/path_planner_{1-5}/change_state  # Lifecycle services
└── lifecycle_msgs/srv/ChangeState
```

## 🎮 Lifecycle Node Management

### States and Transitions
```
Unconfigured → (configure) → Inactive → (activate) → Active
     ↑                           ↓                      ↓
     └────────── (cleanup) ──────┘    (deactivate) ────┘
```

### Manual Control
```bash
# Check lifecycle state
ros2 lifecycle get /path_planner_1

# Manual transitions
ros2 lifecycle set /path_planner_1 configure
ros2 lifecycle set /path_planner_1 activate
ros2 lifecycle set /path_planner_1 deactivate
ros2 lifecycle set /path_planner_1 cleanup

# List available states
ros2 lifecycle list /path_planner_1
```

### Gamepad-Triggered Control
```bash
# Start joystick (ensure gamepad is connected)
ros2 run joy joy_node

# Launch formation control system
ros2 launch px4_ros_com ground_control.py

# Button mapping:
# Button 0: Configure → Activate all path planners
```

## 🔧 System Components

### Core Nodes
| Node | Type | Purpose |
|------|------|---------|
| `drone{1-5}` | Standard | Main UAV offboard controller |
| `swarm_communication_{1-5}` | Standard | Neighbor GPS publisher |
| `path_planner_{1-5}` | Lifecycle | Formation path planning |
| `formation_control_{1-5}` | Standard | Lifecycle manager via joystick |
| `camera_bridge_{1-5}` | Standard | Image topic bridge |
| `camera_info_bridge_{1-5}` | Standard | Camera info bridge |
| `camera_process_{1-5}` | Lifecycle | OpenCV processing |

### Executables
```bash
# C++ executables
ros2 run px4_ros_com uav_controller
ros2 run px4_ros_com swarm_member_path_planner
ros2 run px4_ros_com swarm_communication
ros2 run px4_ros_com formation_control

# Python executables
ros2 run px4_ros_com gamepad_control.py
ros2 run px4_ros_com camera.py
```

## 🚦 Usage Workflow

### 1. System Startup
```bash
# Terminal 1: Launch PX4 SITL (Gazebo will open)
cd /path/to/PX4-Autopilot
./start_multi_drones.sh

# Wait for all instances to boot (watch for "Ready for takeoff")
```

### 2. ROS2 System Launch
```bash
# Terminal 2: Launch ROS2 nodes
cd ~/ws_offboard_control
source install/setup.bash
ros2 launch px4_ros_com multi_robot_start.py

# Verify nodes running
ros2 node list
```

### 3. Formation Control (Option A: Manual)
```bash
# Terminal 3: Manual lifecycle control
ros2 lifecycle set /path_planner_1 configure
ros2 lifecycle set /path_planner_2 configure
ros2 lifecycle set /path_planner_3 configure
ros2 lifecycle set /path_planner_4 configure
ros2 lifecycle set /path_planner_5 configure

# Activate formation
ros2 lifecycle set /path_planner_1 activate
ros2 lifecycle set /path_planner_2 activate
ros2 lifecycle set /path_planner_3 activate
ros2 lifecycle set /path_planner_4 activate
ros2 lifecycle set /path_planner_5 activate
```

### 4. Formation Control (Option B: Gamepad)
```bash
# Terminal 3: Start joystick node
ros2 run joy joy_node

# Terminal 4: Launch formation control
ros2 launch px4_ros_com ground_control.py

# Press Button 0 on gamepad to configure+activate all drones
```

### 5. Monitor System
```bash
# Watch drone positions
ros2 topic echo /px4_1/fmu/out/vehicle_global_position

# Monitor neighbor communication
ros2 topic echo /px4_1/neighbors_info

# Check path planner targets
ros2 topic echo /px4_1/target_positions

# View lifecycle states
ros2 lifecycle get /path_planner_1
```

### 6. Automatic Deactivation
The system automatically deactivates path planners when:
- Formation target reached (distance < 0.01m)
- After 50 verification cycles (5 seconds at 100ms intervals)

### 7. Shutdown
```bash
# Stop ROS2 nodes (Ctrl+C in each terminal)
# Stop PX4 instances
# Close Gazebo
```

## 🐛 Troubleshooting

### Common Issues

**Gamepad not detected**
```bash
# Check joystick device
ls /dev/input/js*

# Test with jstest
jstest /dev/input/js0

# Verify joy_node output
ros2 topic echo /joy
```

**Path planner not responding**
```bash
# Check lifecycle state
ros2 lifecycle get /path_planner_1

# Verify service availability
ros2 service list | grep change_state

# Test manual transition
ros2 lifecycle set /path_planner_1 configure
```

**Formation control deadlock**
- Ensure you're using async service calls (fixed in latest version)
- Avoid `spin_until_future_complete` inside callbacks

**Python node not executing**
```bash
# Check shebang line
head -1 /path/to/script.py  # Should be: #!/usr/bin/env python3

# Verify executable permission
ls -la install/px4_ros_com/lib/px4_ros_com/*.py

# Rebuild if needed
colcon build --packages-select px4_ros_com
```

## 📝 Custom Interfaces

### NeighborsInfo.msg
```
px4_msgs/VehicleGlobalPosition main_position
px4_msgs/VehicleGlobalPosition[] neighbor_positions
uint8 main_id
```

### TargetPositions.msg
```
float64 target_dlat
float64 target_dlon
```

## 🎯 Key Algorithms

### Center of Gravity Formation
1. Collect all drone GPS positions via `NeighborsInfo`
2. Calculate center of gravity (mean lat/lon)
3. Generate offset positions around center (configurable spacing)
4. Assign each drone to nearest offset position
5. Publish target deltas for controller

### Collision Avoidance
```cpp
for each neighbor:
    if distance < collision_tolerance:
        target_offset = -neighbor_vector + target_offset
```

### Lifecycle Auto-Deactivation
```cpp
if (distance_to_target < 0.01m && verification_count >= 50):
    trigger_deactivate_transition()
```

## 📚 Dependencies
- ROS2 Humble
- PX4-Autopilot (SITL)
- Gazebo Garden
- px4_msgs
- custom_interfaces
- rclcpp_lifecycle
- lifecycle_msgs
- sensor_msgs
- joy
- cv_bridge
- OpenCV

## 🤝 Contributing
Fork, branch, commit, push, PR. Standard GitHub workflow.

## 📄 License
BSD 3-Clause (see LICENSE)
```
/world/aruco/model/x500_mono_cam_down_{1-5}/link/camera_link/sensor/imager/
├── image                         # Camera image stream
└── camera_info                   # Camera calibration data
```

afdasdfasdfa
