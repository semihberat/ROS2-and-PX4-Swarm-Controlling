# 🚁 PX4 ROS2 Offboard Control

Advanced multi-vehicle offboard control system for PX4 autopilot using ROS2. Production-ready architecture with lifecycle node management and swarm coordination.

[Screencast from 12-01-2025 08:38:23 AM.webm](https://github.com/user-attachments/assets/0f7b60f2-8eb0-4fc3-a765-8c1691f37ffc)

## ✨ Key Features

### 🎯 Core Capabilities
- **Multi-Vehicle Swarm**: Coordinate up to 5 drones with intelligent formation control
- **Lifecycle Node Architecture**: State-managed nodes (configure → activate → deactivate → cleanup)
- **Swarm Communication**: Custom neighbor GPS sharing via `NeighborsInfo` interface
- **Automated Path Planning**: Center-of-gravity based formation with collision avoidance

### 📷 Computer Vision
- **ArUco World Integration**: Specialized Gazebo environment for marker detection
- **Dual Camera Bridges**: Synchronized image + camera_info publishing (Lifecycle managed)
- **Multi-Camera Support**: Independent camera streams per drone

### 🔧 Advanced Features
- **Dynamic Configuration**: Runtime parameter adjustment via ROS2 parameters
- **Namespace Isolation**: Clean topic organization with `/px4_{id}/` namespaces
- **Modular Architecture**: Separated concerns (controller, formulations, interfaces)
- **Collision Avoidance**: Real-time distance checking and evasive maneuvers

## 🚀 Quick Start

### Build
```bash
git clone https://github.com/semihberat/OffboardControl.git
cd OffboardControl
colcon build --packages-select swarm_drone_control custom_interfaces
source install/setup.bash
```

### Launch System
```bash
# Terminal 1: Start PX4 SITL instances
./start_multi_drones.sh

# Terminal 2: Launch main system
source install/setup.bash
ros2 launch swarm_drone_control multi_robot_start.py
```

## 📁 Project Structure

```
swarm_drone_control/
├── src/
│   ├── controller/           # Core UAV control & swarm coordination
│   │   ├── uav_controller.cpp            # Main offboard controller (Setpoint publishing)
│   │   ├── swarm_member_path_planner.cpp # Lifecycle path planner (Goal generation)
│   │   └── swarm_communication.cpp       # Neighbor GPS sharing
│   ├── formulations/         # Mathematical algorithms
│   │   ├── geographic/       # GPS calculations
│   │   │   ├── calculate_center_of_gravity.hpp
│   │   │   ├── calculate_offset_from_center.hpp
│   │   │   └── calculate_distance.hpp
│   │   └── path_planning/    # Path planning math
│   ├── interfaces/           # Custom data structures
│   │   ├── vehicle_positions.hpp
│   │   └── vectoral_distance.hpp
│   ├── object_detection/     # Computer vision modules
│   │   └── camera_bridge.py  # Lifecycle camera node
│   └── lib/                  # Shared libraries
│       └── frame_transforms/ # Coordinate transformations
├── launch/                   # Launch configurations
│   └── multi_robot_start.py  # Main system launcher (Starts everything)
├── config/                   # Parameters
│   └── multi_robot_params.yaml
└── start_multi_drones.sh     # PX4 SITL startup script
```

## 🛠️ Detailed Code Analysis

### 1. UAV Controller (`uav_controller.cpp`)
This is the low-level flight control node. It talks directly to PX4.
- **Role**: Validates offboard mode and sends `TrajectorySetpoint`.
- **Key Logic**: Subscribes to `TargetPositions` (from Path Planner) and converts them into velocity/position commands for the drone.

### 2. Swarm Member Path Planner (`swarm_member_path_planner.cpp`)
**Architecture**: `rclcpp_lifecycle::LifecycleNode`
This node decides *where* the drone should go.
- **States**:
    - `Unconfigured`: Idle.
    - `Inactive`: Ready, parameters loaded.
    - `Active`: Computing and publishing targets.
    - `Finalized`: Safety state.
- **Logic**: Calculates the "Center of Gravity" of the swarm + a specific offset for this drone. Use collision avoidance formulations if too close to neighbors.

### 3. Swarm Communication (`swarm_communication.cpp`)
The "Social Network" of drones.
- **Role**: Broadcasts this drone's GPS position to others and listens for neighbors.
- **Topic**: `/px4_{id}/neighbors_info` (Custom Message).

### 4. Camera Bridge (`camera_bridge.py`)
Lifecycle-managed node that bridges Gazebo camera data to ROS2.
- **Role**: Publishes image and camera info.
- **Integration**: Works with ArUco marker detection environment.

## 🧮 Formulations Module
Mathematical algorithms and geometric calculations for swarm operations:
- **CalculateCenterofGravity.hpp**: Template-based center of gravity computation for GPS positions
- **CalculateOffsetsFromCenter.hpp**: Formation offset generation with GPS ↔ meter conversion
- **CalculateDistance.hpp**: Precise distance calculations with vectoral components
- **Performance optimized**: Pre-allocated vectors, efficient transformations

## 📊 ROS2 Architecture

### 🔍 **Topic Discovery**
```bash
# List all active topics
ros2 topic list | grep px4

# Monitor neighbors
ros2 topic echo /px4_1/neighbors_info
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
├── vehicle_status                # Flight state
├── vehicle_odometry              # Odometry data

/px4_1/neighbors_info             # Swarm Communication
└── custom_interfaces/msg/NeighborsInfo

/px4_1/target_positions           # Path Planner Output
└── custom_interfaces/msg/TargetPositions
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
```

## 📚 Dependencies
- **ROS2 Humble**
- **PX4-Autopilot (SITL)**
- **Gazebo Garden**: For simulation.
- **px4_msgs**: PX4 standard uORB messages.
- **custom_interfaces**: Our swarming message definitions.
- **rclcpp_lifecycle**: For managed nodes.
- **lifecycle_msgs**: For state transition services.
- **OpenCV & cv_bridge**: For camera processing.

## 🤝 Contributing
Fork, branch, commit, push, PR. Standard GitHub workflow.

## 📄 License
BSD 3-Clause (see LICENSE)
