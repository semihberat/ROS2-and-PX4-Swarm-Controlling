# 🚁 PX4 ROS2 Offboard Swarm Control

A comprehensive, production-ready system for multi-UAV swarm coordination using **PX4 Autopilot**, **ROS2 (Humble)**, and **Gazebo**. The project is designed with a highly modular architecture, leveraging ROS2 Lifecycle Nodes for strict state management and robust swarm behavior.

---

## ✨ Key Features

- **Multi-Agent Swarm Coordination:** Supports coordinating up to 5 drones simultaneously with dynamic formation control.
- **Lifecycle Node Management:** Strict node control (Unconfigured → Inactive → Active → Finalized) ensuring safe initialization and termination.
- **Advanced Path Planning & Collision Avoidance:** Formulations based on the center of gravity of the swarm, including real-time distance checking and evasive maneuvers.
- **Modular & Performant Architecture:** Clean separation of concerns with isolated modules for communication, autonomy, flight control, and mathematical formulations.
- **Custom Swarm Communication:** Peer-to-peer GPS and status sharing across the swarm through customized ROS2 interfaces (`NeighborsInfo`).

---

## 📁 System Architecture

The codebase has been refactored for maximum modularity and maintainability.

```text
swarm_drone_control/
├── include/
│   └── calculations/         # Core mathematical libraries (Geographic conversions, PID, offset calculations)
├── src/
│   ├── autonomus/            # High-level mission planning and lifecycle state machine management
│   ├── communication/        # P2P swarm communication, broadcasting GPS and health status
│   ├── controller/           # Main swarm intelligence: center of gravity, formation, and collision avoidance
│   ├── uav_controller/       # Low-level flight controller interfacing directly with PX4 (Trajectory Setpoints)
│   ├── ground_control/       # Ground station integration and interaction nodes
│   └── utils/                # Utility scripts (altitude adjustments, formation rotation, leaving the herd)
├── launch/                   # ROS2 Launch files
│   └── multi_robot_start.py  # Main entry point to launch the entire swarm system
├── config/                   # YAML configuration files
│   └── multi_robot_params.yaml
└── start_multi_drones.sh     # Bash script to ignite PX4 SITL instances in Gazebo
```

---

## 🛠️ Module Breakdown

1. **`uav_controller` (Low-Level Flight Control):**
   Communicates directly with the PX4 Autopilot. Responsible for arming, validating offboard mode, and converting high-level target positions into low-level `TrajectorySetpoint` commands.

2. **`controller` (Swarm Intelligence):**
   The brain of the swarm. It acts as a Lifecycle Node calculating the swarm's Center of Gravity, assigning formation offsets to individual drones, and executing collision avoidance formulations when drones get too close.

3. **`autonomus` (Mission Management):**
   A Lifecycle Node responsible for handling autonomous flight stages (e.g., takeoff, navigation, landing). It dictates when the drone should transition between mission phases.

4. **`communication` (Social Network):**
   Manages the data flow between drones. Broadcasts the host drone's telemetry and listens to neighbor data to ensure the `controller` has accurate real-time information of the entire swarm.

5. **`calculations` (Math Library):**
   Provides highly optimized C++ templates for geographic transformations (Lat/Lon/Alt to Local NED), vector distances, and Proportional-Integral-Derivative (PID) control algorithms.

---

## 🚀 Quick Start Guide

### Prerequisites
- **Ubuntu 22.04**
- **ROS2 Humble**
- **PX4 Autopilot (v1.14+)**
- **Gazebo Garden**
- ROS2 Dependencies: `px4_msgs`, `lifecycle_msgs`, `rclcpp_lifecycle`

### 1. Build the Workspace

```bash
git clone https://github.com/semihberat/OffboardControl.git
cd OffboardControl
colcon build --packages-select swarm_drone_control custom_interfaces
source install/setup.bash
```

### 2. Launch the Simulation (Gazebo & PX4)
Open Terminal 1 and start the SITL instances:
```bash
cd OffboardControl
./start_multi_drones.sh
```

### 3. Start the ROS2 Swarm System
Open Terminal 2, source your workspace, and launch the controllers:
```bash
cd OffboardControl
source install/setup.bash
ros2 launch swarm_drone_control multi_robot_start.py
```

---

## 🎮 Interacting with Lifecycle Nodes

Because the standard architecture uses ROS2 Lifecycle nodes, systems start in an `Unconfigured` state. You can interact with them via the terminal:

```bash
# Check the state of a node
ros2 lifecycle get /path_planner_1

# Trigger transitions manually (if not handled by the autonomus module)
ros2 lifecycle set /path_planner_1 configure
ros2 lifecycle set /path_planner_1 activate
```

---

## 📡 Essential ROS2 Topics

Each drone operates in its own isolated namespace (e.g., `/px4_1/`, `/px4_2/`).

- **`/px4_1/fmu/in/trajectory_setpoint`**: Sending movement commands to PX4.
- **`/px4_1/fmu/out/vehicle_global_position`**: Reading current GPS telemetry from PX4.
- **`/px4_1/neighbors_info`**: Real-time swarm communication topic (Custom message type).
- **`/px4_1/target_positions`**: Calculated next goal for the UAV controller.

---

## 🤝 Contributing
1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License
This project is licensed under the **BSD 3-Clause License** - see the LICENSE file for details.
