# ROS 2 + PX4 Swarm Drone Control — Arayüz Referansı

## Topics

| Topik | Tip | Yön | Açıklama |
|-------|-----|-----|----------|
| `/drone_info` | `custom_interfaces/msg/DroneInfo` | Pub + Sub | Her drone kendi bilgisini yazar, tüm sürüyü buradan okur. GCS izlemesi için ana topik. |
| `/px4_{id}/fmu/out/vehicle_global_position` | `px4_msgs/msg/VehicleGlobalPosition` | Sub | Ham GPS (lat, lon, alt). DroneCore bu veriyi `/drone_info`'ya dönüştürür. |
| `/px4_{id}/fmu/out/vehicle_control_mode` | `px4_msgs/msg/VehicleControlMode` | Sub | Arm/disarm durumu. |
| `/px4_{id}/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | Sub | Kuaterniyon姿姿 → yaw açısına dönüştürülür. |
| `/px4_{id}/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Pub | Offboard mod sinyali (10 Hz+ zorunlu). Velocity modu aktif. |
| `/px4_{id}/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | Pub | Hız setpoint'i (NED). Position alanları NaN, velocity dolu. |
| `/px4_{id}/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | Pub | MAVLink komutları (arm, mod değişimi). |
| `/world/aruco/model/x500_mono_cam_down_{id}/link/camera_link/sensor/imager/image` | `sensor_msgs/msg/Image` | Sub | Gazebo kamera görüntüsü (ros_gz_bridge üzerinden). |

---

## Services

### `/uav_{id}/set_parameters` — `custom_interfaces/srv/SetParameters`

```
# Request
float32 id               # 0 = broadcast, 1..N = belirli drone
float32 takeoff_alt      # varsayılan: 5.0 m
float32 min_alt          # varsayılan: 1.0 m
float32 max_alt          # varsayılan: 200.0 m
float32 formation_radius # varsayılan: 5.0 m
float32 velocity         # varsayılan: 5.0 m/s
float32 yaw_vel          # varsayılan: 1.0 rad/s
float32 alt_vel          # varsayılan: 2.0 m/s
---
# Response
bool success
string message
```

---

## Actions

### `/uav_{id}/move_drone` — `custom_interfaces/action/MoveDrone`

```
# Goal
uint8 HOLD     = 0
uint8 TAKEOFF  = 1
uint8 FORMATION = 2
uint8 GRID     = 3
uint8 WAYPOINT = 4

uint8 command
GeoPoint[] geo_points
Formation formation
---
# Result
DroneInfo reached
bool success
string message
---
# Feedback
DroneInfo current
```

---

## Custom Interface Tipleri

### `DroneInfo.msg`
```
uint8 id
GeoPoint geo_point
float32 yaw
GeoVel geo_vel
```

### `GeoPoint.msg`
```
float64 lat
float64 lon
float32 alt
```

### `GeoVel.msg`
```
float32 vel
float32 yaw_vel
float32 alt_vel
```

### `Formation.msg`
```
uint8 LINE=0  uint8 TRIANGLE=1  uint8 V_SHAPE=2  uint8 SQUARE=3
uint8 formation_type
float32 space
```

### `NeighborsInfo.msg`
```
DroneInfo[] neighbors
```
