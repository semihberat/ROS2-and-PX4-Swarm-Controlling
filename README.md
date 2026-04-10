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

---

## Kod İşleyişi

### DroneCore Node'u Ne Yapar?

Her drone için ayrı bir `DroneCore` node'u çalışır. `sys_id` parametresiyle 1'den numaralandırılır.

**Başlangıçta (constructor):**
1. `sys_id` parametresini okur.
2. Publisher, subscriber, action server ve service server'ları oluşturur.
3. 100 ms'de bir çalışan timer başlatır.

**Timer callback — her 100 ms:**
- İlk 10 tick'te PX4'e offboard mod + sıfır hız setpoint'i gönderir (PX4 offboard geçişi için zorunlu).
- 10. tick'te `VEHICLE_CMD_DO_SET_MODE` ile offboard moda geçiş komutu atar.
- Her tick'te kendi konumunu `/drone_info`'ya yayınlar.

**Veri akışı:**
```
PX4 FMU
  └─ vehicle_global_position  ──→ drone_info_.geo_point (lat/lon/alt)
  └─ vehicle_attitude          ──→ drone_info_.yaw (kuaterniyon → yaw)
  └─ vehicle_control_mode      ──→ is_armed_

DroneCore
  └─ /drone_info (pub)  ──→ kendi konumu sürüye + GCS'e
  └─ /drone_info (sub)  ──→ diğer dronların konumu swarm_info_[id-1]'e kaydedilir
```

**Action server (MoveDrone) işleyişi:**

Yeni goal gelince aktif goal **preempt** edilir (abort), yeni goal çalıştırılır.

| Komut | Davranış | Succeed Koşulu |
|-------|----------|----------------|
| `HOLD` | Sıfır setpoint | Anında |
| `TAKEOFF` | Arm et → P kontrol ile `takeoff_alt`'a çık | ±0.25 m yaklaşınca |
| `WAYPOINT` | Bearing hesapla → hız vektörü → sıradaki noktaya git | Tüm noktalar ±0.25 m'de geçilince |
| `GRID` | Köşe noktalardan tarama yolu üret → WAYPOINT gibi işle | Tüm tarama yolu bitti |

Her döngüde feedback olarak drone'un anlık `DroneInfo`'su gönderilir. Cancel isteği her zaman kabul edilir.

**Service (SetParameters):**  
`request.id == sys_id` ise parametreleri `ServiceParameters` struct'ına yazar. Action server bu struct'ı kullanır.

> ⚠️ `id = 0` broadcast olarak tasarlanmış ama koddaki `id != sys_id || id != 0` koşulu yanlış — broadcast çalışmıyor. Her drone'a ayrı ayrı çağrı atın.

---

## Harita Entegrasyonlu GCS İçin Gerekenler

### Okunacaklar

| Veri | Topik | QoS |
|------|-------|-----|
| Tüm sürünün konumu | `/drone_info` | `BEST_EFFORT`, depth 5 |
| Arm durumu | `/px4_{id}/fmu/out/vehicle_control_mode` | `BEST_EFFORT`, depth 5 |
| Kamera görüntüsü | `/world/aruco/model/x500_mono_cam_down_{id}/link/camera_link/sensor/imager/image` | default |

> `/drone_info` mesajında haritada kullanılacak alanlar:
> - `msg.id` → hangi drone marker'ı
> - `msg.geo_point.lat / .lon` → marker konumu
> - `msg.geo_point.alt` → irtifa
> - `msg.yaw` → drone'un baktığı yön (radyan, NED)
> - `msg.geo_vel.vel` → hız

### Yazılacaklar

| İşlem | Arayüz | Komut |
|-------|--------|-------|
| Kaldır | `/uav_{id}/move_drone` action | `TAKEOFF` |
| Noktaya gönder | `/uav_{id}/move_drone` action | `WAYPOINT + geo_points` |
| Alan tarat | `/uav_{id}/move_drone` action | `GRID + köşe noktaları` |
| Beklet / durdur | `/uav_{id}/move_drone` action | `HOLD` |
| Hız / irtifa ayarla | `/uav_{id}/set_parameters` service | uçuş öncesi veya sırası |

### Dikkat Edilmesi Gerekenler

- **QoS:** PX4 ve `/drone_info` topikleri `BEST_EFFORT`. GCS subscriber aynısını kullanmalı, yoksa mesaj gelmez.
- **Preemption:** Action goal aktifken yenisi gönderilirse eski `ABORTED` döner. GCS bu durumu handle etmeli.
- **Arm zorunluluğu:** `WAYPOINT` ve `GRID` arm değilse anında `ABORTED` döner.
- **Offboard süreklilik:** PX4, setpoint 500 ms gelmezse modu düşürür. `DroneCore` bunu kendi timer'ıyla hallediyor, GCS PX4'e doğrudan yazmıyorsa sorun yok.
