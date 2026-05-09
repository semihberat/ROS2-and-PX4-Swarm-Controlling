# Swarm Drone Control Workspace

Bu proje, PX4 otopilotu ve ROS 2 kullanılarak çoklu İHA'ların (drone) sürü halinde (swarm) offboard kontrolünü, görev yürütülmesini (kalkış, iniş, bekleme, rota takibi) ve otonom davranışlarını (çarpışma önleme vb.) yönetmek için geliştirilmiş bir sistemdir.

## Mimari ve Temel Bileşenler

*   `swarm_drone_control`: Droneların hareketlerini, iletişimini ve sürü mantığını (örn: çarpışma önleyici PID kontrolcüleri ve durum makineleri) yöneten ana paket. PX4 offboard modunu kullanarak `VehicleCommand`, `TrajectorySetpoint` yayınlar.
*   `drone_core`: Her bir drone'u temsil eden, sensör verilerini (`VehicleGlobalPosition`, `VehicleLocalPosition`, `VehicleAttitude`) dinleyen ve komutları ilgili PX4 arayüzlerine yollayan çekirdek asenkron ROS 2 düğümüdür (Node).
*   `custom_interfaces`: Dronelar ve diğer birimler arası iletişimi sağlamak için oluşturulan özel mesaj (`.msg`) ve servis (`.srv`) tanımlarını içerir. (`DroneInfo`, `GeoPoint`, `DroneCommands.srv` vb.)
*   `calculations` (Mesafe ve Kontrol): Drone'ların coğrafi hedeflerini metrik koordinatlara çeviren (Haversine formülü) algoritmalar (`geographic`) ve sürünün stabilizasyonu/çarpışma önlenmesi için P-I-D döngülerini (`PID`) barındırır.
*   `Gazebo / ROS 2 Bridge`: Çoklu araç simülasyonundaki kameraları ve metrikleri ROS 2 ortamına taşıyabilen (`camera_bridge.py`) bileşenlerdir.

## Kurulum (Workspace Setup)

Bu workspace, PX4 Autopilot ve ROS 2 (Humble/Iron/Jazzy vb.) yüklü bir sistem üzerinde çalışacak şekilde tasarlanmıştır.

1. **Gerekli bağımlılıkları yükleyin:**
   Sisteminizde ROS 2, `colcon` ve PX4 yapılandırmasının tam olduğuna emin olun. (Micro XRCE-DDS Agent gereklidir).
   ```bash
   sudo apt install python3-colcon-common-extensions ros-<ros2_distro>-ros-gz-bridge
   ```

2. **Workspace'i oluşturun ve derleyin:**
   ```bash
   cd ~/ws_offboard_control
   colcon build --symlink-install
   ```

3. **Workspace'i kaynaklayın (source):**
   Yeni bir terminal açtığınızda her zaman bu komutu çalıştırın:
   ```bash
   source install/setup.bash
   ```

## Kullanım

Sistemi başlatmak için hazırlanan fırlatma dosyaları (launch files) ve shell betikleri kullanılır.

Çoklu drone kurulumunu başlatmak için launch dosyasını çalıştırın:
```bash
ros2 launch swarm_drone_control multi_robot_start.py
```
*(Bu launch dosyası belirtilen sayıdaki drone düğümlerini ve kamera köprülerini eşzamanlı olarak ayağa kaldırır).*

### Servis Çağrıları (Terminalden Kullanım)

Dronelara görev atamak ve kontrol etmek için `custom_interfaces/srv/DroneCommands` ve `custom_interfaces/srv/SetParameters` servisleri kullanılır. Parametre ve komutları terminalden `ros2 service call` üzerinden yollayabilirsiniz.

**1. Drone'u Havalandırma (Takeoff):**
```bash
ros2 service call /drone_commands custom_interfaces/srv/DroneCommands "{command: 2}"
```
*(Not: `command: 2` DroneCommands.srv içinde TAKEOFF değerine karşılık gelmektedir).*

**2. Drone'u Konumunda Bekletme (Hold):**
```bash
ros2 service call /drone_commands custom_interfaces/srv/DroneCommands "{command: 1}"
```

**3. Drone'u İndirme (Land):**
```bash
ros2 service call /drone_commands custom_interfaces/srv/DroneCommands "{command: 3}"
```

**4. Rota (Waypoint) Takibi:**
Birden fazla coğrafi noktayı (enlem, boylam, irtifa) `geo_points` dizisi halinde göndererek rotayı takip etmesini sağlayabilirsiniz.
```bash
ros2 service call /drone_commands custom_interfaces/srv/DroneCommands "{command: 4, geo_points: [{lat: 41.1, lon: 29.0, alt: 10.0}, {lat: 41.101, lon: 29.001, alt: 10.0}]}"
```

**5. Uçuş Parametrelerini Değiştirme:**
Örneğin uçuş hızını veya formasyon yarıçapını terminal üzerinden dinamik değiştirmek için `SetParameters` kullanılır:
```bash
ros2 service call /set_parameters custom_interfaces/srv/SetParameters "{velocity: 8.0, takeoff_alt: 15.0}"
```