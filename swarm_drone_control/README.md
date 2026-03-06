# Swarm Drone Control - `autonomus` Modülü Dokümantasyonu

Bu repo ve paket (`swarm_drone_control`), çoklu İHA (İnsansız Hava Aracı) sistemleri için ROS 2 ve PX4 Framework tabanlı gelişmiş otonom görev hesaplama, pozisyon planlama ve sürü haberleşmesini koordine eden bir sistemdir.

## 📌 Genel Mimari ve `autonomus` Sınıfı

`SwarmMemberPathPlanner` sınıfı (Lifecycle Node), `autonomus.hpp` ve `autonomus.cpp` aracılığıyla her bir drone'un bireysel otonom uçuşunu idare eder. Her drone, bulunduğu sürünün ağırlık merkezine (Center of Gravity - CoG) göre formasyonda bir konum alır ve MAVLink/MAVROS aracılığıyla PX4 FCU'ya Setpoint Velocity (Hız Kontrol Verisi - `TrajectorySetpoint`) yollar.

Projedeki modüler kod şöyledir:
- **`autonomus.cpp` ve `autonomus.hpp`**: Ana sınıf yapısı, modülün kurulumu, Waypointlerin (Hedef yükseklik ve koordinat listesi) alınması.
- **`autonomus_lifecycle.cpp`**: ROS 2 Lifecycle yönetimi (`on_configure`, `on_activate`, `on_deactivate` vd.)
- **`autonomus_timers.cpp`**: Sistem döngüsü ve kontrol çağrıları (`state_cycle_callback` ve `collision_avoidance` döngüleri)
- **`autonomus_missions.cpp`**: Çeşitli Görevler (`formational_takeoff`, `formational_rotation`, `goto_position` vb.) içerisinde sürünün hedef rotasyona girmesi ve hareket hesaplamaları buradadır.
- **`autonomus_subscribers.cpp` / `autonomus_publishers.cpp`**: PX4 mesajlarına (Örn: `VehicleGlobalPosition`, `VehicleAttitude`) abone olunması ve PX4'e mesaj basılması işlemleri.
- **`autonomus_services.cpp`**: Client-Server ilişkisiyle dronların pozisyonuna ulaşıp ulaşmadığını kontrol eden `InTarget` servis uçları.

---

## 🛠️ Matematiksel Sistem ve Akış Şeması (Flow)

Dronlar sırasıyla `FORMATIONAL_TAKEOFF`, `FORMATIONAL_ROTATION` ve `GOTO_POSITION` adımlarından (enum state) geçer:

1. **`FORMATIONAL_TAKEOFF`**:
   Drone kendi `target_altitude_` irtifasına (Örneğin: -10 metre vb.) çıkar. İrtifa için z ekseninde `calculate_velocity() ` fonksiyonu ile hata (error) hesaplanıp PID tarzı (-z min_vel, max_vel arası) clamped velocity yollanır. Tüm sürü kalkış komutunu sağlayana dek beklenir.
2. **`FORMATIONAL_ROTATION`**:
   `autonomus_utils::calculate_target_bearing_for_drone` fonksiyonu aracılığıyla, sürünün **Ağırlık Merkezi (Center of Gravity - CoG)** hesaplanır. Bu eksende hedefe (Waypoint) bakan rotasyon formasyonu oluşturulur. Matematiksel olarak, `spatial::WrapAngleToPi(current_drone_bearing_to_cog + formation_rotation_angle)` hesaplanır. Diğer drone'larla mesafe (`geo::calculate_distance`) ölçümleri sonrası, PID bazlı dLon ve dLat ile teğetsel (`v_lat`, `v_lon`) uçuş hızları PX4'e basılır. 
3. **`GOTO_POSITION`**:
   Sürü hep beraber rotasyonlarını tamamladığında teker teker hedef `current()` waypoint koordinatına çapraz kayarak gitmeye başlarlar.

### Sürü Koordinat ve Yörünge Hesaplama (`autonomus_utils.hpp`)

Ana otonom hesaplar `autonomus_utils.hpp` yardımcısında (helper) barındırılır.

- `combine_positions`: Drone ve otonom komşuları (Neighbors Info) ortak bir diziye getirilir.
- `find_nearest_vehicle_to_target`: Listedeki hedefe en yakın (Nearest) dronu bulur ve mesafeyi optimize eder.
- `calculate_velocity`: Temel PID formülüyle hızı hedeften çıkartıp (error * P_GAIN) sınırlandırıp otonom koda iletir.

---

## 🛡️ Bellek Koruması ve WaypointManager 

Sistemin hedeflere gitmesi ("GOTO_POSITION" görevi), kod çökmesine (Segfault) karşı **`autonomus_utils::WaypointManager`** yardımıyla sıkı denetime alınmıştır. 
Bağlı liste (Linked List) mantığında çalışır. O anki hedefe `waypoint_manager_.current()` pointer'i ile güvenle erişim sağlanırken, hedefe varıldığında `waypoint_manager_.next()` çağrısı yapılarak bir sonraki hedefe geçiş güvenle yapılır. Dizi dışına çıkıldığında sistem `nullptr` dönerek arıza emniyetine geçer (Failsafe stop).

---

## 🚀 Kullanım Kılavuzu

Sistemi çalıştırmak için terminalden `swarm_drone_control` paketi çağrılır.
- Dronun hedef alacağı **Altitude (İrtifa)** statik değerden çıkarılıp `target_altitude_` değişkenine aktarılmıştır. Kod compile edildikten sonra (Örn: `colcon build`) hedef pozisyon node içindeki varsayılan değere ek olarak başlangıç veya görev değişimi ile sisteme sunulabilir.

Sistem kendi Lifecycle süreci üzerinden `Unconfigured -> Inactive -> Active` basamaklarından otonom olarak `change_state` (Eğer harici tetikleme varsa) servisiyle ilerleyerek kalkış emrini otomatik basacaktır.
