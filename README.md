# Sürü İHA (Swarm Drone Control) Kapsamlı Sistem ve Algoritma Dokümantasyonu

Bu belge, **ROS 2** ve **PX4 Framework (MAVLink/MAVROS)** üzerine inşa edilmiş `swarm_drone_control` paketinin detaylı uçuş mekaniklerini, matematiksel modellerini, kod organizasyonunu ve algoritmik yapısını içermektedir. Proje, çoklu İHA sistemlerinin formasyon halinde hareket etmesi, birbirleriyle otonom şekilde organize olması ve çarpışma/enkaz sorunlarından kaçınmasını hedeflemektedir.

---

## İçindekiler
1. [Sisteme Genel Bakış](#1-sisteme-genel-bakış)
2. [Sistem Mimarisi ve Dizin Yapısı](#2-sistem-mimarisi-ve-dizin-yapısı)
3. [Matematiksel Teori ve Formasyon Algoritmaları](#3-matematiksel-teori-ve-formasyon-algoritmaları)
    - 3.1. WGS84 Coğrafi Koordinat Sistemi ve Mesafe İşlemleri
    - 3.2. Sürünün Ağırlık Merkezi (Center of Gravity - CoG)
    - 3.3. Doğrultu (Bearing) Çıkarımı
    - 3.4. Formasyon Rotasyonu Algoritması (Formation Rotation)
    - 3.5. Oransal Hız Kontrolü (P-Controller Velocity Clamping)
4. [Bellek Güvenliği ve `WaypointManager` Mimarisi](#4-bellek-güvenliği-ve-waypointmanager-mimarisi)
5. `SwarmMemberPathPlanner` State Machine Akışı
    - 5.1. FORMATIONAL_TAKEOFF
    - 5.2. FORMATIONAL_ROTATION
    - 5.3. GOTO_POSITION
6. [Kapsamlı Kullanım Kılavuzu (Usage Guide)](#6-kapsamlı-kullanım-kılavuzu-usage-guide)
7. [API ve Fonksiyon Referansları](#7-api-ve-fonksiyon-referansları)

---

## 1. Sisteme Genel Bakış

Çoklu İHA (Swarm UAV) sistemlerinde temel problem, her bir birimin asenkron karar verirken senkronize bir bütün (sürü) gibi hareket edebilmesidir. Bu paket, merkezi bir uçuş bilgisayarından ziyade, her İHA'nın kendi **ROS 2 Lifecycle Node**'u üzerinden çevresindeki birimlerin (`NeighborsInfo`) koordinatlarını alıp kendi rotasını otonom olarak çizdiği dağıtık bir (distributed) mimari sunar.

**Temel Yetenekler:**
- **Merkeziyetsiz Planlama:** Her drone kendi vektörel hesaplamasını gerçekleştirir.
- **Hafıza Güvenliği:** Doğrusal hedeflerde `Segmentation Fault` gibi çökmeleri engelleyen özel *Linked List* varyantı veri yapıları kullanır.
- **Dinamik Coğrafi Rotasyon:** Sürü, bir hedefe yöneldiğinde formasyonunun geometrisini bozmadan kütle merkezinden (CoG) o hedefe doğru komple bir dönüş sağlar.

---

## 2. Sistem Mimarisi ve Dizin Yapısı

Modüler refactoring süreci sonrası `swarm_drone_control` paketindeki dizin sistemi ve görevleri tamamen izole edilmiştir.

* `src/autonomus/`: Otonom görev durumlarını (Takeoff, Rotation, GoTo) yürüten ve Lifecycle üzerine oturan ana beyin kısmı.
* `include/calculations/`: Haversine formülleri, bearing matematiği (`geographic.hpp`) ve uzamsal Euler operasyonlarını (`spatial.hpp`) barındıran kütüphane.
* `src/communication/`: Sürüdeki diğer dronların telemetri/durum paylaşımlarının sağlandığı iç-iletişim node'ları.
* `autonomus_utils.hpp`: Matematiksel verileri anlamlı sistem listelerine dönüştüren ve güvenliğini sağlayan fonksiyon blokları.

Her `SwarmMemberPathPlanner` (Lifecycle) birimi;
1. PX4'ten kendi durumunu (`VehicleGlobalPosition`, `VehicleAttitude`) çeker.
2. Formasyondaki diğer dronların durumunu Subscripe eder (`NeighborsInfo`).
3. PX4'e Hedef Hız Vektörünü (`TrajectorySetpoint`) basar.

---

## 3. Matematiksel Teori ve Formasyon Algoritmaları

Uçuşun pürüzsüz çalışması büyük oranda `calculations/` kütüphanesindeki WGS84 küresel geometri modellerine ve trigonometrik hesaplamalara bağlıdır.

### 3.1. WGS84 Coğrafi Koordinat Sistemi ve Mesafe İşlemleri

Dünya düz olmadığı için Descartes (X, Y) koordinat sistemlerini doğrudan kullanamayız. Projede Dünya'nın küresel geometrisi ele alınır (`Equatorial Radius, R = 6378137.0 m`). İki GPS noktası ($\phi_1, \lambda_1$) ve ($\phi_2, \lambda_2$) arasındaki mesafe **Haversine** formülüyle veya Vincenty algoritması ivmelendirilmiş hali ile hesaplanır:

$$ \Delta\phi = \phi_2 - \phi_1 $$
$$ \Delta\lambda = \lambda_2 - \lambda_1 $$
$$ a = \sin^2\left(\frac{\Delta\phi}{2}\right) + \cos(\phi_1)\cos(\phi_2)\sin^2\left(\frac{\Delta\lambda}{2}\right) $$
$$ c = 2 \cdot \operatorname{atan2}\left(\sqrt{a}, \sqrt{1-a}\right) $$
$$ d = R \cdot c $$

Sistem bu sayede $\Delta Lat, \Delta Lon$ cinsi radyan hatalarını metre cinsine ($d_{lat\_meter}, d_{lon\_meter}$) doğru şekilde çevirir.

### 3.2. Sürünün Ağırlık Merkezi (Center of Gravity - CoG)

Formasyon uçuşunun kilit noktası, otonom kararların dronun kendisine göre değil, sürünün kütle merkezine (CoG) göre alınmasıdır.

Elimizde $N$ adet drone (komşular + ana drone) olduğunu varsayarsak, bu koordinatlar WGS84 üzerinden toplanıp ortalanır:

$$ \text{CoG}_{\text{lat}} = \frac{1}{N} \sum_{i=1}^{N} \text{Lat}_i $$
$$ \text{CoG}_{\text{lon}} = \frac{1}{N} \sum_{i=1}^{N} \text{Lon}_i $$

Bu, algoritmada `geo::calculate_cog<VehicleGlobalPosition>(all_positions)` çağrısı aracılığıyla vektörel olarak anlık çözümlenir.

### 3.3. Doğrultu (Bearing) Çıkarımı

Kütle merkezinin (CoG) yönünün nereye dönmesi gerektiğini anlamak üzere başlangıç ($\phi_1, \lambda_1$) noktasından hedef ($\phi_2, \lambda_2$) noktasına giden okun mutlak açısı (kuzeye göre yönelim) çıkarılmalıdır:

$$ \theta = \operatorname{atan2}\left( \sin(\Delta\lambda)\cos(\phi_2) , \cos(\phi_1)\sin(\phi_2) - \sin(\phi_1)\cos(\phi_2)\cos(\Delta\lambda) \right) $$

### 3.4. Formasyon Rotasyonu Algoritması (Formation Rotation)

Drone'lar sadece hedefe doğru dümdüz uçmazlar; önce kütle merkezleri etrafında (gezegenin güneş etrafında dönmesi gibi) hedefe doğru **yönelerek (Bearing matching)** kendi hizalarını korurlar.

1. **Hedef ve En Yakın Araç Açısı Bulunur:** 
   O an sürüde hedefe **en yakın** (Nearest) aracın CoG'a olan açısı ile, CoG'dan **Hedefe (Target)** olan açık karşılaştırılır. Sürünün dönmesi gereken miktar:
   
   $$ \theta_{\text{rotasyon}} = \text{WrapAngleToPi}(\theta_{\text{CoG} \to \text{Target}} - \theta_{\text{CoG} \to \text{Nearest}}) $$

2. **Dronun Daire Üzerindeki Yeni Yeri (Circular Position):**
   Mevcut drone'un (THIS DRONE) CoG etrafındaki kendi standart açısı bulunur:
   $$ \theta_{\text{kendi\_açım}} = \theta_{\text{CoG} \to \text{O anki Konum}} $$
   Drone'un hedefe bakması için gitmesi gereken yeni açı formülüze edilir:
   $$ \theta_{\text{yeni\_hedef}} = \text{WrapAngleToPi}(\theta_{\text{kendi\_açım}} + \theta_{\text{rotasyon}}) $$

3. **İleri Projeksiyon (Forward Geolocation):**
   CoG noktasından başlanıp yeni açı yönü ($\theta_{\text{yeni\_hedef}}$) ve yarıçap mesafesinde (radius) ileriye atılarak yeni konum tespit edilir:
   $$ \phi_{\text{yeni}} = \arcsin\left(\sin(\phi_{\text{CoG}})\cos(\delta) + \cos(\phi_{\text{CoG}})\sin(\delta)\cos(\theta_{\text{yeni\_hedef}})\right) $$
   *(Burada $\delta = \text{Mesafe} / R$)*

Sırasıyla `calculate_target_bearing_for_drone` ve `calculate_new_point` mekanizması matematiksel olarak bunu otonom işletir.

### 3.5. Oransal Hız Kontrolü (P-Controller Velocity Clamping)

Yeni nokta veya rota hesaplandıktan sonra drone anında ışınlanamayacağı için, aradaki farktan (error) bir PID-orantılı hız dizgesi çıkarılır.

$$ Hata (e) = Konum_{istenen} - Konum_{mevcut} $$
$$ V_{\text{hesaplanan}} = e \times K_p $$
$$ V_{\text{son_hız}} = \max\left(-V_{\text{max\_limit}}, \min(V_{\text{hesaplanan}}, V_{\text{max\_limit}})\right) $$

`autonomus_utils::calculate_velocity<double>(error, desired_vel)` fonksiyonu yardımıyla güvenli maksimum hızı asla aşmayan (*clamped*) pürüzsüz hızlanma (smooth transition) verileri PX4'e (`TrajectorySetpoint.velocity`) itilir.

---

## 4. Bellek Güvenliği ve `WaypointManager` Mimarisi

C++ tabanlı kontrol sistemlerinde yaşanan en büyük sorunlardan biri, `std::vector` gibi yapıların dışına çıkılması sonucu tetiklenen **Segmentation Fault (Çökme)** hatalarıdır.

Önceki tasarımlarda hedefler statik index dizileri `i_curr_wp_` ile okunuyordu. Bu indeksin boyutu geçmesi yazılım çökmesine, drone'un ise olduğu yerde donmasına ya da irtifa kaybetmesine yol açar.
**Çözüm:** `autonomus_utils.hpp` içerisinde **`WaypointManager`** class'i oluşturuldu.

**Özellikleri:**
- Dışarıdan hedefler `set_waypoints()` arayüzü ile eklenir.
- Liste boyutu her sorguda (`current_index_ < waypoints_.size()`) kontrol edilir.
- `waypoint_manager_.current()` komutu; o anki hedefin pointer'ını döndürür. Eğer liste bitmişse `nullptr` döndürerek sonsuz döngü veya bellek sızıntısını keser.
- Görev bittiğinde veya tetiklendiğinde `waypoint_manager_.next()` çağrılarak internal index bir (1) arttırılır ve yeni birim döndürülür.

```cpp
const auto wp = waypoint_manager_.current();
if (!wp) return; // Hedef bittiyse crash yerine güvenli çıkış yap
// Güvenli pointer kullanımı: 
double z_error = current_altitude + wp->alt;
```

---

## 5. `SwarmMemberPathPlanner` State Machine Akışı

Robotikte "Durum Makinesi" (State Machine) çok yaygındır. Ana algoritma `autonomus_missions.cpp` içerisinde Enum yapısıyla tutulur ve bir Lifecycle timer loop ($10$ ms - $100$ Hz) içerisinde akıcı tepki verir.

### 5.1. FORMATIONAL_TAKEOFF (Formasyon Kalkışı)
- Her bir drone, `target_altitude_` (Varsayılan `-10.0` m) parametresini okur. 
- İrtifa hatası (`z_error = current_altitude + wp->alt;`) ölçülür ve yukarıya doğru dikey `z_vel` kontrolcüsü çalışır.
- Drone istenilen irtifaya varınca ($\text{error} < 0.01\text{m}$), **`NeighborVerification`** fonksiyonunu bekler. Bu fonksiyon sayesinde **tüm sürünün o irtifaya ulaşması** beklenir ki bir drone rotasyona geçerken diğeri altta kalıp çarpışmasın (Senkronizasyon kilidi).

### 5.2. FORMATIONAL_ROTATION (Formasyon Dönüşü)
- Tüm takım havalandığında rota belirleme aşaması başlar. 
- Drone, [bölüm 3.4](#34-formasyon-rotasyonu-algoritması-formation-rotation)'te açıklanan matematiksel işlemlerle yeni konumunu hesaplar (`swarm_positions.circular_position`).
- Yanal (Latitudinal) ve Dikey (Longitudinal) mesafeler `dlat_meter` ve `dlon_meter` cinsinden hıza ($V_x, V_y$) dönüştürülür ve rotasyon başlar.
- O konuma gelindiğinde ROS 2 Service çağrısı (`InTarget`) gönderilerek, "Ben yerimi aldım" denir. Diğerleri de döndüyse sonraki adıma atlanır.

### 5.3. GOTO_POSITION (Konuma İlerleme)
- Tüm formasyon yüzünü hedefe (Waypoint) döndükten sonra kütlece ilerlenir.
- Drone'un hızları `current_commands.v_lat` ve `current_commands.v_lon` olarak hesaplanır.
- Ayrıca bu adımda araca **`collision_bias`** (Çarpışma Engelleme İtme Vektörü) dahil edilir (Örn: `v_lat += collision_bias.vlat`). Ortamda algılanan yabancı cisimler itki vektörüyle sürünün içine ek kuvvet basar.

---

## 6. Kapsamlı Kullanım Kılavuzu (Usage Guide)

### Gereksinimler
- **Ubuntu 22.04 LTS** ve **ROS 2 Humble**
- PX4 Autopilot (Simulation - gazebo-classic/ignition veya Hardware)
- `px4_msgs` sürüm uyumunun sağlanması (U-XRCE-DDS)

### Derleme
Sadece sürü kontrol modülünün derlenip test edilmesi için Workspace kök dizininde (`~/ws_offboard_control/`):
```bash
colcon build --packages-select swarm_drone_control --symlink-install
```
*Environment sourcing:* `source install/setup.bash`

### Uygulamanın Başlatılması ve Parametreler
Lifecycle Node olduğu için standart Lifecycle yayıncısı ile tetiklenebilir ya da `ros2 run` kullanılarak konfigürasyon yapılabilir. Sistem, hedefe uçmak için `target_altitude_` parametresi ve `target_lat`, `target_lon` gibi ROS paramlarına sahiptir.

```bash
ros2 run swarm_drone_control autonomus_main_node --ros-args -p sys_id:=1 -p target_lat:=47.398000 -p target_lon:=8.546000 -p target_alt:=-15.0
```

*Not:* Yükseklik (`alt`) değerleri PX4 dünyasında **NED (North-East-Down)** referansından dolayı yeryüzünün üstüne çıkıldıkça **negatif** (Örn: Havada 10m = `-10.0`) değer almaktadır! 

### Yayınlanan Topic ve Servisler (Topic and Service Interfaces)
- **`fmu/in/trajectory_setpoint`**: İHA'ya hızın/koordinatın basıldığı PX4 DDS konusu.
- **`/in_position`**: Dronun hedefte olduğunu senkronizeleyen Service (Client/Server).
- **`neighbors_info`**: (Özel Arayüz) Kendi ID'niz dışındaki diğer sistemlerin pozisyon ve durumunu çekmek için.

---

## 7. API ve Fonksiyon Referansları

Bu paketi sonradan devralan veya birleştirmek isteyen geliştiriciler için temel API tabloları:

| Fonksiyon Adı | Kaynak (Dosya) | Görevi / Özelliği | Dönüş Tipi |
|---------------|----------------|-------------------|------------|
| `WaypointManager::current()` | `autonomus_utils.hpp` | O anki WP pointer'ini verir (Failsafe). | `const VehicleGlobalPosition*` |
| `WaypointManager::next()` | `autonomus_utils.hpp` | İndeksi artırır ve sonraki WP'yi verir. | `const VehicleGlobalPosition*` |
| `calculate_target_bearing_for_drone`| `autonomus_utils.hpp` | CoG ve Nearest tabanlı formasyon açısı bulur | `double` (Radyan) |
| `calculate_velocity<T>` | `autonomus_utils.hpp` | PID algoritması ve Clamping çalıştırır. | `template T` |
| `geo::calculate_distance` | `geographic.hpp` | İki GPS verisi (WGS84) arası metrik mesafe ölçer. | `DLatDLon (Struct)` |
| `spatial::WrapAngleToPi` | `spatial.hpp` | Açıyı $-\pi$ ile $+\pi$ sınırlarına bastırır. | `double` |
| `formational_takeoff()` | `autonomus_missions.cpp` | Güvenli senkronize kalkış state komutlarıdır. | `void` |

---

*Geliştirme, Bakım ve Matematik Modellemesi: `swarm_drone_control` Development Core - Mart 2026*
