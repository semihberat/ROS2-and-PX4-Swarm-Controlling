# Sürü İHA (Swarm Drone Control) Kapsamlı Sistem, Mimari ve Matematiksel Modeller Dokümantasyonu

Bu belge, **ROS 2 (Robot Operating System 2)** ve **PX4 Framework (MAVLink/MAVROS ve uORB/DDS)** üzerine inşa edilmiş `swarm_drone_control` paketinin uçtan uca çalışmasını anlatan **kapsamlı mühendislik ve teorik altyapı** referansıdır.

Bu döküman, projenin matematiksel altyapısını, otonom seyir logaritmasını (navigation log), güvenlik katmanlarını ve sistemin asenkron çalışma prensiplerini detaylandırmaktadır. Projenin ana hedefi, bir "Sürü" (Swarm) topolojisindeki İHA'ların ortak bir uçuş planını çarpışmadan (collision avoidance), asgari ağ gecikmesiyle (minimum latency) ve WGS84 uzamsal modeline uygun bir şekilde tamamlamasıdır.

---

## İçindekiler Tablosu

1. [Sisteme Genel Bakış ve Problemin Tanımı](#1-sisteme-genel-bakış-ve-problemin-tanımı)
2. [Sistem Mimarisi, Dizin Yapısı ve Modülerlik](#2-sistem-mimarisi-dizin-yapısı-ve-modülerlik)
   - 2.1. Dağıtık ve Merkeziyetsiz Mimari
   - 2.2. Dizin Organizasyonu
3. [ROS 2 Lifecycle ve State Machine Dinamikleri](#3-ros-2-lifecycle-ve-state-machine-dinamikleri)
   - 3.1. Düğüm Evreleri (Node Transitions)
   - 3.2. Görev-Durum (Mission State) Makinesi
4. [Teorik Altyapı ve Matematiksel Algoritmalar](#4-teorik-altyapı-ve-matematiksel-algoritmalar)
   - 4.1. WGS84 Coğrafi Koordinat Modelleri
   - 4.2. Mesafe Belirleme (Haversine Formula)
   - 4.3. Mutlak Doğrultu (Absolute Bearing) Çıkarımı
   - 4.4. Ağırlık Merkezi (Center of Gravity - CoG) Yaklaşımı
   - 4.5. Hedef Rotasyon Algoritması ve Geometrik İzdüşüm
   - 4.6. Euler/Quaternion Dönüşüm Teorisi
5. [Oransal (PID) Hız Kontrolü Katmanı](#5-oransal-pid-hız-kontrolü-katmanı)
   - 5.1. Continuous Time vs Discrete Time Karşılaştırması
   - 5.2. Hata (Error) Çıkarımı ve Clamping Limits
6. [Bellek Yönetimi ve `WaypointManager` Mimarisi](#6-bellek-yönetimi-ve-waypointmanager-mimarisi)
   - 6.1. Sorun: Vektör İndeks Çökmeleri (Segmentation Faults)
   - 6.2. Çözüm: Bağlı Liste (Linked List) Felsefesi ve Failsafe
7. [Çarpışma Önleme (Collision Avoidance) Mimarisi](#7-çarpışma-önleme-collision-avoidance-mimarisi)
8. [ROS 2 QoS, DDS ve Haberleşme Modeli](#8-ros-2-qos-dds-ve-haberleşme-modeli)
9. [Kod İçi API ve Fonksiyon Referansları](#9-kod-içi-api-ve-fonksiyon-referansları)
   - 9.1. `autonomus_utils.hpp` Çekirdek İşlevleri
   - 9.2. `geographic.hpp` Yardımcıları
10. [Adım Adım Kurulum ve Kullanım Kılavuzu](#10-adım-adım-kurulum-ve-kullanım-kılavuzu)
    - 10.1. Gereksinimler ve Bağımlılıklar
    - 10.2. Build İşlemleri
    - 10.3. Çalıştırma Parametreleri
    - 10.4. ROS Topic Haritası

---

## 1. Sisteme Genel Bakış ve Problemin Tanımı

Birden çok İnsansız Hava Aracının dar veya geniş alanlarda ortak bir vizyon doğrultusunda uçması (Swarm Robotics), modern havacılığın en kompleks algoritmik problemlerinden biridir. Geleneksel sistemler "Master/Slave" bağımlılığı üzerinden çalışır. Yani yerdeki veya gökyüzündeki bir merkez bilgisayar ağdaki her bir drone'a ne yapması gerektiğini dikte eder. Ancak bu tasarımın en büyük handikabı, merkezdeki ünitenin haberleşmeyi kaybetmesi veya düşmesi durumunda tüm sürünün de felç geçirmesidir.

**Çözüm:** `swarm_drone_control` paketinin otonom seyir modülü olan `SwarmMemberPathPlanner`, **merkeziyetsiz (decentralized)** bir formasyon geometrisi uygular. Her bir drone, uORB DDS (Data Distribution Service) köprüsü ile gelen `NeighborsInfo` (komşu bilgileri) topic'ini okur, formasyon içindeki kütle merkezini tespit eder ve etrafında dönerek rotasını dinamik bir vektörel tepki ile ayarlar. Böylece sistemden bir drone düşse bile diğerleri kendi ağırlık merkezlerini baştan asenkron hesaplayarak formasyonu anında onarabilir.

## 2. Sistem Mimarisi, Dizin Yapısı ve Modülerlik

### 2.1. Dağıtık ve Merkeziyetsiz Mimari
- **Node (Düğüm) Standardı:** Sistemdeki her bir `SwarmMemberPathPlanner` bir `rclcpp_lifecycle::LifecycleNode` objesinden ürer.
- **Micro-XRCE-DDS:** PX4 tabanlı iletişim standardına ayak uydurmak üzere ROS 2 Humble ile PX4 Firmware arasında köprüleme yapar.
- **İzolasyon:** Her dronun kendi sistemi sadece "kendisinden" (`TrajectorySetpoint` basmak) ve dışarıdan aldığı komşu pozisyonlarından sorumludur.

### 2.2. Dizin Organizasyonu
Daha önce iç içe geçmiş olan spagetti kodu, profesyonel bir yapı standartına getirilmiştir:
- `src/autonomus/`: Tüm temel kararlar bu dizinden çıkar. (Uçuş evreleri, timer'lar ve otonom tepkiler)
- `include/calculations/`: Salt matematik fonksiyonları sınıfıdır (Haversine, Bearing, Euler to Quaternion).
- `src/communication/`: DDS ve ağ problemleri, IP atamaları, port eşleştirmeleri ve komşu ağ protokollerini içerir.

---

## 3. ROS 2 Lifecycle ve State Machine Dinamikleri

### 3.1. Düğüm Evreleri (Node Transitions)
Düğümler doğrudan uçuşa geçecek bir komut uygulamaz. Önce sistemin her bir sensörden veri alabildiğinden emin olması gerekir:
1. **Unconfigured:** Başlangıç state'i. Hiçbir bellek tahsisi henüz gerçekleştirilmedi.
2. **Inactive (Configured):** `on_configure` tetiklendi. Pointers tanımlandı, Waypoint serisi yüklendi ancak motorlara komut verilmiyor.
3. **Active:** Sistem `on_activate` tetiklemesiyle uyanır. `state_cycle_callback` timer loop'u motorlara Hız verilerini gönderir. İHA Offboard (Otonom) moda geçer ve formasyona kalkar.

### 3.2. Görev-Durum (Mission State) Makinesi
Bir C++ `enum class Mission` ile sürünün hangi evrede olduğu 10 Milisaniyede ($100 \mathrm{Hz}$) bir denetlenir.

- `FORMATIONAL_TAKEOFF`: İHA'nın Z ekseninde havalanması ($e_z = z_{\mathrm{hedef}} - z_{\mathrm{mevcut}}$ formülüyle). Oransal kontolör ile pürüzsüzce kalkış yapar.
- `FORMATIONAL_ROTATION`: Kütle merkezine göre formasyonun yönünü belirleme ve hedefe doğru dönmesi.
- `GOTO_POSITION`: Dönüş bittikten sonra tam konuma teğetsel X-Y vektörel gidiş işlemi.
- `DO_PROCESS` ve `END_TASK`: Görev noktasında bekleyip aksiyon alma veya sensör verisi depolama.

---

## 4. Teorik Altyapı ve Matematiksel Algoritmalar

Bu başlık, `autonomus_utils.hpp` ve `calculations/` içerisinde kodlanmış olan tüm matematik formülleri ve coğrafi uzamsal işlemlerin çekirdek bilgisini içerir.

### 4.1. WGS84 Coğrafi Koordinat Modelleri
Dünya düz olmadığı için harita üzerindeki herhangi iki nokta arasındaki işlemi 2D Descartes koordinat sistemi ($X$, $Y$) ile yapmak büyük hata verir. Ekvator yarıçapının $R = 6378137.0 \mathrm{\ m}$ olduğu bir küresel uzay olan **WGS84** projeksiyonu esastır. `VehicleGlobalPosition` nesnesindeki ($\phi: \mathrm{Latitude}$, $\lambda: \mathrm{Longitude}$) girdilerini kullanırız.

### 4.2. Mesafe Belirleme (Haversine Formula)
İki drone (iki WGS84 koordinat seti) arasındaki coğrafi mesafenin ölçümü radyan bazında alınır. ($\phi_1, \lambda_1$) ve ($\phi_2, \lambda_2$) nokta girdileri üzerinde;

Hata miktarı tespiti:

$$ \Delta\phi = \phi_2 - \phi_1 $$

$$ \Delta\lambda = \lambda_2 - \lambda_1 $$

Merkezi küre kiriş (chord) uzunluğunun karesi olan $a$ parametresi:

$$ a = \sin^2\left(\frac{\Delta\phi}{2}\right) + \cos(\phi_1)\cos(\phi_2)\sin^2\left(\frac{\Delta\lambda}{2}\right) $$

Küresel açı ve mesafe izdüşümü hesaplanması:

$$ c = 2 \cdot \mathrm{atan2}\left(\sqrt{a}, \sqrt{1-a}\right) $$

$$ d = R \cdot c $$

Mesafe ($d$), metre cinsinden çıkartılır. `geo::calculate_distance<DLatDLon>` fonksiyonunun taban formulizasyonu budur.

### 4.3. Mutlak Doğrultu (Absolute Bearing) Çıkarımı
Drone formasyonunda, merkez birimlerin coğrafi kuzeye (True North) göre hangi açıyla (derece/radyan cinsinden) baktığını bulmamız gerekir. İki WGS84 noktasından türetilen Başlangıç yönelimi (Initial Bearing) formülü şöyledir:

$$ \theta = \mathrm{atan2}\left( \sin(\Delta\lambda)\cos(\phi_2) , \cos(\phi_1)\sin(\phi_2) - \sin(\phi_1)\cos(\phi_2)\cos(\Delta\lambda) \right) $$

Sonuç $\theta$ bir radyan cinsindendir ve $0$ Kuzey, $\pi/2$ Doğu, $\pm\pi$ Güney, $-\pi/2$ Batı'ya karşılık gelir.

### 4.4. Ağırlık Merkezi (Center of Gravity - CoG) Yaklaşımı
Ağırlık merkezi formasyonda "sürünün görünmez beyni/çekirdeği"dir. Topolojik bir hesapla sürüdeki tüm İHA'ların nokta bulutları ortalanarak bu sanal nokta yaratılır. Kod içerisindeki iteratif fonksiyon şudur:

ELİNDEKİ SÜRÜDEKİ DRONELERI KOORDİNASYON AŞAMASI:

$$ \mathrm{CoG}_{\mathrm{lat}} = \frac{1}{N} \sum_{i=1}^{N} \mathrm{Lat}_i \quad , \quad \mathrm{CoG}_{\mathrm{lon}} = \frac{1}{N} \sum_{i=1}^{N} \mathrm{Lon}_i \quad (N = \mathrm{Tum\ Drone\ Sayisi}) $$

`geo::calculate_cog<VehicleGlobalPosition>(all_positions)` ile çağırılır.

### 4.5. Hedef Rotasyon Algoritması ve Geometrik İzdüşüm
Formasyonun yüzünü hedefe dönmesini sağlayan **Formasyon Döndürme** mekanizması şu kurallarla ardışık hesaplanır:

1. **Önce mevcut sapmayı bul (Formation Error):**

$$ \theta_{\mathrm{rotasyon}} = \mathrm{WrapAngleToPi}(\theta_{\mathrm{CoG} \to \mathrm{Target}} - \theta_{\mathrm{CoG} \to \mathrm{Nearest\_Drone}}) $$

Bu $\theta_{\mathrm{rotasyon}}$, tüm dairesel formasyonun kendi ekseninde "ne kadar" açıyla, "nereye" dönmesi gerektiğini barındırır.

2. **Dronun Dairedeki Yeni Hedef Açısı (New Node Circular Degree):**
Benim drone'um (THIS) şu anki kütle merkezinden kaç derecede duruyor? Formasyon tamamen döndüğünde, benim aracıma düşen yeni açısal sapma payı ve teğet koordinatı şöyledir:

$$ \theta_{\mathrm{kendi\_acim}} = \theta_{\mathrm{CoG} \to \mathrm{Mevcut\_Pozisyon}} $$

$$ \theta_{\mathrm{yeni\_hedef}} = \mathrm{WrapAngleToPi}(\theta_{\mathrm{kendi\_acim}} + \theta_{\mathrm{rotasyon}}) $$

Açıların $+\pi$ ile $-\pi$ arasında kalması için `WrapAngleToPi` bir çark görevi görür ve trigonometrik hataları $\sin, \cos, \tan$ fonksiyon grafiğinden aşmasına müsaade etmez. 

3. **Uzamsal Projeksiyon:**
Sanal konumun ($\theta_{\mathrm{yeni\_hedef}}$  doğrultusunda $\delta$ metre ilerisinin) CoG'a uygulanması ile yeni GPS hedefleri çıkartılır:

$$ \phi_{\mathrm{yeni}} = \arcsin\left(\sin(\phi_{\mathrm{CoG}})\cos(\delta) + \cos(\phi_{\mathrm{CoG}})\sin(\delta)\cos(\theta_{\mathrm{yeni\_hedef}})\right) $$

Bu denklemde radyal $\delta$, mesafe/Yer Yarıçapı $d / R$ denklemiyle standardize edilmiştir.

### 4.6. Euler/Quaternion Dönüşüm Teorisi
Dronun fiziksel yalpalaması (Attitude data), PX4'ten Quaternion (`[q0, q1, q2, q3]`) veya Quaternion struct'ı olarak gelir. Çarpışma algılayıcılar veya formasyon hesabı bu veriyi anlayabilmek için bir fonksiyonla Gimbal kilitlenmesi yaşamayacağı tarzda `Roll`, `Pitch` ve `Yaw` verilerine döker (`spatial.hpp` içerisinde barınır). Dronun yüzü kuzeye ($\theta = 0$ radyan) doğru döndükçe otonomi sistemi hedefe vardığını onaylar.

---

## 5. Oransal (PID) Hız Kontrolü Katmanı

Dönüşümü bulduk. Peki oraya nasıl gideceğiz? Eski nesil sistemlerde direkt "Konuma Işınlanma" komutu verilir ancak bu kontrolcü sapmasına (drift) sebep olur. Bu yazılım hız tabanlı kapalı çevrim kontrolcü `Offboard` modu olan `TrajectorySetpoint` dizgilerini kurgular. Temel olarak bir **P-Controller (Oransal Kontrolör)** çalıştırılır.

### 5.1. Continuous Time vs Discrete Time Karşılaştırması
Sistem $10\mathrm{\ ms}$ frekansta ($100 \mathrm{Hz}$) iterasyon yaptığı için hesaplamalar Ayrık Zamanlı (Discrete) matematik formlarına dökülmüştür. Oransal kazanç $K_p$, sabit $P\_GAIN = 0.5$ şeklindedir.

### 5.2. Hata (Error) Çıkarımı ve Clamping Limits
Diyelim ki drone'un varış X vektörü $20$ metre ileride. $e = 20$. 
İstenilen hız $= 20 \times 0.5 = 10\mathrm{\ m/s}$. 
Ancak İHA'nın rüzgarda takla atmaması için hızına bir limit verilir. `autonomus_utils::calculate_velocity<T>` bu formülü çalıştırır:

$$ V_{\mathrm{cikis}} = \mathrm{clamp}(e \times K_p, \quad -V_{\mathrm{max\_hiz}}, \quad V_{\mathrm{max\_hiz}}) $$

Sonuç: Limit $2.0 \mathrm{\ m/s}$ ise sistem $2.0 \mathrm{\ m/s}$ döndürür, drone konuma yaklaştıkça (*Mesela* $e=2$) hız pürüzsüz biçimde asimptotik olarak yavaşlamaya (Smooth deceleration) başlar. ($e \times K_p = 2 \times 0.5 = 1.0 \mathrm{\ m/s}$).

---

## 6. Bellek Yönetimi ve `WaypointManager` Mimarisi

C++ dili güçlüdür ancak bellek taşması (Buffer Overflow) ve belirsiz işaretçi (Dangling Pointer) hatalarına fazlasıyla meyillidir.

### 6.1. Sorun: Vektör İndeks Çökmeleri (Segmentation Faults)
Pek çok yazılımda drone hedefleri `std::vector` içinde toplanır. Drone hedefe vardığında `i_curr_wp_` integer'ı arttırılır (`i_curr_wp_ += 1;`). Hedef dizisi 5 tane ise ve araç ardı ardına 6 konuma gitmeye denerse, C++ `std::vector[6]` bellek alanına bakarak yetkisiz erişim başlatır. Bu; programı anında çökertir (Segfault) ve ROS node düşer. Sonuç: Cihaz havada kontrolden çıkar.

### 6.2. Çözüm: Bağlı Liste (Linked List) Felsefesi ve Failsafe
`autonomus_utils.hpp` içerisine kurulan **`WaypointManager`** class'i ile bellek yönetimi dış dünyaya kapalı, Failsafe garantili bir kapsüllemeye (Encapsulation) oturtuldu.

**Özellikleri:**
- Sisteme hedefler `set_waypoints()` arayüzü ile eklenir. `current_index_{0}` initialize edilir.
- Bir C/C++ yapısında boyutu `(size_t)` hesaplayarak array check uygular (`current_index_ < static_cast<int>(wp_msg_->waypoints.size())`).
- **Okuma/Getirme:** `waypoint_manager_.current()` komutu; o anki waypoint yapısının adres pointer'ını döndürür. Eğer liste geçersizleşirse veya bitmişse `nullptr` döndürerek sonsuz döngü veya bellek okuma sızıntısını kapatır.
- İlgili hedef tamamlandığında (`autonomus_missions.cpp` adım atladığında) `waypoint_manager_.next()` çağrılarak internal index bir birim iterasyon yaptırılır ve güvenle bir sonraki veriye kayılır.

Örnek güvenli okuma:
```cpp
const auto wp = waypoint_manager_.current();
if (!wp) return; // wp boş (nullptr) ise alttaki kodlar hiç çalışma
double z_error = current_altitude + wp->alt;
```

---

## 7. Çarpışma Önleme (Collision Avoidance) Mimarisi

Dronlar rotasyondan çıkıp `GOTO_POSITION`'da hedefe uçarken, sensörlerin veya komşu telemetrilerin (NeighborsInfo DDS) algıladığı objeler, bir potansiyel enerji alanı (Artificial Potential Field Vector) yaratacak şekilde hesaplanır:

1. Başlangıçta hedefe giden teorik X ve Y hızları bulunur. ($V_{\mathrm{hedef\_lat}}$, $V_{\mathrm{hedef\_lon}}$)
2. Çarpışma riski taşıyan bir obje yaklaşmaktaysa, o objenin dronumuz üzerinde itici bir kuvvet vektörü (`collision_bias.vlat` , `.vlon`) yarattığı fiziki model işleme koyulur.
3. Drone, iki hız vektörünü üst üste toplar (Superposition Model):

$$ V_{\mathrm{son\_x}} = V_{\mathrm{hedef\_lat}} + \mathrm{Bias}_{\mathrm{carpisma\_x}} $$

$$ V_{\mathrm{son\_y}} = V_{\mathrm{hedef\_lon}} + \mathrm{Bias}_{\mathrm{carpisma\_y}} $$

Böylece drone objeden teğet kaçarak yumuşak bir yörünge çizer ve hedef noktasına ulaşmadan önce "sekter" eylemi gerçekleştirmiş olur.

---

## 8. ROS 2 QoS, DDS ve Haberleşme Modeli

Havada sinyalin kopma ihtimali çok yüksektir. TCP protokolündeki veri doğrulama istekleri bu sebeple robotikte ciddi gecikmelere (Latency Lag) sebep olur. UDP tabanlı DDS (Data Distribution Service) **Reliability QoS** parametreleri ile şekillendirilir.

Düğüm (Node) arası ve FCU (PX4 U-XRCE-DDS Micro) arası mesajlar (`NeighborsInfo`, `TrajectorySetpoint`):
- `Best Effort`: Mesaj yolla ama gelip gelmediği kontrolü sormama, doğrudan ağa fırlat.
- `Volatile`: Son gelen mesajı saklama, yeni gelene adapte ol.
- Prensibiyle kodlanmıştır, sürünün iletişimde kalitesizlik olsa bile kilitlenmelere sebebiyet vermez.

---

## 9. Kod İçi API ve Fonksiyon Referansları

Bu paketi devralan veya birleştirmek isteyen robotik/yazılım geliştiriciler için temel API ve Helper C++ sınıfları:

### 9.1. `autonomus_utils.hpp` Çekirdek İşlevleri

| Fonksiyon/Struktür Adı | Kullanım Çerçevesi | Görevi / Özelliği | Dönüş Tipi |
|------------------------|--------------------|-------------------|------------|
| `WaypointManager::current()` | Otonom Akış Çekirdeği | O anki (Memory Safe) WP pointer'ini verir. Sistem bitmişse `nullptr`. | `const VehicleGlobalPosition*` |
| `WaypointManager::next()` | Görev Geçişleri | İndeksi güvenle ($N < size$) artırır ve sonraki WP'yi pointer olarak iterasyonla verir. | `const VehicleGlobalPosition*` |
| `combine_positions` | Kütle Merkezi Grubu | Kendisi dahil komşuları DDS üzerinden okuyarak tek `array`/`list` haline sarmalar. | `std::vector<VehicleGlobalPosition>` |
| `calculate_target_bearing_for_drone`| Rotasyon Algoritması | Tüm komşu araçların yüzey alanındaki CoG tabanlı açı yönlemesini ve dönüş rotasını kurgular. | `double` (Radyan) |
| `find_nearest_vehicle_to_target` | Geometrik Kıyas | Sürü içerisinden hedefe kuş uçuşu en yakın olan drone referansını diziden seçer. | `VehicleGlobalPosition` |
| `calculate_velocity<T>` | Hız Kontrolü | PID algoritması çalıştırır ve ebatları `T` sınırları dışına (clamping threshold) taşan hataları keser. | `template T` (Örn. double) |
| `NeighborVerification` | Lambda Koşul Sorgusu | Tüm sürünün `LAMBDA` tabanlı şarta uyup uymadığını (`std::all_of`) kontrol ederek formasyon lock koyar. | `bool` |

### 9.2. `geographic.hpp` Yardımcıları

| Fonksiyon Adı | Kayıtlı Olduğu Kütüphane | Görevi / Özelliği | Matematik Formülü | Dönüş Tipi |
|---------------|--------------------------|-------------------|-------------------|------------|
| `geo::calculate_distance` | `<geographic.hpp>` | WGSGPS İki Veri Arası Doğrusal Metre ve Radyal Mesafesi. | Haversine Formula | `DLatDLon (Struct)` |
| `geo::calculate_bearing` | `<geographic.hpp>` | İki WGS noktası arası True North Yöneliş Açısını hesaplar. | ArcTangent Bear. | `double` |
| `geo::calculate_cog` | `<geographic.hpp>` | Sürünün Kütle Ağırlık Merkezi (Center of Globus). | Average Pointing | `VehicleGlobalPosition` |
| `spatial::WrapAngleToPi` | `<spatial.hpp>` | Yüksek açı ve Euler problemlerini $-\pi$ / $+\pi$ arasına izomerleştirir. | $f(\theta)$ wrap | `double` |
| `geo::calculate_offsets` | `<geographic.hpp>` | Merkezden İleri ve Yan mesafelerle yeni GPS üretimi. | WGS Inverse. | `std::vector<WP>` |

---

## 10. Adım Adım Kurulum ve Kullanım Kılavuzu

Projenin kendi bilgisayarınızda bir GCS (Ground Control Station), donanım drone veya SITL ortamında başlatılması için izlenmesi gereken yörünge.

### 10.1. Gereksinimler ve Bağımlılıklar
Paket spesifik sistem bileşenleriyle inşa edilmiştir:
- **Ubuntu 22.04 LTS (Jammy)** İşletim Sistemi.
- **ROS 2 Humble Hawksbill** (Full Desktop Installation önerilir).
- **PX4-Autopilot** Toolchain: PX4'ün `px4_msgs` veri formlarıyla derlenebilmesi gerekir.
- **Micro XRCE-DDS Agent**: FCU (Flight Controller Unit) DDS bağlantısı için ara port arayüzüdür (FastDDS ile koordine çalışır).

### 10.2. Build İşlemleri
Projenin tamamen kurulabilmesi veya sadece modüllerinin temizlenip (`clean`) inşa (`build`) edilebilmesi için Workspace kök dizinine geçilerek aşağıdaki işlemler yapılır:

```bash
cd ~/ws_offboard_control
# Bağımlılıkları kontrol etme
rosdep install --from-paths src --ignore-src -r -y

# Yalnızca Otonomi Paketini (ve symlinkleri, Python senaryoları için) Derleme
colcon build --packages-select swarm_drone_control --symlink-install

# Yeni kurulan ROS paket sistemini Environment'e geçirme
source install/setup.bash
```

*(Önemli Not:* `px4_msgs` kütüphanesi yüklü değilse, öncesinde PX4 bağımlılıklarının veya `colcon build --packages-select px4_msgs` komutunun verilmiş olduğundan emin olunuz.)*

### 10.3. Çalıştırma Parametreleri
İlgili Lifecycle konseptli node'un parametrelerle ayaklandırılması komutu:

```bash
ros2 run swarm_drone_control autonomus_main_node \
  --ros-args \
  -p sys_id:=1 \
  -p target_lat:=47.398000 \
  -p target_lon:=8.546000 \
  -p target_alt:=-15.0
```

- `-p sys_id:=X`: Her node'a MAVLink Target üzerinden kimlik numarası (SYSID) atar. Çakışma durumunda komutlar ulaşmaz.
- `-p target_alt:=-15.0`: Sistemin hedef (Z eksen) yüksekliğidir. Otonomi sistemi **NED (North-East-Down)** standardına tabidir, yerin yukarısı **Negatif ($\mathbf{-Z}$)** değerindedir. Örneğin $20 \mathrm{\ metre}$ uçuç, `-20.0` anlamına gelir. Aksi istenirse drone yerin içine (`+Z`) çakılmaya çalışacaktır.

Sistem Lifecycle uyumlu olduğu için Terminal'den yayınlandıktan sonra `[Unconfigured]` uyku modunda sistem kontrollerinin açılmasını bekler. Aktifleştirmek için harici bir ros2 lifecycle komutu kullanılmalıdır (*örneğin* `ros2 lifecycle set /swarm_member_path_planner configure`).

### 10.4. ROS Topic Haritası Modeli
- `[Kapsülleyen]` YAYIN Topics (Publisher):
  - `fmu/in/trajectory_setpoint`: Düşük gecikmeli XYZ Offboard Motor Hızlarını DDS üzerinden PX4'e basar.
  - `/in_position`: Servis mimarisidir. İki drone arası "Döndüm, pozisyon aldım" kilitlemesinde server sunuculuğu yapar.
- `[Kapsüllenen]` ABONELİK Topics (Subscriber):
  - `fmu/out/vehicle_local_position` veya `/global_position`: PX4 tarafından basılan İHA durumu.
  - `fmu/out/vehicle_attitude`: Quaternion olarak drone'un açılımsal yalpalama bilgileri.
  - `neighbors_info`: (Bağımsız özel arayüz) Yakınlardaki sürünün `lat/lon` kimlik haritası, kendi otonomisine veri sağlaması.

---

*Geliştirme, Bakım, Optimizasyon ve Matematik Modellemesi:*
*Bu README dosyası, **`swarm_drone_control`** sisteminin algoritma standardizasyonu amacıyla Semih'in revizeleri sonrası çekirdek dokümantasyonu olarak M. O. yapay zeka entegrasyonu ile (2026 versiyon) derlenmiştir.*
