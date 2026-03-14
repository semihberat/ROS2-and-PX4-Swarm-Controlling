# Sürü İHA (Swarm Drone Control) Kapsamlı Sistem, Mimari ve Matematiksel Modeller Dokümantasyonu

Bu belge, **ROS 2 (Robot Operating System 2)** ve **PX4 Framework (MAVLink/MAVROS ve uORB/DDS)** üzerine inşa edilmiş `swarm_drone_control` paketinin uçtan uca çalışmasını anlatan **kapsamlı mühendislik ve teorik altyapı** referansıdır.
## Demo (Screencast)

![Screencast](screencast.mp4)

Bu döküman, projenin matematiksel altyapısını, otonom seyir logaritmasını (navigation log), güvenlik katmanlarını ve sistemin asenkron çalışma prensiplerini detaylandırmaktadır. Projenin ana hedefi, bir "Sürü" (Swarm) topolojisindeki İHA'ların ortak bir uçuş planını çarpışmadan (collision avoidance), asgari ağ gecikmesiyle (minimum latency) ve WGS84 uzamsal modeline uygun bir şekilde tamamlamasıdır.

---

## İçindekiler Tablosu

- [Sürü İHA (Swarm Drone Control) Kapsamlı Sistem, Mimari ve Matematiksel Modeller Dokümantasyonu](#sürü-i̇ha-swarm-drone-control-kapsamlı-sistem-mimari-ve-matematiksel-modeller-dokümantasyonu)
  - [Demo (Screencast)](#demo-screencast)
  - [İçindekiler Tablosu](#i̇çindekiler-tablosu)
  - [1. Sisteme Genel Bakış ve Problemin Tanımı](#1-sisteme-genel-bakış-ve-problemin-tanımı)
    - [1.1. Teknoloji Yığını ve Seçim Kriterleri](#11-teknoloji-yığını-ve-seçim-kriterleri)
      - [Neden Dağıtık Sürü (Distributed Swarm) Mimarisi?](#neden-dağıtık-sürü-distributed-swarm-mimarisi)
      - [Neden ROS 2 (Robot Operating System 2)?](#neden-ros-2-robot-operating-system-2)
      - [Neden PX4 Autopilot?](#neden-px4-autopilot)
      - [Neden C++ Kullanıldı? (Python yerine)](#neden-c-kullanıldı-python-yerine)
  - [2. Sistem Mimarisi, Dizin Yapısı ve Modülerlik](#2-sistem-mimarisi-dizin-yapısı-ve-modülerlik)
    - [2.1. Dağıtık ve Merkeziyetsiz Mimari](#21-dağıtık-ve-merkeziyetsiz-mimari)
    - [2.2. Dizin Organizasyonu](#22-dizin-organizasyonu)
  - [3. ROS 2 Lifecycle ve State Machine Dinamikleri](#3-ros-2-lifecycle-ve-state-machine-dinamikleri)
    - [3.1. Düğüm Evreleri (Node Transitions)](#31-düğüm-evreleri-node-transitions)
    - [3.2. Görev-Durum (Mission State) Makinesi](#32-görev-durum-mission-state-makinesi)
  - [4. Teorik Altyapı ve Matematiksel Algoritmalar](#4-teorik-altyapı-ve-matematiksel-algoritmalar)
    - [4.1. WGS84 Coğrafi Koordinat Modelleri](#41-wgs84-coğrafi-koordinat-modelleri)
    - [4.2. Mesafe Belirleme (Haversine Formula)](#42-mesafe-belirleme-haversine-formula)
    - [4.3. Mutlak Doğrultu (Absolute Bearing) Çıkarımı](#43-mutlak-doğrultu-absolute-bearing-çıkarımı)
    - [4.4. Ağırlık Merkezi (Center of Gravity - CoG) Yaklaşımı](#44-ağırlık-merkezi-center-of-gravity---cog-yaklaşımı)
    - [4.5. Hedef Rotasyon Algoritması ve Geometrik İzdüşüm](#45-hedef-rotasyon-algoritması-ve-geometrik-i̇zdüşüm)
    - [4.6. Euler/Quaternion Dönüşüm Teorisi](#46-eulerquaternion-dönüşüm-teorisi)
  - [5. Oransal (PID) Hız Kontrolü Katmanı](#5-oransal-pid-hız-kontrolü-katmanı)
    - [5.1. Continuous Time vs Discrete Time Karşılaştırması](#51-continuous-time-vs-discrete-time-karşılaştırması)
    - [5.2. Hata (Error) Çıkarımı ve Clamping Limits](#52-hata-error-çıkarımı-ve-clamping-limits)
  - [6. Bellek Yönetimi ve `WaypointManager` Mimarisi](#6-bellek-yönetimi-ve-waypointmanager-mimarisi)
    - [6.1. Sorun: Vektör İndeks Çökmeleri (Segmentation Faults)](#61-sorun-vektör-i̇ndeks-çökmeleri-segmentation-faults)
    - [6.2. Çözüm: Bağlı Liste (Linked List) Felsefesi ve Failsafe](#62-çözüm-bağlı-liste-linked-list-felsefesi-ve-failsafe)
  - [7. Çarpışma Önleme (Collision Avoidance) ve Formasyon Koruma Mimarisi](#7-çarpışma-önleme-collision-avoidance-ve-formasyon-koruma-mimarisi)
    - [7.1. Dinamik Güvenlik Mesafesi (Speed-Adaptive Safety Distance)](#71-dinamik-güvenlik-mesafesi-speed-adaptive-safety-distance)
    - [7.2. Çok Katmanlı Kuvvet Sistemi (Multi-Layered Force Architecture)](#72-çok-katmanlı-kuvvet-sistemi-multi-layered-force-architecture)
      - [A) Formation Keeping Force (Formasyon Koruma Kuvveti)](#a-formation-keeping-force-formasyon-koruma-kuvveti)
      - [B) Distance-Based Adaptive Damping (Mesafe Bazlı Adaptif Sönümleme)](#b-distance-based-adaptive-damping-mesafe-bazlı-adaptif-sönümleme)
      - [C) Attractive Force Towards Target (Hedefe Çekim Kuvveti)](#c-attractive-force-towards-target-hedefe-çekim-kuvveti)
    - [7.3. Final Velocity Calculation (Son Hız Hesabı)](#73-final-velocity-calculation-son-hız-hesabı)
    - [7.4. Algoritmanın Avantajları](#74-algoritmanın-avantajları)
    - [7.5. Kod Referansı ve İmplementasyon](#75-kod-referansı-ve-i̇mplementasyon)
  - [8. ROS 2 QoS, DDS ve Haberleşme Modeli](#8-ros-2-qos-dds-ve-haberleşme-modeli)
  - [9. Kod İçi API ve Fonksiyon Referansları](#9-kod-i̇çi-api-ve-fonksiyon-referansları)
    - [9.1. `autonomus_utils.hpp` Çekirdek İşlevleri](#91-autonomus_utilshpp-çekirdek-i̇şlevleri)
    - [9.2. `geographic.hpp` Yardımcıları](#92-geographichpp-yardımcıları)
  - [10. Adım Adım Kurulum ve Kullanım Kılavuzu](#10-adım-adım-kurulum-ve-kullanım-kılavuzu)
    - [10.1. Gereksinimler ve Bağımlılıklar](#101-gereksinimler-ve-bağımlılıklar)
    - [10.2. Build İşlemleri](#102-build-i̇şlemleri)
    - [10.3. Çalıştırma Parametreleri](#103-çalıştırma-parametreleri)
    - [10.4. ROS Topic Haritası Modeli](#104-ros-topic-haritası-modeli)
  - [11. Gerçek Dünya Donanım ve Ağ Kılavuzu](#11-gerçek-dünya-donanım-ve-ağ-kılavuzu)
    - [11.1. Uçuş Kontrol Kartı ve Companion Computer Seçimi](#111-uçuş-kontrol-kartı-ve-companion-computer-seçimi)
    - [11.2. Donanım ve Ağ Topolojisi: Drone ve Yer İstasyonu Cihazları](#112-donanım-ve-ağ-topolojisi-drone-ve-yer-i̇stasyonu-cihazları)
      - [A) Yer Kontrol İstasyonu (GCS - Yerdeki Donanımlar)](#a-yer-kontrol-i̇stasyonu-gcs---yerdeki-donanımlar)
      - [B) Drone Üzerindeki Modüller ve Antenler (Uçan Donanımlar)](#b-drone-üzerindeki-modüller-ve-antenler-uçan-donanımlar)
      - [C) Wi-Fi Ad-Hoc / Mesh Network (Düğüm Ağı - Tam Merkeziyetsiz)](#c-wi-fi-ad-hoc--mesh-network-düğüm-ağı---tam-merkeziyetsiz)
    - [11.3. Uzun Mesafe İletişim (Long-Range Telemetry)](#113-uzun-mesafe-i̇letişim-long-range-telemetry)
    - [11.4. Donanımsal Olarak Sistem Blok Şeması](#114-donanımsal-olarak-sistem-blok-şeması)

---

## 1. Sisteme Genel Bakış ve Problemin Tanımı

Birden çok İnsansız Hava Aracının dar veya geniş alanlarda ortak bir vizyon doğrultusunda uçması (Swarm Robotics), modern havacılığın en kompleks algoritmik problemlerinden biridir. Geleneksel sistemler "Master/Slave" bağımlılığı üzerinden çalışır. Yani yerdeki veya gökyüzündeki bir merkez bilgisayar ağdaki her bir drone'a ne yapması gerektiğini dikte eder. Ancak bu tasarımın en büyük handikabı, merkezdeki ünitenin haberleşmeyi kaybetmesi veya düşmesi durumunda tüm sürünün de felç geçirmesidir.

**Çözüm:** `swarm_drone_control` paketinin otonom seyir modülü olan `SwarmMemberPathPlanner`, **merkeziyetsiz (decentralized)** bir formasyon geometrisi uygular. Her bir drone, uORB DDS (Data Distribution Service) köprüsü ile gelen `NeighborsInfo` (komşu bilgileri) topic'ini okur, formasyon içindeki kütle merkezini tespit eder ve etrafında dönerek rotasını dinamik bir vektörel tepki ile ayarlar. Böylece sistemden bir drone düşse bile diğerleri kendi ağırlık merkezlerini baştan asenkron hesaplayarak formasyonu anında onarabilir.

### 1.1. Teknoloji Yığını ve Seçim Kriterleri

Robotik sistemlerin çekirdeği (Core framework) inşa edilirken alınan teknolojik kararlar hayati önem taşır. Projede aşağıdaki spesifik teknolojiler bilerek seçilmiş ve entegre edilmiştir:

#### Neden Dağıtık Sürü (Distributed Swarm) Mimarisi?
- **Hata Toleransı (Fault Tolerance):** Merkezi bir "Master" İHA veya Yer İstasyonu kilitlenirse veya iletişim koparsa tüm sürü çökmez. 
- **Ölçeklenebilirlik (Scalability):** Lider-Takipçi (Leader-Follower) mimarisinde ağ yükü lidere binerken, dağıtık mimaride her İHA kendi matematiksel hesabını yaptığı için filoyu $3$ araçtan $50$ araca çıkartmak işlemci yükünü katlarca artırmaz.
- **Gerçek Zamanlı Otonomi:** Gerçek "Kuş veya Balık Sürüsü" matematiğinde olduğu gibi, kararlar lokal (yerel komşuluk verisiyle) alınır ancak bütünde organize, global sonuçlar üretir.

#### Neden ROS 2 (Robot Operating System 2)?
- **Uçtan Uca DDS (Data Distribution Service):** ROS 1'deki merkezi `roscore` Master düğümünün aksine, ROS 2 tamamen P2P (noktadan noktaya) UDP/DDS altyapısı sunar. Bu tam da sürünün dağıtık/merkeziyetsiz mantığıyla örtüşür.
- **QoS (Quality of Service) Profilleri:** Uçuşta sensör verilerindeki anlık kopma ve paket kayıplarında, geçmiş pakedin yeniden iletilmesini beklemeden (TCP gecikmesi) güncel pakedi direkt yakalamak (UDP - Best Effort) İHA otonomisinde dronun donmasını engeller.
- **Lifecycle Düğümleri:** ROS 2'ye özel *Lifecycle/Managed Nodes* yöntemi sayesinde her drone sensörlerinden ve algoritmalarından emin olana dek tehlikeli PX4 motor komutlarını basamaz, sistem istenildiğinde güvenle uykuya alınabilir/uyandırılabilir.

#### Neden PX4 Autopilot?
- **Araştırma ve Sürü Geliştirmesi:** PX4, hem yazılım standartı hem de donanım kartları yapısı bakımından (Pixhawk donanımları vb.) dünyada çoklu İHA (Swarm) geliştiricileri için en modifiye edilebilir, stabil Açık Kaynak platformudur.
- **Offboard (Otonom) Kontrol ve TrajectorySetpoint:** PX4 Offboard uçuş moduyla, PID üzerinden $100\mathrm{Hz}$ frekansta doğrudan motor thrust ve velocity komutları hesaplanarak (X-Y-Z kartezyen boyutlarında) mükemmel bir tepki alınır.
- **Micro-XRCE-DDS Köprüsü:** PX4, MAVROS'un getirdiği iletişim hantallığını tamamen aşarak Firmware içinden doğrudan ROS 2 DDS ağına köprü (bridge) kurabilir.

#### Neden C++ Kullanıldı? (Python yerine)
- **Gerçek Zamanlı İcra (Real-Time Performance):** Havada otonomi (özellikle $10\mathrm{\ ms}$ frekanslı `state_cycle_callback`) süreklilik arz eder. Binlerce Haversine ve trigonometrik integrali asenkron çözen bir döngüde, Python'un GIL (Global Interpreter Lock) engeli ve Garbage Collector duraksamaları sürünün havada saniyelik teklemesine (latency spike) ve kaza yapmasına yol açar.
- **Donanım ve Bellek Seviyesi Güvenliği:** Oluşturduğumuz `WaypointManager` gibi pointer manipülasyonu mantıkları bellekle birebir etkileşim zorunluluğu barındırır. C++, donanımla arada yüksek performanslı iletişim sağlarken aynı zamanda `Segmentation Fault` sızıntılarını kırmak için gerekli kilitleri kurmamıza izin verir.

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

## 7. Çarpışma Önleme (Collision Avoidance) ve Formasyon Koruma Mimarisi

Dronlar rotasyondan çıkıp `GOTO_POSITION`'da hedefe uçarken, hem çarpışmalardan kaçınmaları hem de başlangıç formasyonunu koruyarak koordineli hareket etmeleri gerekmektedir. Bu sistem, **Artificial Potential Field (Yapay Potansiyel Alan)** teorisi ile **Distance-Based Adaptive Damping (Mesafe Bazlı Adaptif Sönümleme)** yaklaşımının hibridinden oluşan sofistike bir algoritmadır.

### 7.1. Dinamik Güvenlik Mesafesi (Speed-Adaptive Safety Distance)

Geleneksel sabit güvenlik mesafesi yaklaşımı yerine, drone'un anlık hızına göre adaptif bir güvenlik bölgesi tanımlanmıştır:

$$ D_{\mathrm{safety}} = D_{\mathrm{base}} + (V_{\mathrm{current}} \times K_{\mathrm{speed}}) $$

Parametreler:
- $D_{\mathrm{base}} = 1.0 \mathrm{\ m}$: Durağan durumda minimum güvenlik mesafesi
- $K_{\mathrm{speed}} = 0.85$: Hız kazanç faktörü (metre/saniye başına eklenen mesafe)
- $V_{\mathrm{current}} = \sqrt{v_{\mathrm{lat}}^2 + v_{\mathrm{lon}}^2}$: Drone'un yatay düzlemdeki anlık hızı

**Örnek Senaryolar:**
- Drone duruyorken → $D_{\mathrm{safety}} = 1.0 \mathrm{\ m}$
- $1 \mathrm{\ m/s}$ hızda → $D_{\mathrm{safety}} = 1.85 \mathrm{\ m}$
- $2 \mathrm{\ m/s}$ hızda → $D_{\mathrm{safety}} = 2.7 \mathrm{\ m}$
- $3 \mathrm{\ m/s}$ hızda → $D_{\mathrm{safety}} = 3.55 \mathrm{\ m}$

Bu yaklaşım sayesinde yüksek hızlarda reaktif mesafe artarak güvenlik marjini korunur.

### 7.2. Çok Katmanlı Kuvvet Sistemi (Multi-Layered Force Architecture)

Sistem üç ana kuvvet bileşeninden oluşur:

#### A) Formation Keeping Force (Formasyon Koruma Kuvveti)

Başlangıçta (`FORMATIONAL_ROTATION` evresinde) kaydedilen komşu drone'larla olan mesafeler (`initial_n_distances`) referans olarak saklanır. Mission sırasında bu mesafelere yakınlık sürekli hesaplanır:

$$ \Delta d_{\mathrm{lat}}^{(i)} = d_{\mathrm{current\_lat}}^{(i)} - d_{\mathrm{initial\_lat}}^{(i)} $$

$$ \Delta d_{\mathrm{lon}}^{(i)} = d_{\mathrm{current\_lon}}^{(i)} - d_{\mathrm{initial\_lon}}^{(i)} $$

Eğer sapma **1.0 metre** eşiğini aşarsa, düzeltici kuvvet uygulanır:

$$ \mathbf{F}_{\mathrm{formation}} = \sum_{i=1}^{N_{\mathrm{neighbors}}} \begin{cases} 
\Delta d_{\mathrm{lat/lon}}^{(i)} & \text{if } |\Delta d^{(i)}| > 1.0 \mathrm{\ m} \\
0 & \text{otherwise}
\end{cases} $$

#### B) Distance-Based Adaptive Damping (Mesafe Bazlı Adaptif Sönümleme)

**Kritik Yenilik:** Formasyonun hedefe yaklaştıkça "gevşemesi" ve lokal minimum tuzağından kaçınması için mesafe bazlı bir sönümleme mekanizması geliştirilmiştir.

Hedefe olan mesafe ($d_{\mathrm{target}}$) ile hesaplanan **distance ratio**:

$$ r_{\mathrm{distance}} = \min\left(\frac{d_{\mathrm{target}}}{10.0}, 1.0\right) $$

Bu ratio'ya göre dinamik kuvvet katsayıları:

$$ K_{\mathrm{formation}} = 0.3 + (r_{\mathrm{distance}} \times 0.2) \quad \in [0.3, 0.5] $$

$$ K_{\mathrm{attraction}} = 0.5 - (r_{\mathrm{distance}} \times 0.3) \quad \in [0.2, 0.5] $$

**Davranış Profili:**

| Hedefe Mesafe | $r_{\mathrm{distance}}$ | $K_{\mathrm{formation}}$ | $K_{\mathrm{attraction}}$ | Davranış |
|---------------|-------------------------|--------------------------|---------------------------|----------|
| > 10 m        | 1.0                     | 0.5 (Güçlü)              | 0.2 (Zayıf)               | Formasyon öncelikli |
| 5-10 m        | 0.5-1.0                 | 0.4                      | 0.35                      | Dengeli mod |
| < 5 m         | < 0.5                   | 0.3 (Gevşek)             | 0.5 (Güçlü)               | Hedef öncelikli |

#### C) Attractive Force Towards Target (Hedefe Çekim Kuvveti)

Lokal minimum problemi (drone'ların formasyon kuvvetleri yüzünden hedefe varamadan "takılıp kalması") çözümü için hedefe doğru ek bir çekim kuvveti eklenmiştir:

$$ \mathbf{F}_{\mathrm{attraction}} = -K_{\mathrm{attraction}} \cdot \min\left(\frac{d_{\mathrm{target}}}{5.0}, 1.0\right) \cdot \hat{\mathbf{d}}_{\mathrm{target}} $$

Burada $\hat{\mathbf{d}}_{\mathrm{target}}$ hedefe doğru normalize edilmiş birim vektördür:

$$ \hat{\mathbf{d}}_{\mathrm{target}} = \frac{1}{d_{\mathrm{target}}} \begin{pmatrix} \Delta \mathrm{lat}_{\mathrm{meter}} \\ \Delta \mathrm{lon}_{\mathrm{meter}} \end{pmatrix} $$

**Önemli Not:** Bu kuvvet sadece `Mission::GOTO_POSITION` evresinde aktiftir. Diğer evrelerde (kalkış, rotasyon) standart formasyon koruma devam eder.

### 7.3. Final Velocity Calculation (Son Hız Hesabı)

Tüm kuvvetler toplanarak drone'un PX4'e gönderilen son hız komutu hesaplanır:

$$ V_{\mathrm{final\_lat}} = V_{\mathrm{target\_lat}} + (K_{\mathrm{formation}} \cdot F_{\mathrm{formation\_lat}}) + F_{\mathrm{attraction\_lat}} $$

$$ V_{\mathrm{final\_lon}} = V_{\mathrm{target\_lon}} + (K_{\mathrm{formation}} \cdot F_{\mathrm{formation\_lon}}) + F_{\mathrm{attraction\_lon}} $$

Sonuçlar güvenlik sınırları içinde tutulur (anti-oscillation):

$$ V_{\mathrm{final}} = \mathrm{clamp}(V_{\mathrm{final}}, \quad -1.5, \quad +1.5) \mathrm{\ m/s} $$

Bu limitler sürünün aşırı hareketlerle (oscillation/salınım) enerji kaybetmesini veya kontrolden çıkmasını engeller.

### 7.4. Algoritmanın Avantajları

1. **Lokal Minimum Çözümü:** Hedefe yaklaşırken formasyon gevşer, drone'lar hedefe ekstra çekimle ulaşır
2. **Formasyon Bütünlüğü:** Uzak mesafelerde güçlü formasyon kuvveti koordineli hareketi garanti eder
3. **Hıza Göre Adaptasyon:** Güvenlik mesafesi dinamik olduğu için farklı hız profillerinde güvenli kalır
4. **Mission-Aware Behavior:** Her mission evresine özel davranış profili
5. **Smooth Transitions:** Kuvvet geçişleri keskin değil pürüzsüz (continuous) olduğu için drone yalpalamaz

### 7.5. Kod Referansı ve İmplementasyon

Collision avoidance işleyişi `autonomus_timers.cpp` içerisindeki `collision_avoidance()` fonksiyonunda $100\mathrm{Hz}$ frekansta çalışır:

```cpp
void SwarmMemberPathPlanner::collision_avoidance()
{
    // 1. Formation keeping forces hesaplama
    for (size_t i = 0; i < current_n_distances.size(); ++i) {
        double dlat_diff = current_n_distances[i].dlat_meter - initial_n_distances[i].dlat_meter;
        // Sapma > 1.0m ise düzeltici kuvvet ekle
    }
    
    // 2. Distance-based damping (sadece GOTO_POSITION'da)
    if (current_mission == Mission::GOTO_POSITION) {
        double distance_ratio = std::min(target_distance.distance / 10.0, 1.0);
        double formation_strength = 0.3 + (distance_ratio * 0.2);
        double attraction_strength = 0.5 - (distance_ratio * 0.3);
        
        // 3. Attractive force towards target
        collision_bias.vlat -= (target_normalized * attraction_strength);
    }
    
    // 4. Smooth limiting
    collision_bias.vlat = std::clamp(collision_bias.vlat, -1.5, 1.5);
}
```

Bu işleyiş ardından `goto_position()` içerisinde hız komutlarına eklenir:

```cpp
current_commands.v_lat += collision_bias.vlat;
current_commands.v_lon += collision_bias.vlon;
```

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

## 11. Gerçek Dünya Donanım ve Ağ Kılavuzu

Yazılım mimarisi başarılı bir şekilde SITL (Simülasyon) üzerinde test edildikten sonra gerçek saha uçuşları (Real-world operations) için aşağıdaki donanım ve RF (Radyo Frekans) altyapısının kurulması hayati önem taşır. Merkeziyetsiz çalışan bu ROS 2 sürüsünde "İletişim, her şeydir."

### 11.1. Uçuş Kontrol Kartı ve Companion Computer Seçimi
Sistemin sahada uçabilmesi için İHA'ların iç donanımı ikiye ayrılır:

1. **Low-Level (FCU) Autopilot:** Sadece sensörleri okuyan (IMU, Barometre, GPS) ve motor ESC'lerine elektrik sinyali PWM basan kart. 
   - *Önerilen:* **Pixhawk 4, Pixhawk 6C / 6X veya Cube Orange+**
   - *Protokol:* Üzerlerinde PX4 Autopilot yüklü olmalı, uçuş kartındaki `TELEM 1` veya `TELEM 2` portları C++ otonomi yazılımı ile konuşmak üzere seri port (`/dev/ttyUSB0`) aracılığıyla Companion bilgisayarına aktarılmalıdır.
2. **High-Level (Companion Computer):** Sizin yazdığınız bu C++ ROS 2 Node'unu, WGS84 CoG hesaplamalarını yapan ana işlemcidir.
   - *Önerilen:* **Raspberry Pi 4 (Mod B) veya Raspberry Pi 5 / NVIDIA Jetson Nano / Xavier NX**
   - *Sebep:* `Micro-XRCE-DDS` altyapısı ve ROS 2 Humble doğrudan bu bilgisayarlara kurulur. Hem Wi-Fi çipi bulunduğu için hem de seri bağlantı hatları barındırdığı için FCU'nun doğrudan beyni olur.

### 11.2. Donanım ve Ağ Topolojisi: Drone ve Yer İstasyonu Cihazları
Dağıtık ROS 2 ağı (DDS/eProsima FastDDS) cihazların birbiriyle IP üzerinden pürüzsüz konuşabilmesini arzular. Wi-Fi bandında (2.4 GHz veya 5 GHz) uzun menzilli bir çözüm için cihazların donanım seçimi kritik düzeydedir:

#### A) Yer Kontrol İstasyonu (GCS - Yerdeki Donanımlar)
Yer İstasyonu, güçlü bir "Merkezi Access Point (AP)" etrafında tasarlanmalıdır. Standart ev modemleriyle bu operasyon yapılamaz.
- **Sektörel ve Yönlü Antenler (Base Station):** Yer istasyonunda dronların uçacağı koridora doğru bakan yüksek kazançlı ($15\mathrm{dBi} - 120^\circ$ Sector veya $24\mathrm{dBi}$ Dish) antenler seçilmelidir.
- **Marka ve Model:** **Ubiquiti Rocket M5** (veya M2) + **airMAX Sector Antenna** kombosu sürüler için endüstri standardıdır. Alternatif olarak **MikroTik BaseBox 5** kullanılabilir.
- **Ağ İletişim Hattı:** AP cihazı direğe (Tripod) asılır, "PoE (Power over Ethernet)" ile beslenerek aşağıdaki bir **Switch/Modem** portuna CAT6 kablo ile bağlanır. GCS bilgisayarınız (Ubuntu ROS 2 cihazı) doğrudan bu Switch'e kablo ile girerek devasa Wi-Fi havuzuna katılmış olur. Açık alanda bu konfigürasyon $3\mathrm{\ km} - 5\mathrm{\ km}$ arası kayıpsız ROS 2 / UDP pakedi fırlatabilir.

#### B) Drone Üzerindeki Modüller ve Antenler (Uçan Donanımlar)
Dronların kendi üzerlerindeki Raspberry Pi'nin dahili Wi-Fi çipi oldukça zayıftır ($1\mathrm{-}2\mathrm{\ dBi}$), metal/karbon fiber gövde (Faraday kafesi etkisi) altında $30\mathrm{\ metre}$ sonra sinyal kopar. Bu sorunu aşmak için:
- **Harici Wi-Fi Adaptörleri (USB):** Üzerine harici SMA anten takılabilen yüksek çekim gücüne sahip çipler (Özellikle *Atheros* chipset) gerekir. 
- **Önerilen Marka:** **Alfa Network AWUS036NHA** (2.4 GHz) veya **AWUS036ACH** (Dual Band) adaptörleri İHA kasasına sabitlenir, RPi'ye USB'den bağlanır ve cihaz `wlan1` olarak ayağa kaldırılır.
- **Drone Anten Seçimi:** Drone'daki adaptörlere standart çubuk anten yerine yönsüz (Omni-Directional) en az $5\mathrm{\ dBi}$'lik "Mushroom" (Mantar) veya "Dipole" anten takılması yere bakacak şekilde konumlandırılır. Bu sayede drone döndüğünde sinyal kör noktası oluşmaz.

#### C) Wi-Fi Ad-Hoc / Mesh Network (Düğüm Ağı - Tam Merkeziyetsiz)
Eğer devasa bir swarm yapılıyorsa ve bir baz istasyonuna bağlanmak otonomiyi zedeliyorsa "Mesh Mimarisi" kurulur. 
- **Donanım:** Yukarıda bahsedilen **Alfa Network** cihazları veya daha profesyonel askeri bant modülü olan **Doodle Labs Mesh Rider** donanımları takılır.
- **Mantık:** Ağda bir yerleşik "Modem" yoktur. $10$ numaralı İHA, $5$ numaralı İHA üzerinden sekerek $1$ numaralı İHA'ya mesaj atabilir. İHA'ların bizzat kendileri o uçan uçağın bir yönlendiricisidir (BATMAN-adv protocol). 
- *Avantaj:* Öndeki drone arkadaki drone'a repeater görevi gördüğü müddetçe sürü dünyayı turlayabilir! ROS 2 DDS'in doğasına en yatkın ağdır.

### 11.3. Uzun Mesafe İletişim (Long-Range Telemetry)
Wi-Fi mesafeleri anten Gain'ine ($\mathrm{dBi}$) bağlı olarak maksimum $1$ ile $3 \mathrm{\ km}$ civarında tutulur. Eğer sistem askeri düzeyde bir arama kurtarma sürüsü ($10\mathrm{+} \mathrm{\ km}$) olarak kullanılacaksa standart Wi-Fi çipleri sökülüp özel RF modülasyonları eklenir:
1. **RFD900X Telemetry Modelleri:** $900 \mathrm{\ MHz}$ üzerinden Point-to-Multipoint yayın yapar. Mesh altyapısı ile çoklu bağlantıyı destekler (DDS Discovery profilleri biraz kısıtlanmak/daraltılmak zorundadır).
2. **LTE / 4G-5G Modülasyonları:** Dronedaki Raspberry Pi sistemlerine takılan birer $4\mathrm{G}$ simkart modül modemi, verileri hücresel ağlar üzerinden **Husarnet VPN** veya **Tailscale** peer-to-peer tünellemeyle bağlar. DDS doğrudan mobil ağdan havada haberleşir. Sınırlar komple kalkar!

### 11.4. Donanımsal Olarak Sistem Blok Şeması

```text
[DRONE 1]                                               [DRONE 2] 
┌────────────────────────────┐                          ┌────────────────────────────┐
│ ┌────────────────────────┐ │                          │ ┌────────────────────────┐ │
│ │ Raspberry Pi 4 (ROS 2) │ │ <---(Wi-Fi UDP/DDS)--->  │ │ Raspberry Pi 4 (ROS 2) │ │
│ │ "autonomus" Node'u Çlş.│ │          (veya)          │ │ "autonomus" Node'u Çlş.│ │
│ └───────────┬────────────┘ │      <Mesh-Husarnet>     │ └───────────┬────────────┘ │
│             │ (UART/Serial)│                          │             │ (UART/Serial)│
│ ┌───────────▼────────────┐ │                          │ ┌───────────▼────────────┐ │
│ │ Pixhawk FCU (PX4 UORB) │ │                          │ │ Pixhawk FCU (PX4 UORB) │ │
│ │ RPM ve Çırpıntı İdaresi│ │                          │ │ RPM ve Çırpıntı İdaresi│ │
│ └───────────┬────────────┘ │                          │ └───────────┬────────────┘ │
│             │ (PWM Hız)    │                          │             │ (PWM Hız)    │
│            ESC             │                          │            ESC             │
└────────────────────────────┘                          └────────────────────────────┘
              ▲
              │   <---- (Örn) Bir Ubiquiti Outdoor Access Point Yönlendiricisi
```

---

*Geliştirme, Bakım, Optimizasyon ve Matematik Modellemesi:*
*Bu README dosyası, **`swarm_drone_control`** sisteminin algoritma standardizasyonu amacıyla Semih'in revizeleri sonrası çekirdek dokümantasyonu olarak M. O. yapay zeka entegrasyonu ile (2026 versiyon) derlenmiştir.*
