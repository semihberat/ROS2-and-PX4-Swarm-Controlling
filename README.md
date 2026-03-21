<div align="center">
    <h1>🦅 Sürü İHA Uçuş Kontrol ve Otonomi Yazılımı</h1>
    <h3>TEKNOFEST 2026 Sürü İHA Yarışması Kritik Tasarım Raporu (KTR) Standartlarında Yazılım Dokümantasyonu</h3>
    <br>
</div>

![Sürü İHA Mimarisi](https://img.shields.io/badge/Platform-ROS%202%20Humble-22314E?style=for-the-badge&logo=ros)
![PX4](https://img.shields.io/badge/Flight_Stack-PX4%20Autopilot-0B2C4A?style=for-the-badge&logo=px4)
![C++](https://img.shields.io/badge/Language-C%2B%2B17-00599C?style=for-the-badge&logo=c%2B%2B)
![Mimarisi](https://img.shields.io/badge/Topology-Decentralized-ff69b4?style=for-the-badge)

*Bu doküman, Teknofest 2026 Sürü İHA Yarışması isterlerine uygun olarak geliştirilmiş; Dağıtık (Decentralized) Sistem Mimarisi, Yapay Potansiyel Alanlar (APF) Tabanlı Çarpışma Önleyici ve Durum Makinesi algoritmalarının matematiksel ve mimari detaylarını içeren resmi bir yapı niteliğindedir.*

---

## 📑 İçindekiler
1. [Sistem Mimarisi ve Teknoloji Yığını](#1-sistem-mimarisi-ve-teknoloji-yığını)
2. [Sürü İHA Haberleşme Topolojisi (DDS/ROS 2)](#2-sürü-iha-haberleşme-topolojisi-ddsros-2)
3. [Matematiksel Modeller ve Kinematik Altyapı](#3-matematiksel-modeller-ve-kinematik-altyapı)
4. [Dinamik Çarpışma Önleme (Yapay Potansiyel Alanlar ve Vortex)](#4-dinamik-çarpışma-önleme-yapay-potansiyel-alanlar-ve-vortex)
5. [Lider Seçimi ve Formasyon Kontrolü](#5-lider-seçimi-ve-formasyon-kontrolü)
6. [Görev ve Durum Makinesi (State Machine)](#6-görev-ve-durum-makinesi-state-machine)
7. [Histerezis (Hysteresis) ve Hata Toleransı](#7-histerezis-hysteresis-ve-hata-toleransı)
8. [Kurulum ve Derleme (Kullanım Kılavuzu)](#8-kurulum-ve-derleme-kullanım-kılavuzu)

---

## 1. Sistem Mimarisi ve Teknoloji Yığını
Projede, sürü İHA'ların gerçek zamanlı tepki verebilmesi ve tam otonom şekilde uçuş görevlerini (kalkış, formasyonel rotasyon, hedef takibi) yerine getirebilmesi için modüler, genişletilebilir ve deterministik bir yapı tercih edilmiştir.

* **ROS 2 (Humble Hawksbill):** Haberleşme güvenilirliği (QoS) ve asenkron/dağıtık düğüm (node) mimarisini sağlamak amacıyla omurga olarak kullanılmıştır.
* **PX4 Autopilot & uORB:** Alt katmanda (Low-level) motor kontrolcülerinin sürülmesi ve sensör verilerinin (EKF2) harmanlanması için standartlaştırılmış uçuş yığını.
* **C++17:** Gerçek zamanlı (Real-Time) sistemlerin bellek yönetimi (Pointer allocation, Heap clustering) gereksinimlerini en optimum ve en az işlemci yükü (Low CPU footprint) ile karşılamak için kullanılmıştır.

---

## 2. Sürü İHA Haberleşme Topolojisi (DDS/ROS 2)
Teknofest senaryolarında, havada ağ kopması ve paket kayıpları beklenen koşullardır. Mimari, **Merkeziyetsiz (Decentralized)** formda tasarlanmış olup, yer kontrol istasyonuna bağımlı olmadan sürü içi iletişimi DDS (Data Distribution Service) standardıyla çözümlemektedir:

* **Sürü İçi Ağ:** İHA'lar, kendi konumsal verilerini ve State bilgilerini multicast tabanlı DDS haberleşmesiyle tüm ağa saniyede $10 \mathrm{Hz}$ hızında yayınlar (`NeighborsInfo`, `InTarget`).
* **MicroRTPS / MicroXRCE-DDS Bridge:** ROS 2'nin üst katman hız komutları (V_lat, V_lon, Yaw), PX4'ün eyleyicilerine milisaniyelik gecikmelerle doğrudan köprülenir.

---

## 3. Matematiksel Modeller ve Kinematik Altyapı
Dünya düzlemi üzerinde $X-Y$ bağıl koordinat kullanmak yerine WGS84 Coğrafi koordinat (Geographic WGS84) sistemi baz alınmıştır. Bu yaklaşım, sistemin dünyanın herhangi bir enlem/boylam düzleminde çalışmasına olanak tanır.

### Haversine Formülü ile Uzamsal Dönüşüm
İki İHA arasındaki coğrafi uzaklık metrik ($d_{\mathrm{lat}}, d_{\mathrm{lon}}$) düzleme yansıtılırken küresel sapmalar dikkate alınmıştır:

$$ a = \sin^2\left(\frac{\Delta\varphi}{2}\right) + \cos(\varphi_1)\cdot\cos(\varphi_2)\cdot\sin^2\left(\frac{\Delta\lambda}{2}\right) $$
$$ c = 2 \cdot \operatorname{atan2}\left(\sqrt{a}, \sqrt{1-a}\right) $$
$$ d = R \cdot c $$

$R = 6371000 \mathrm{\ m}$ (Dünya Yarıçapı). C++ katmanında `geo::calculate_distance` ile optimize edilmiştir.

### ArcTangent Tabanlı Doğrultu (Bearing)
Drone'un ilerlemesi gereken mutlak Yönelim (Yaw) açısı, kuzey referansına göre sürekli hesaplanıp `WrapAngleToPi` $\left(-\pi \leq \theta \leq +\pi \right)$ ile limitlere çekilmektedir.

---

## 4. Dinamik Çarpışma Önleme (Yapay Potansiyel Alanlar ve Vortex)
Geleneksel sürü kontrol sistemlerinde sıklıkla görülen "sert çarpışmalar" ve "kilitlenme (deadlock)" problemleri, literatüre uygun **Yapay Potansiyel Alanlar (APF)** ve **Girdap (Vortex) Alanları** entegrasyonu ile çözülmüştür.

Sistem `autonomous_col_avoid.cpp` modülü üzerinden çalışır:

**1. Hıza Duyarlı Güvenlik Yarıçapı:**
Drone hızlı uçarken durma mesafesi artacağından, etki alanı da dinamik olarak genişler:
$$ D_{\mathrm{safe}} = D_{\mathrm{min\_safe}} + \left(|V_{\mathrm{current}}| \times K_{\mathrm{speed\_factor}}\right) $$
*Burada $K_{\mathrm{speed\_factor}} = 0.8$, $D_{\mathrm{min\_safe}} = 1.5$ olarak yapılandırılmıştır.*

**2. Karesel İtme Kuvveti (Repulsive Force):**
Girişim (penetrasyon) miktarının karesi ile orantılı artan bir şok önleme algoritması uygulanmıştır. İHA ile komşusu arasındaki vektöre göre geriye doğru fırlatma kuvveti entegre edilmiştir.
$$ F_{\mathrm{repel}} = -K_{\mathrm{repulsive}} \times \left( \frac{(D_{\mathrm{safe}} - D_{\mathrm{current}})^2}{D_{\mathrm{safe}}} \right) $$

**3. Anti-Deadlock Girdap (Vortex) Kuvveti:**
Özellikle **Formasyonel Rotasyon** sırasında araçların tam kafa kafaya gelip ($\Delta x \approx 0, \Delta y \approx 0$ kuvvet denkleşmesi) sistemin felç olması engellenmiştir. Komşuya ait doğrultu vektörünün 90° dik eksenine teğetsel bir $K_{\mathrm{vortex}}$ kuvveti ile İHA'ların sağdan manevrayla sıyrılmaları sağlanır:
$$ \vec{V}_{\mathrm{vortex}} = \langle -dir_{\mathrm{lon}}, dir_{\mathrm{lat}} \rangle \times K_{\mathrm{vortex}} \times \text{Penetrasyon} $$

Sonuç olarak tüm bias (sapma) verileri bir **Low-Pass Filter (Düşük Geçiren Filtre, $\alpha=0.5$)** katmanından geçirilerek ani ivmelenmeler/titreşimler (jitters) önlenmiştir.

---

## 5. Lider Seçimi ve Formasyon Kontrolü
Ağ üzerinde önceden programlanmış bir "Statik Lider" yoktur. Teknofest görev koşulları gereği liderin düşmesi ya da ağdan kopması senaryolarına karşı **Otonom Hedefe-Yakınlık Minimizasyonu** kullanılarak dinamik re-election (yeniden atama) tetiklenir (`elect_leader()` metodu).

1. Tüm İHA konumları toplanarak bir **CoG (Center of Gravity - Ağırlık Merkezi)** küresel lokasyonu tespit edilir.
2. Formasyon rotasyonunda Lider, Waypoint'e (Hedefe) dik açıyı referans alır; tüm diğer İHA'lar CoG etrafında statik göreceli açılarıyla ($\Delta \theta_{\mathrm{offset}}$) yörüngelerine yerleşirler.

---

## 6. Görev ve Durum Makinesi (State Machine)
Sistemin beyni, 100 milisaniyede (10Hz) bir koşan asenkron `state_cycle_callback` algoritmasına dayanır. Giyilebilir teknolojinin temel evreleri aşağıdaki gibidir:

* `FORMATIONAL_TAKEOFF`: Uçuş İzinlerinin alınması, Senkronize irtifa kazanımı.
* `FORMATIONAL_ROTATION`: İHA'ların hedef noktaya yönelmeleri, Yaw hizalamaları ve "Go-To" öncesi formasyon dizilimi (`ALIGN_YAW`).
* `GOTO_POSITION`: Ortak ulaşılan Waypoint rotasına çarpışma önleyici açık şekilde sürünün akması.
* `DO_PROCESS`: Waypoint üzerinde görev yapılması (QR okuma, Tarama, Faydalı Yük Bırakma vb.).
* `END_TASK`: İniş ve görev bitimi.

---

## 7. Histerezis (Hysteresis) ve Hata Toleransı
Sensör gürültüleri (Wind, GPS Glitch) nedeniyle İHA'ların hedefte "var-yok" şeklinde hızlı bool geçişleri yapması (Ping-Pong / Oscillation Effect) sistemi çökertebilir. 

`autonomus_missions.cpp` içerisindeki `verify_in_target_state` fonksiyonu, $0.25 \mathrm{m}$ giriş toleransı ve $0.75 \mathrm{m}$ ($\times 3$) çıkış tolerans eşik değerleriyle **Dual-Threshold Hysteresis** kullanmaktadır. Bu, ağ genelinde "Bölgeye Ulaşıldı" komutlarının mutlak stabilitesini garanti eder. Sadece bütün sistem `check_all_drones_in_target()` onayı verdiğinde `next_step()` tetiklenir.

---

## 8. Kurulum ve Derleme (Kullanım Kılavuzu)

### Gereksinimler
- Ubuntu 22.04 LTS
- ROS 2 Humble
- PX4 Autopilot v1.14+ (Micro XRCE-DDS aktif)
- Özel mesaj paketlerini içeren `custom_interfaces` eklentisi.

### Ortam Derleme
```bash
cd ~/ws_offboard_control
colcon build --packages-select custom_interfaces
source install/setup.bash
colcon build --packages-select swarm_drone_control
source install/setup.bash
```

### Başlatma Betiği
Sistemi tek bir komut ile SİT (Software In The Loop) ortamında çoklu araç topolojisine geçirmek için:
```bash
./src/swarm_drone_control/start_multi_drones.sh
```

---
*Geliştirilmiş ve test edilmiş bu altyapı algoritması, sürü formasyonunun fiziksel limitlerini optimize eden güvenli, hızlı ve Teknofest donanım şablonuyla doğrudan senkron çalışabilir haldedir. Başarılar dileriz.*
