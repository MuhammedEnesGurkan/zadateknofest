# ZADA-GCS Kod Analizi ve Failsafe Devre Dışı Bırakma Rehberi

## Kod Analizi

### Bu Kod Ne Yapıyor?

Bu Python kodu, drone'lar (İHA'lar) için geliştirilmiş kapsamlı bir **Ground Control Station (GCS)** uygulamasıdır. Temel özellikleri:

#### 🎯 Ana Özellikler:
1. **Drone Kontrolü**: MAVLink protokolü üzerinden drone bağlantısı ve kontrolü
2. **Failsafe Sistemi**: Güvenlik için otomatik failsafe mekanizması
3. **Video İşleme**: AI destekli nesne tespit sistemi (YOLO ile şişe tespiti)
4. **Harita Entegrasyonu**: Gerçek zamanlı drone konumu ve uçuş yolu takibi
5. **Mission Planlama**: CSV dosyasından waypoint yükleme ve mission çalıştırma
6. **Kamera Kontrolü**: Drone kamerası modları değiştirme

#### 🛡️ Failsafe Sistemi Detayları:
- **Batarya İzleme**: %15 altında uyarı, %8 altında acil iniş
- **Bağlantı İzleme**: 10 saniye sinyal kaybında failsafe tetikleme
- **İrtifa Kontrolü**: 200m maksimum irtifa sınırı
- **Manuel Failsafe**: Acil durum butonu ile manuel tetikleme

#### 🤖 AI Nesne Tespiti:
- YOLO modeli ile gerçek zamanlı şişe tespiti
- Tespit edilen şişelerin haritada görüntülenmesi
- GPU desteği ile hızlandırılmış işleme

---

## Failsafe Devre Dışı Bırakma Rehberi

### 🔴 PX4 Firmware için Failsafe Devre Dışı Bırakma

#### QGroundControl ile:

1. **RTL (Return to Launch) Failsafe Devre Dışı:**
```
Parameters → Safety → RTL Settings
- COM_DL_LOSS_T = 0 (Data Link Loss Timeout - sıfır yaparsa devre dışı)
- COM_RC_LOSS_T = 0 (RC Loss Timeout - sıfır yaparsa devre dışı)
- NAV_DLL_ACT = 0 (Data Link Loss Action - disabled)
- NAV_RCL_ACT = 0 (RC Loss Action - disabled)
```

2. **Batarya Failsafe Devre Dışı:**
```
Parameters → Power Management
- BAT_LOW_THR = 0.0 (Low Battery Threshold - sıfır yaparsa devre dışı)
- BAT_CRIT_THR = 0.0 (Critical Battery Threshold - sıfır yaparsa devre dışı)
- BAT_EMERGEN_THR = 0.0 (Emergency Battery Threshold - sıfır yaparsa devre dışı)
- COM_LOW_BAT_ACT = 0 (Low Battery Action - disabled)
```

3. **Geofence Devre Dışı:**
```
Parameters → Safety → Geofence
- GF_ACTION = 0 (Geofence Action - disabled)
- GF_MAX_HOR_DIST = 0 (Maximum Horizontal Distance - sıfır yaparsa sınırsız)
- GF_MAX_VER_DIST = 0 (Maximum Vertical Distance - sıfır yaparsa sınırsız)
```

#### MAVLink Komutları ile (Python):
```python
# PX4 parametrelerini ayarlama
await drone.param.set_param_float("COM_DL_LOSS_T", 0.0)
await drone.param.set_param_float("COM_RC_LOSS_T", 0.0)
await drone.param.set_param_int("NAV_DLL_ACT", 0)
await drone.param.set_param_int("NAV_RCL_ACT", 0)
await drone.param.set_param_float("BAT_LOW_THR", 0.0)
await drone.param.set_param_float("BAT_CRIT_THR", 0.0)
await drone.param.set_param_int("COM_LOW_BAT_ACT", 0)
await drone.param.set_param_int("GF_ACTION", 0)
```

### 🟢 ArduPilot Firmware için Failsafe Devre Dışı Bırakma

#### Mission Planner ile:

1. **Radio Failsafe Devre Dışı:**
```
Config/Tuning → Extended Tuning
- FS_THR_ENABLE = 0 (Throttle Failsafe disabled)
- FS_THR_VALUE = 0 (Throttle Failsafe Value)
```

2. **Battery Failsafe Devre Dışı:**
```
Config/Tuning → Extended Tuning
- BATT_FS_LOW_ACT = 0 (Low Battery Action - disabled)
- BATT_FS_CRT_ACT = 0 (Critical Battery Action - disabled)
- BATT_LOW_VOLT = 0.0 (Low Battery Voltage - sıfır yaparsa devre dışı)
- BATT_CRT_VOLT = 0.0 (Critical Battery Voltage - sıfır yaparsa devre dışı)
```

3. **GPS Failsafe Devre Dışı:**
```
Config/Tuning → Extended Tuning
- FS_GPS_ENABLE = 0 (GPS Failsafe disabled)
- GPS_HDOP_GOOD = 999 (HDOP threshold - çok yüksek değer)
```

4. **GCS Failsafe Devre Dışı:**
```
Config/Tuning → Extended Tuning
- FS_GCS_ENABLE = 0 (GCS Failsafe disabled)
```

5. **Geofence Devre Dışı:**
```
Config/Tuning → GeoFence
- FENCE_ENABLE = 0 (Fence disabled)
- FENCE_ACTION = 0 (Fence Action - disabled)
```

#### MAVLink Komutları ile (Python):
```python
# ArduPilot parametrelerini ayarlama
await drone.param.set_param_int("FS_THR_ENABLE", 0)
await drone.param.set_param_int("FS_GPS_ENABLE", 0)
await drone.param.set_param_int("FS_GCS_ENABLE", 0)
await drone.param.set_param_int("BATT_FS_LOW_ACT", 0)
await drone.param.set_param_int("BATT_FS_CRT_ACT", 0)
await drone.param.set_param_float("BATT_LOW_VOLT", 0.0)
await drone.param.set_param_float("BATT_CRT_VOLT", 0.0)
await drone.param.set_param_int("FENCE_ENABLE", 0)
await drone.param.set_param_int("FENCE_ACTION", 0)
```

---

## ⚠️ ÖNEMLİ GÜVENLİK UYARILARI

### 🚨 Failsafe Devre Dışı Bırakırken Dikkat Edilmesi Gerekenler:

1. **Test Ortamında Yapın**: İlk testleri güvenli, açık alanlarda yapın
2. **Yedek Kontrol**: Manuel kontrol her zaman hazır olsun
3. **Batarya İzleme**: Failsafe devre dışıyken bataryayı sürekli izleyin
4. **Bağlantı Kontrolü**: Sinyal gücünü sürekli kontrol edin
5. **Acil Durum Planı**: Failsafe olmadan acil durum senaryolarını planlayın

### 📋 Önerilen Güvenlik Önlemleri:

1. **Kademeli Test**: Önce sadece bir failsafe'i devre dışı bırakın
2. **Gözlemci**: Deneyimli bir pilot hazır bulunsun
3. **Yedek Kumanda**: Manuel RC kumanda her zaman aktif
4. **İrtifa Sınırı**: Düşük irtifada test yapın
5. **Hava Durumu**: Sadece ideal hava koşullarında uçuş yapın

### 🔧 Kodda Failsafe Ayarları:

Bu GCS uygulamasında failsafe ayarları `FailsafeConfig` sınıfında tanımlı:

```python
@dataclass
class FailsafeConfig:
    min_battery_level: float = 15.0      # %15 altında uyarı
    critical_battery_level: float = 8.0   # %8 altında acil iniş  
    max_heartbeat_interval: float = 10.0  # 10 saniye sinyal kayıpları
    max_altitude: float = 200.0          # Maksimum irtifa (metre)
    enable_geofence: bool = False        # Geofence zaten devre dışı
    goto_timeout: float = 30.0           # Goto komutu timeout süresi
```

Bu değerleri `False` veya çok yüksek değerler yaparak devre dışı bırakabilirsiniz.

---

## 📝 Sonuç

Bu GCS uygulaması gelişmiş failsafe özelliklerine sahip profesyonel bir drone kontrol sistemidir. Failsafe'leri devre dışı bırakırken mutlaka güvenlik önlemlerini alın ve test ortamında çalışın. Her iki firmware (PX4 ve ArduPilot) için yukarıdaki parametreleri ayarlayarak failsafe sistemlerini devre dışı bırakabilirsiniz.

**Unutmayın**: Failsafe sistemleri drone'unuzun ve çevrenin güvenliği için vardır. Sadece gerekli durumlarda ve uzman gözetiminde devre dışı bırakın!