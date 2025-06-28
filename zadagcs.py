import customtkinter as ctk
from mavsdk import System
import asyncio
import math
import tkintermapview
import threading
import csv
from mavsdk.mission import MissionItem, MissionPlan
import tkinter.filedialog as filedialog
import tkinter.messagebox as messagebox
import os
import serial.tools.list_ports
from mavsdk.camera import (CameraError, Mode)
import tkinter
import cv2
from PIL import Image, ImageTk
import torch
from ultralytics import YOLO
import time
import queue
import logging
from dataclasses import dataclass
from typing import Optional
from datetime import datetime

# Logging konfigürasyonu
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class LogHandler(logging.Handler):
    """Custom log handler GUI'de log göstermek için"""

    def __init__(self, log_callback):
        super().__init__()
        self.log_callback = log_callback

    def emit(self, record):
        log_message = self.format(record)
        if self.log_callback:
            self.log_callback(log_message)


@dataclass
class DroneState:
    """Drone durumunu takip eden sınıf"""
    is_connected: bool = False
    is_armed: bool = False
    battery_level: float = 100.0
    altitude: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0
    is_failsafe_active: bool = False
    last_heartbeat: float = 0.0


@dataclass
class FailsafeConfig:
    """Failsafe konfigürasyonu - Daha esnek ayarlar"""
    min_battery_level: float = 15.0  # %15 altında uyarı
    critical_battery_level: float = 8.0  # %8 altında acil iniş
    max_heartbeat_interval: float = 10.0  # 10 saniye sinyal kayıpları
    max_altitude: float = 200.0  # Maksimum irtifa (metre) - artırıldı
    enable_geofence: bool = False  # Geofence devre dışı
    goto_timeout: float = 30.0  # Goto komutu timeout süresi


class FailsafeManager:
    """Failsafe işlemlerini yöneten sınıf"""

    def __init__(self, drone_system, gui_callback=None):
        self.drone = drone_system
        self.gui_callback = gui_callback
        self.config = FailsafeConfig()
        self.drone_state = DroneState()
        self.is_monitoring = False
        self.goto_active = False  # Goto komutu aktif mi
        self.goto_start_time = 0  # Goto başlangıç zamanı

    async def start_monitoring(self):
        """Failsafe izlemeyi başlat"""
        self.is_monitoring = True
        logger.info("Failsafe monitoring başlatıldı")

        # Paralel monitoring görevleri
        await asyncio.gather(
            self._monitor_battery(),
            self._monitor_connection(),
            self._monitor_altitude(),
            return_exceptions=True
        )

    def set_goto_active(self, active: bool):
        """Goto komutunun aktif olduğunu belirt"""
        self.goto_active = active
        if active:
            self.goto_start_time = time.time()
            logger.info("Goto komutu aktif - Failsafe esnek modda")
        else:
            logger.info("Goto komutu tamamlandı - Failsafe normal modda")

    async def _monitor_battery(self):
        """Batarya seviyesini izle"""
        while self.is_monitoring:
            try:
                battery_info = await self.drone.telemetry.battery().__anext__()
                self.drone_state.battery_level = battery_info.remaining_percent

                # Goto aktifken daha esnek kontrol
                critical_level = self.config.critical_battery_level
                if self.goto_active:
                    critical_level = max(5.0, self.config.critical_battery_level - 3.0)

                if self.drone_state.battery_level <= critical_level:
                    if not self.goto_active:
                        await self._trigger_emergency_landing("Kritik batarya seviyesi!")
                    else:
                        await self._trigger_warning(
                            f"Düşük batarya (Goto aktif): %{self.drone_state.battery_level:.1f}")
                elif self.drone_state.battery_level <= self.config.min_battery_level:
                    await self._trigger_warning(f"Düşük batarya: %{self.drone_state.battery_level:.1f}")

                await asyncio.sleep(2)
            except Exception as e:
                logger.error(f"Batarya monitoring hatası: {e}")
                await asyncio.sleep(5)

    async def _monitor_connection(self):
        """Bağlantı durumunu izle"""
        while self.is_monitoring:
            try:
                self.drone_state.last_heartbeat = time.time()

                # Bağlantı durumunu kontrol et
                async for state in self.drone.core.connection_state():
                    if not state.is_connected and not self.goto_active:
                        await self._trigger_failsafe("Bağlantı kaybı!")
                    elif not state.is_connected and self.goto_active:
                        await self._trigger_warning("Bağlantı kaybı (Goto aktif)")
                    break

                await asyncio.sleep(1)
            except Exception as e:
                logger.error(f"Bağlantı monitoring hatası: {e}")
                await asyncio.sleep(3)

    async def _monitor_altitude(self):
        """İrtifa sınırlarını izle"""
        while self.is_monitoring:
            try:
                position = await self.drone.telemetry.position().__anext__()
                self.drone_state.altitude = position.relative_altitude_m
                self.drone_state.latitude = position.latitude_deg
                self.drone_state.longitude = position.longitude_deg

                # Goto aktifken daha yüksek limit
                max_alt = self.config.max_altitude
                if self.goto_active:
                    max_alt = self.config.max_altitude + 50  # 50m ek tolerans

                if self.drone_state.altitude > max_alt:
                    await self._trigger_warning(f"Maksimum irtifa aşıldı: {self.drone_state.altitude:.1f}m")

                # Goto timeout kontrolü
                if self.goto_active and (time.time() - self.goto_start_time) > self.config.goto_timeout:
                    logger.warning("Goto timeout - Normal monitoring'e dönülüyor")
                    self.goto_active = False

                await asyncio.sleep(1)
            except Exception as e:
                logger.error(f"İrtifa monitoring hatası: {e}")
                await asyncio.sleep(3)

    async def _trigger_failsafe(self, reason: str):
        """Failsafe'i tetikle"""
        if self.drone_state.is_failsafe_active:
            return

        self.drone_state.is_failsafe_active = True
        logger.critical(f"FAILSAFE TETİKLENDİ: {reason}")

        if self.gui_callback:
            self.gui_callback(f"FAILSAFE: {reason}")

        try:
            # RTL (Return to Launch) komutunu ver
            await self.drone.action.return_to_launch()
            logger.info("RTL komutu gönderildi")
        except Exception as e:
            logger.error(f"RTL komutu hatası: {e}")
            # RTL başarısızsa acil iniş yap
            await self._trigger_emergency_landing("RTL başarısız, acil iniş!")

    async def _trigger_emergency_landing(self, reason: str):
        """Acil iniş tetikle"""
        logger.critical(f"ACİL İNİŞ: {reason}")

        if self.gui_callback:
            self.gui_callback(f"ACİL İNİŞ: {reason}")

        try:
            await self.drone.action.land()
            logger.info("Acil iniş komutu gönderildi")
        except Exception as e:
            logger.error(f"Acil iniş komutu hatası: {e}")

    async def _trigger_warning(self, message: str):
        """Uyarı mesajı gönder"""
        logger.warning(message)
        if self.gui_callback:
            self.gui_callback(f"UYARI: {message}")

    async def manual_failsafe(self):
        """Manuel failsafe tetikleme"""
        await self._trigger_failsafe("Manuel failsafe tetiklendi")

    def stop_monitoring(self):
        """İzlemeyi durdur"""
        self.is_monitoring = False
        logger.info("Failsafe monitoring durduruldu")


class VideoProcessor:
    """Video işleme sınıfı"""

    def __init__(self, model_path: str, detection_callback=None):
        self.model_path = model_path
        self.model = None
        self.is_processing = False
        self.input_queue = queue.Queue(maxsize=5)
        self.output_queue = queue.Queue(maxsize=5)
        self.detection_callback = detection_callback
        self.last_bottle_detection_time = 0
        self.detection_cooldown = 5.0
        self._load_model()

    def _load_model(self):
        """YOLO modelini yükle"""
        try:
            if os.path.exists(self.model_path):
                self.model = YOLO(self.model_path)
                if torch.cuda.is_available():
                    self.model = self.model.cuda()
                    logger.info(f"GPU Kullanılıyor: {torch.cuda.get_device_name(0)}")
                else:
                    logger.info("GPU bulunamadı, CPU kullanılıyor.")
            else:
                logger.warning(f"Model dosyası bulunamadı: {self.model_path}")
        except Exception as e:
            logger.error(f"Model yükleme hatası: {e}")

    def start_processing(self):
        """Video işleme başlat"""
        if not self.model:
            logger.error("Model yüklenmemiş, video işleme başlatılamıyor")
            return

        self.is_processing = True
        processing_thread = threading.Thread(target=self._process_frames, daemon=True)
        processing_thread.start()
        logger.info("Video işleme başlatıldı")

    def _process_frames(self):
        """Frame işleme döngüsü"""
        threshold = 0.7
        class_colors = {0: (0, 255, 0), 1: (255, 0, 0), 2: (0, 0, 255)}
        class_names = {0: "Bottle", 1: "Box", 2: "Plastic"}

        while self.is_processing:
            try:
                if not self.input_queue.empty():
                    frame = self.input_queue.get()
                    start_time = time.time()

                    results = self.model(frame, imgsz=320, verbose=False)[0]
                    processed_frame = frame.copy()
                    object_counts = {0: 0, 1: 0, 2: 0}

                    bottle_detected = False

                    for result in results.boxes.data.tolist():
                        x1, y1, x2, y2, score, class_id = result

                        if score > threshold and int(class_id) in class_colors:
                            class_id = int(class_id)
                            object_counts[class_id] += 1

                            if class_id == 0:
                                bottle_detected = True

                            color = class_colors[class_id]
                            cv2.rectangle(processed_frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                            cv2.putText(processed_frame, f"{class_names[class_id]} {score:.2f}",
                                        (int(x1), int(y1) - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                    if bottle_detected and self.detection_callback:
                        current_time = time.time()
                        if current_time - self.last_bottle_detection_time > self.detection_cooldown:
                            self.detection_callback("bottle", object_counts[0])
                            self.last_bottle_detection_time = current_time

                    y_offset = 30
                    for class_id, count in object_counts.items():
                        cv2.putText(processed_frame, f"{class_names[class_id]}: {count}",
                                    (10, y_offset),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, class_colors[class_id], 2, cv2.LINE_AA)
                        y_offset += 30

                    processing_time = (time.time() - start_time) * 1000

                    if self.output_queue.full():
                        self.output_queue.get()
                    self.output_queue.put(processed_frame)
                else:
                    time.sleep(0.01)
            except Exception as e:
                logger.error(f"Frame işleme hatası: {e}")
                time.sleep(0.1)

    def stop_processing(self):
        """Video işlemeyi durdur"""
        self.is_processing = False


class DroneGCS:
    """Ana GCS sınıfı"""

    def __init__(self):
        # Global değişkenler
        # Şişe tespit sistemi için yeni değişkenler - BURAYA EKLEYİN
        self.bottle_detections = []  # Tespit edilen şişe konumları
        self.bottle_markers = []  # Haritadaki şişe markerları
        self.drone = None
        self.failsafe_manager = None
        self.video_processor = VideoProcessor('/home/meg/best.pt', detection_callback=self._on_object_detected)
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        # GUI elemanları
        self.root = None
        self.info_label = None
        self.status_label = None
        self.log_text = None
        self.drone_marker = None
        self.path_line = None
        self.video_canvas = None
        self.cap = None
        self.map_widget = None

        # Drone durumu
        self.current_lat = 45.0
        self.current_lon = 37.5
        self.flight_path = []
        self.selected_port = None

        # Log mesajlarını saklamak için
        self.log_messages = []
        self.max_log_messages = 50

        # Event loop'u başlat
        self._start_asyncio_loop()

        # Custom log handler ekle
        log_handler = LogHandler(self._add_log_message)
        log_handler.setLevel(logging.INFO)
        logger.addHandler(log_handler)

    def _start_asyncio_loop(self):
        """Asenkron döngüyü başlat"""
        t = threading.Thread(target=self._run_asyncio_loop, daemon=True)
        t.start()

    def _run_asyncio_loop(self):
        """Asenkron döngüyü çalıştır"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _add_log_message(self, message: str):
        """Log mesajı ekle"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"

        self.log_messages.append(formatted_message)

        if len(self.log_messages) > self.max_log_messages:
            self.log_messages.pop(0)

        if self.log_text:
            self.root.after(0, self._update_log_display)

    def _update_log_display(self):
        """Log görüntüsünü güncelle"""
        if self.log_text:
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            for message in self.log_messages[-10:]:
                self.log_text.insert("end", message + "\n")
            self.log_text.configure(state="disabled")
            self.log_text.see("end")

    def _add_bottle_marker_to_map(self, lat: float, lon: float, count: int, timestamp: str):
        """Haritaya şişe markeri ekle - YENİ FONKSİYON"""
        try:
            if hasattr(self, 'map_widget') and self.map_widget:
                # Şişe markeri oluştur - farklı renk ve simge kullan
                bottle_marker = self.map_widget.set_marker(
                    lat,
                    lon,
                    text=f"🍼 Şişe x{count}\n{timestamp}",
                    marker_color_circle="blue",  # Mavi renk (drone kırmızı)
                    marker_color_outside="darkblue",  # Koyu mavi dış renk
                    font=("Arial", 8, "bold")
                )

                # Markeri listeye ekle (sonradan silebilmek için)
                self.bottle_markers.append({
                    'marker': bottle_marker,
                    'lat': lat,
                    'lon': lon,
                    'count': count,
                    'timestamp': timestamp
                })

                logger.info(f"Şişe markeri haritaya eklendi: ({lat:.6f}, {lon:.6f})")

        except Exception as e:
            logger.error(f"Şişe markeri ekleme hatası: {e}")

    def clear_bottle_markers(self):
        """Haritadaki tüm şişe markerlarını temizle - YENİ FONKSİYON"""
        try:
            for bottle_marker_info in self.bottle_markers:
                bottle_marker_info['marker'].delete()

            self.bottle_markers.clear()
            self.bottle_detections.clear()
            logger.info("Tüm şişe markerleri temizlendi")
            self._update_status_label("Şişe markerleri temizlendi")

        except Exception as e:
            logger.error(f"Şişe markerleri temizleme hatası: {e}")

    def show_bottle_detections(self):
        """Tespit edilen şişelerin listesini göster - YENİ FONKSİYON"""
        if not self.bottle_detections:
            messagebox.showinfo("Şişe Tespitleri", "Henüz şişe tespiti yapılmamış.")
            return

        # Yeni pencere oluştur
        detection_window = ctk.CTkToplevel(self.root)
        detection_window.title("Şişe Tespit Listesi")
        detection_window.geometry("700x450")

        # Başlık
        title_label = ctk.CTkLabel(detection_window, text="🍼 Tespit Edilen Şişeler",
                                   font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        # Tablo başlığı
        header_frame = ctk.CTkFrame(detection_window)
        header_frame.pack(fill="x", padx=10, pady=5)

        header_text = "Zaman     | Enlem      | Boylam     | İrtifa  | Adet"
        header_label = ctk.CTkLabel(header_frame, text=header_text, font=("Courier", 12, "bold"))
        header_label.pack(pady=5)

        # Scrollable frame için container
        scrollable_frame = ctk.CTkScrollableFrame(detection_window)
        scrollable_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Her tespit için satır oluştur
        for i, detection in enumerate(self.bottle_detections):
            timestamp = detection['timestamp'].strftime("%H:%M:%S")
            text = f"{timestamp} | {detection['lat']:.6f} | {detection['lon']:.6f} | {detection['altitude']:.1f}m | {detection['count']}"

            row_frame = ctk.CTkFrame(scrollable_frame)
            row_frame.pack(fill="x", pady=1)

            row_label = ctk.CTkLabel(row_frame, text=text, font=("Courier", 10))
            row_label.pack(anchor="w", padx=5, pady=2)

        # Kapatma butonu
        close_btn = ctk.CTkButton(detection_window, text="Kapat", command=detection_window.destroy)
        close_btn.pack(pady=10)

    def _on_object_detected(self, object_type: str, count: int):
        """Nesne tespit edildiğinde çağrılan callback fonksiyonu - ŞİŞE MARKERİ EKLENDİ"""
        try:
            if object_type == "bottle":
                lat = self.current_lat
                lon = self.current_lon

                altitude = 0.0
                if self.failsafe_manager and self.failsafe_manager.drone_state:
                    altitude = self.failsafe_manager.drone_state.altitude

                # Şişe tespit bilgilerini sakla
                detection_info = {
                    'lat': lat,
                    'lon': lon,
                    'altitude': altitude,
                    'count': count,
                    'timestamp': datetime.now()
                }
                self.bottle_detections.append(detection_info)

                location_info = f"Enlem: {lat:.6f}, Boylam: {lon:.6f}, İrtifa: {altitude:.2f}m"
                timestamp = datetime.now().strftime("%H:%M:%S")
                detection_message = f"🍼 ŞİŞE TESPİT EDİLDİ! [{timestamp}] Adet: {count} | Konum: {location_info}"

                logger.info(detection_message)
                self._update_status_label(f"Şişe tespit edildi! Adet: {count}")

                # Haritaya şişe markeri ekle ve mesaj kutusunu göster
                if self.root:
                    self.root.after(0, lambda: self._add_bottle_marker_to_map(lat, lon, count, timestamp))
                    self.root.after(0, lambda: self._show_detection_messagebox(count, location_info, timestamp))

        except Exception as e:
            logger.error(f"Nesne tespit callback hatası: {e}")

    def _show_detection_messagebox(self, count: int, location_info: str, timestamp: str):
        """Tespit mesaj kutusunu göster"""
        try:
            message = f"🍼 Şişe tespit edildi!\n\nZaman: {timestamp}\nAdet: {count}\nKonum: {location_info}"
            messagebox.showinfo("Nesne Tespiti", message)
        except Exception as e:
            logger.error(f"Mesaj kutusu gösterme hatası: {e}")

    def _failsafe_callback(self, message: str):
        """Failsafe mesajlarını GUI'de göster"""
        if self.status_label:
            self.status_label.after(0, lambda: self._update_status_label(message))
        messagebox.showwarning("Failsafe Uyarı", message)

    def _update_status_label(self, message: str):
        """Status label'ı güncelle"""
        if self.status_label:
            self.status_label.configure(text=message)
            self.root.after(5000, lambda: self.status_label.configure(text="Hazır"))

    def _update_map(self):
        """Harita üzerindeki drone konumu ve path'i güncelle"""
        try:
            if hasattr(self, 'map_widget') and self.map_widget:
                # Drone marker'ını güncelle
                if self.drone_marker:
                    self.drone_marker.delete()

                self.drone_marker = self.map_widget.set_marker(
                    self.current_lat,
                    self.current_lon,
                    text="🚁 Drone",
                    marker_color_circle="red",
                    marker_color_outside="darkred"
                )

                # Path çizgisini güncelle
                if len(self.flight_path) > 1:
                    # Mevcut path line'ı sil
                    if hasattr(self, 'path_line') and self.path_line:
                        self.path_line.delete()

                    # Yeni path line çiz
                    self.path_line = self.map_widget.set_path(
                        self.flight_path,
                        color="blue",
                        width=3
                    )

                # Harita merkezini drone konumuna odakla
                self.map_widget.set_position(self.current_lat, self.current_lon)

                logger.debug(
                    f"Harita güncellendi - Drone: ({self.current_lat:.6f}, {self.current_lon:.6f}), Path noktaları: {len(self.flight_path)}")

        except Exception as e:
            logger.error(f"Harita güncelleme hatası: {e}")

    def list_ports(self):
        """Mevcut portları listele"""
        try:
            ports = serial.tools.list_ports.comports()
            port_list = []
            for port in ports:
                logger.info(f"Port: {port.device} - {port.description}")
                port_list.append(port.device)
            return port_list
        except Exception as e:
            logger.error(f"Port listeleme hatası: {e}")
            return []

    def on_port_selected(self, choice):
        """Port seçim callback"""
        self.selected_port = choice
        logger.info(f"Seçilen port: {self.selected_port}")

    async def init_drone(self):
        """Drone bağlantısını başlat"""
        try:
            if not self.selected_port:
                messagebox.showerror("Hata", "Lütfen önce bir port seçin!")
                return

            self.drone = System()

            if "ttyUSB" in self.selected_port or "ttyACM" in self.selected_port:
                connection_str = f"serial://{self.selected_port}:115200"
            else:
                connection_str = "udp://:14540"

            logger.info(f"Bağlantı adresi: {connection_str}")
            await self.drone.connect(system_address=connection_str)

            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    logger.info("Drone bağlantısı başarılı!")

                    self.failsafe_manager = FailsafeManager(self.drone, self._failsafe_callback)
                    asyncio.run_coroutine_threadsafe(self.failsafe_manager.start_monitoring(), self.loop)

                    asyncio.run_coroutine_threadsafe(self._update_telemetry(), self.loop)
                    break
        except Exception as e:
            logger.error(f"Drone bağlantı hatası: {e}")
            messagebox.showerror("Bağlantı Hatası", f"Drone'a bağlanılamadı: {e}")

    async def _update_telemetry(self):
        """Telemetri verilerini güncelle"""
        while True:
            try:
                if not self.drone:
                    await asyncio.sleep(1)
                    continue

                # Pozisyon bilgisi
                pos = await self.drone.telemetry.position().__anext__()
                self.current_lat = pos.latitude_deg
                self.current_lon = pos.longitude_deg
                alt = pos.relative_altitude_m

                # Uçuş yolunu güncelle
                if self.flight_path and (self.current_lat, self.current_lon) != self.flight_path[-1]:
                    # Minimum mesafe kontrolü (çok sık güncellemeyi önle)
                    last_lat, last_lon = self.flight_path[-1]
                    distance = math.sqrt((self.current_lat - last_lat) ** 2 + (self.current_lon - last_lon) ** 2)
                    if distance > 0.00001:  # ~1 metre
                        self.flight_path.append((self.current_lat, self.current_lon))
                elif not self.flight_path:
                    self.flight_path.append((self.current_lat, self.current_lon))

                # Batarya bilgisi
                battery = await self.drone.telemetry.battery().__anext__()
                batarya = battery.remaining_percent

                # Kamera bilgisi
                camera_info = "Bağlantı yok"
                try:
                    if self.drone and hasattr(self.drone, 'camera'):
                        camera_mode = await self.drone.camera.get_mode(1)
                        camera_info = "FOTOĞRAF modu" if camera_mode == Mode.PHOTO else "VİDEO modu"
                except Exception:
                    camera_info = "ERİŞİLEMİYOR"

                # GUI güncelle
                if self.info_label:
                    info_text = (f"Lat: {self.current_lat:.6f}\n"
                                 f"Lon: {self.current_lon:.6f}\n"
                                 f"Alt: {alt:.2f} m\n"
                                 f"Batarya: {batarya:.1f}%\n"
                                 f"Kamera: {camera_info}\n"
                                 f"Path: {len(self.flight_path)} nokta")

                    self.info_label.after(0, lambda: self.info_label.configure(text=info_text))

                # Harita güncellemesini tetikle
                if hasattr(self, 'map_widget') and self.map_widget:
                    self.root.after(0, self._update_map)

                await asyncio.sleep(1)
            except Exception as e:
                logger.error(f"Telemetri güncelleme hatası: {e}")
                await asyncio.sleep(2)

    def manual_failsafe(self):
        """Manuel failsafe tetikleme"""
        if self.failsafe_manager:
            result = messagebox.askyesno("Failsafe Onayı",
                                         "Manuel failsafe tetiklemek istediğinizden emin misiniz?\n"
                                         "Bu işlem drone'u eve döndürecek!")
            if result:
                asyncio.run_coroutine_threadsafe(self.failsafe_manager.manual_failsafe(), self.loop)
                logger.warning("Manuel failsafe tetiklendi!")
        else:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")

    async def check_arm_status(self):
        """Drone arm durumunu kontrol et"""
        if not self.drone:
            return False

        try:
            # Tek seferlik kontrol
            armed_stream = self.drone.telemetry.armed()
            armed = await armed_stream.__anext__()
            return armed
        except Exception as e:
            logger.error(f"Arm kontrol hatası: {e}")
            return False


    def arm_drone(self):
        """Drone'u arm et"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def arm():
            is_armed = await self.check_arm_status()
            try:
                if is_armed:
                    messagebox.showinfo("Bilgi", "Drone zaten arm edilmiş!")
                    return
                logger.info("Drone arm ediliyor...")
                await self.drone.action.arm()
                is_armed = await self.check_arm_status()
                if is_armed:
                    logger.info("Drone arm edildi!")
                    self._update_status_label("Drone arm edildi")
                else:
                    logger.info("Drone arm edilemedi!")
                    self._update_status_label("Drone arm edilemedi")
            except Exception as e:
                logger.error(f"Arm hatası: {e}")
                messagebox.showerror("Hata", f"Arm edilemedi: {e}")

        asyncio.run_coroutine_threadsafe(arm(), self.loop)

    def disarm_drone(self):
        """Drone'u disarm et"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def disarm():
            try:
                logger.info("Drone disarm ediliyor...")
                await self.drone.action.disarm()
                logger.info("Drone disarm edildi!")
                self._update_status_label("Drone disarm edildi")
            except Exception as e:
                logger.error(f"Disarm hatası: {e}")
                messagebox.showerror("Hata", f"Disarm edilemedi: {e}")

        asyncio.run_coroutine_threadsafe(disarm(), self.loop)

    async def check_takeoff_status(self):
        """Drone kalkış durum kontrolü"""

        if not self.drone:
            return False

        try:
            # Belirli bir süre içinde telemetri verisini alma
            position = await asyncio.wait_for(
                self.drone.telemetry.position().__anext__(),
                timeout=3.0
            )

            # Hedef yüksekliği tanımla - eğer önceden tanımlanmamışsa varsayılan değer kullan
            target_altitude = 10.0  # varsayılan olarak 10 metre
            if hasattr(self.drone, "take_off_altitude_m"):
                target_altitude = self.drone.take_off_altitude_m

            # Göreceli yüksekliği kontrol et
            current_altitude = position.relative_altitude_m

            logger.info(f"Drone yüksekliği: {current_altitude:.2f} m / hedef: {target_altitude:.2f} m")

            if current_altitude > target_altitude * 0.9:
                logger.info(f"Drone kalkış başarılı: {current_altitude:.2f} m")
                self._update_status_label(f"Drone kalktı - Yükseklik: {current_altitude:.2f} m")
                return True
            else:
                logger.info(f"Drone henüz hedef yüksekliğe ulaşmadı: {current_altitude:.2f} m")
                self._update_status_label(f"Kalkış devam ediyor - {current_altitude:.2f} m")
                return False

        except asyncio.TimeoutError:
            logger.error("Pozisyon verisi alınamadı - zaman aşımı")
            return False
        except Exception as e:
            logger.error(f"Hata {e}")
            return False


    def takeoff_drone(self):
        """Drone kalkış"""

        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def takeoff():
            try:
                logger.info("Drone kalkış yapıyor...")
                await self.drone.action.takeoff()
                if await self.check_takeoff_status():
                    logger.info("Drone kalktı!")
                    self._update_status_label("Drone kalktı")
                else:
                    logger.info("Drone kalkış yapamadı!")
                    self._update_status_label("Drone kalkış yapamadı")
            except Exception as e:
                logger.error(f"Takeoff hatası: {e}")
                messagebox.showerror("Hata", f"Kalkış yapılamadı: {e}")

        asyncio.run_coroutine_threadsafe(takeoff(), self.loop)

    def land_drone(self):
        """Drone iniş"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def land():
            try:
                logger.info("Drone iniş yapıyor...")
                await self.drone.action.land()
                logger.info("Drone indi!")
                self._update_status_label("Drone indi")
            except Exception as e:
                logger.error(f"Land hatası: {e}")
                messagebox.showerror("Hata", f"İniş yapılamadı: {e}")

        asyncio.run_coroutine_threadsafe(land(), self.loop)

    def rtl_drone(self):
        """Return to Launch"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def rtl():
            try:
                logger.info("Drone eve dönüyor...")
                await self.drone.action.return_to_launch()
                logger.info("RTL komutu gönderildi!")
                self._update_status_label("Eve dönüyor")
            except Exception as e:
                logger.error(f"RTL hatası: {e}")
                messagebox.showerror("Hata", f"RTL başlatılamadı: {e}")

        asyncio.run_coroutine_threadsafe(rtl(), self.loop)

    def transition_fw(self):
        """Fixed-Wing moduna geç"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def t_fw():
            try:
                logger.info("Drone Fixed-Wing moduna geçiyor...")
                await self.drone.action.transition_to_fixedwing()
                logger.info("Drone Fixed-Wing moduna geçti!")
                self._update_status_label("Fixed-Wing modunda")
            except Exception as e:
                logger.error(f"Fixed-Wing geçiş hatası: {e}")
                messagebox.showerror("Hata", f"Fixed-Wing moduna geçilemedi: {e}")

        asyncio.run_coroutine_threadsafe(t_fw(), self.loop)

    async def disable_arm_checks(self, disable=True):
        """Arm kontrollerini devre dışı bırak"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return False

        try:
            # Arm kontrollerini devre dışı bırakmak için parametreleri ayarla
            param_names = [
                "COM_ARM_CHK_ESCS",
                "COM_ARM_MAG_STR",
                "COM_ARM_MIS_REQ",
                "COM_ARM_WO_GPS",
                "COM_ARM_SWISBTN"
            ]

            for param_name in param_names:
                # Parametreyi oku
                current_value = await self.drone.param.get_param_int(param_name)
                logger.info(f"Parametre {param_name} mevcut değeri: {current_value}")

                # Parametreyi ayarla (0=devre dışı, 1=etkin)
                new_value = 0 if disable else 1
                await self.drone.param.set_param_int(param_name, new_value)
                logger.info(f"Parametre {param_name} yeni değeri: {new_value}")

            # ARM_CHECK parametresi (eğer varsa)
            try:
                await self.drone.param.set_param_int("COM_ARM_CHK", 0 if disable else 1)
            except Exception:
                pass  # Bu parametre olmayabilir

            message = "Arm kontrolleri devre dışı bırakıldı!" if disable else "Arm kontrolleri etkinleştirildi!"
            logger.warning(message)
            messagebox.showinfo("Bilgi", message)
            self._update_status_label(message)
            return True

        except Exception as e:
            logger.error(f"Arm kontrolleri değiştirme hatası: {e}")
            messagebox.showerror("Hata", f"Arm kontrolleri değiştirilemedi: {e}")
            return False

    def transition_mc(self):
        """MultiCopter moduna geç"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def t_mc():
            try:
                logger.info("Drone MultiCopter moduna geçiyor...")
                await self.drone.action.transition_to_multicopter()
                logger.info("Drone MultiCopter moduna geçti!")
                self._update_status_label("MultiCopter modunda")
            except Exception as e:
                logger.error(f"MultiCopter geçiş hatası: {e}")
                messagebox.showerror("Hata", f"MultiCopter moduna geçilemedi: {e}")

        asyncio.run_coroutine_threadsafe(t_mc(), self.loop)

    def change_camera_mode(self):
        """Kamera modunu değiştir"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def change_camera():
            try:
                result = await self.drone.camera.get_mode(1)
                if result == Mode.PHOTO:
                    await self.drone.camera.set_mode(Mode.VIDEO)
                    logger.info("Video moduna geçildi")
                else:
                    await self.drone.camera.set_mode(Mode.PHOTO)
                    logger.info("Fotoğraf moduna geçildi")
            except CameraError as e:
                logger.error(f"Kamera modu değiştirme hatası: {e}")
                messagebox.showerror("Hata", f"Kamera modu değiştirilemedi: {e}")

        asyncio.run_coroutine_threadsafe(change_camera(), self.loop)

    def read_csv(self, file_path=None):
        """CSV dosyasından mission waypoint'lerini oku"""
        if file_path is None or not os.path.exists(file_path):
            logger.error("Geçerli bir CSV dosyası bulunamadı!")
            return [], [], []

        lats = []
        lons = []
        alts = []

        try:
            with open(file_path, "r") as file:
                csvf = csv.reader(file)

                for row in csvf:
                    if len(row) >= 3:
                        try:
                            lats.append(float(row[0]))
                            lons.append(float(row[1]))
                            alts.append(float(row[2]))
                        except ValueError as e:
                            logger.error(f"Satır verileri sayıya dönüştürülemedi: {row}, Hata: {e}")

                logger.info(f"CSV'den {len(lats)} adet waypoint okundu.")
        except Exception as e:
            logger.error(f"CSV okurken hata: {e}")

        return lats, lons, alts

    def upload_mission_and_start(self):
        """Mission yükle ve başlat"""
        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        file_path = filedialog.askopenfilename(
            title="Mission CSV dosyasını seçin",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )

        if not file_path:
            logger.info("Dosya seçilmedi!")
            return

        lats, lons, alts = self.read_csv(file_path)

        if not lats or not lons or not alts:
            logger.error("Geçerli waypoint bulunamadı!")
            messagebox.showerror("Hata", "CSV dosyasında geçerli waypoint bulunamadı!")
            return

        async def execute_mission():
            try:
                logger.info("Mission oluşturuluyor...")
                mission_items = []

                for i in range(len(lats)):
                    mission_items.append(MissionItem(
                        latitude_deg=lats[i],
                        longitude_deg=lons[i],
                        relative_altitude_m=alts[i],
                        speed_m_s=10.0,
                        is_fly_through=False,
                        acceptance_radius_m=4,
                        gimbal_pitch_deg=float('nan'),
                        gimbal_yaw_deg=float('nan'),
                        camera_action=MissionItem.CameraAction.NONE,
                        loiter_time_s=float('nan'),
                        camera_photo_interval_s=float('nan'),
                        camera_photo_distance_m=float('nan'),
                        vehicle_action=MissionItem.VehicleAction.NONE,
                        yaw_deg=float('nan')
                    ))

                if mission_items:
                    mission_items.append(MissionItem(
                        latitude_deg=lats[0],
                        longitude_deg=lons[0],
                        relative_altitude_m=0,
                        speed_m_s=10.0,
                        is_fly_through=False,
                        acceptance_radius_m=4,
                        gimbal_pitch_deg=float('nan'),
                        gimbal_yaw_deg=float('nan'),
                        camera_action=MissionItem.CameraAction.NONE,
                        loiter_time_s=float('nan'),
                        camera_photo_interval_s=float('nan'),
                        camera_photo_distance_m=float('nan'),
                        vehicle_action=MissionItem.VehicleAction.LAND,
                        yaw_deg=float('nan')
                    ))

                mission_plan = MissionPlan(mission_items)

                logger.info("Mevcut mission temizleniyor...")
                await self.drone.mission.clear_mission()

                logger.info("Yeni mission yükleniyor...")
                await self.drone.mission.upload_mission(mission_plan)

                logger.info("Drone durumu kontrol ediliyor...")
                async for health in self.drone.telemetry.health():
                    if not health.is_home_position_ok:
                        logger.warning("UYARI: Home pozisyonu ayarlanmamış!")
                    if not health.is_global_position_ok:
                        logger.warning("UYARI: Global pozisyon geçerli değil!")
                    if not health.is_armable:
                        logger.warning("UYARI: Drone arm edilemiyor!")
                    break

                is_armed = await self.drone.telemetry.armed().__anext__()
                if not is_armed:
                    logger.info("Drone arm ediliyor...")
                    try:
                        await self.drone.action.arm()
                        for _ in range(5):
                            is_armed = await self.drone.telemetry.armed().__anext__()
                            if is_armed:
                                logger.info("Drone başarıyla arm edildi!")
                                break
                            logger.info("Arm olmasını bekliyorum...")
                            await asyncio.sleep(1)
                    except Exception as arm_error:
                        logger.error(f"Arm hatası: {arm_error}")
                        messagebox.showerror("Hata", f"Drone arm edilemedi: {arm_error}")
                        return
                else:
                    logger.info("Drone zaten arm edilmiş.")

                flight_mode = await self.drone.telemetry.flight_mode().__anext__()
                logger.info(f"Mevcut uçuş modu: {flight_mode}")

                logger.info("Mission başlatılıyor...")
                try:
                    await self.drone.mission.start_mission()
                    logger.info("Mission başlatıldı!")
                    self._update_status_label("Mission çalışıyor...")
                except Exception as mission_error:
                    logger.error(f"Mission başlatma hatası: {mission_error}")
                    messagebox.showerror("Hata", f"Mission başlatılamadı: {mission_error}")

                    logger.info("Mission yeniden yükleniyor...")
                    await self.drone.mission.clear_mission()
                    await asyncio.sleep(1)
                    await self.drone.mission.upload_mission(mission_plan)

                    logger.info("Mission yeniden başlatılıyor...")
                    try:
                        await self.drone.mission.start_mission()
                        logger.info("Mission başarıyla başlatıldı!")
                        self._update_status_label("Mission çalışıyor...")
                    except Exception as retry_error:
                        logger.error(f"Mission yeniden başlatılamadı: {retry_error}")
                        messagebox.showerror("Hata", f"Mission tekrar başlatılamadı: {retry_error}")

                logger.info("Mission takip ediliyor...")
                async for prog in self.drone.mission.mission_progress():
                    logger.info(f"Mission İlerleme: {prog.current}/{prog.total}")
                    if prog.current == prog.total:
                        logger.info("Mission başarıyla tamamlandı!")
                        self._update_status_label("Mission tamamlandı!")
                        break

            except Exception as e:
                logger.error(f"Mission işlemleri sırasında hata: {str(e)}")
                messagebox.showerror("Mission Hatası", f"Mission sırasında hata: {str(e)}")

        asyncio.run_coroutine_threadsafe(execute_mission(), self.loop)

    def start_video_stream(self):
        """Video akışını başlat"""
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()

            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                logger.error("Kamera açılamadı!")
                self.video_canvas.delete("all")
                self.video_canvas.create_text(150, 100, text="Kamera açılamadı", fill="red")
                return

            self.video_processor.start_processing()
            self._update_video_stream()
            logger.info("Video akışı başlatıldı")
        except Exception as e:
            logger.error(f"Video başlatma hatası: {e}")
            messagebox.showerror("Hata", f"Video başlatılamadı: {e}")

    def _update_video_stream(self):
        """Video akışını güncelle"""
        try:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    if not self.video_processor.input_queue.full():
                        self.video_processor.input_queue.put_nowait(frame)

                    if not self.video_processor.output_queue.empty():
                        processed_frame = self.video_processor.output_queue.get()
                        processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2RGB)
                    else:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        processed_frame = frame

                    canvas_width = self.video_canvas.winfo_width() or 320
                    canvas_height = self.video_canvas.winfo_height() or 240

                    processed_frame = cv2.resize(processed_frame, (canvas_width, canvas_height))
                    img = Image.fromarray(processed_frame)
                    imgtk = ImageTk.PhotoImage(image=img)
                    self.video_canvas.delete("all")
                    self.video_canvas.create_image(0, 0, anchor="nw", image=imgtk)
                    self.video_canvas.image = imgtk

            if self.cap and self.cap.isOpened():
                self.root.after(30, self._update_video_stream)
        except Exception as e:
            logger.error(f"Video güncelleme hatası: {e}")

    def stop_video_stream(self):
        """Video akışını durdur"""
        try:
            if self.cap and self.cap.isOpened():
                self.cap.release()
                self.cap = None

            self.video_processor.stop_processing()
            self.video_canvas.delete("all")
            self.video_canvas.create_text(160, 120, text="Video durduruldu", fill="white")
            logger.info("Video akışı durduruldu")
        except Exception as e:
            logger.error(f"Video durdurma hatası: {e}")

    def clear_flight_path(self):
        """Uçuş yolunu temizle"""
        self.flight_path = [(self.current_lat, self.current_lon)]

        # Path line'ı haritadan sil
        if hasattr(self, 'path_line') and self.path_line:
            try:
                self.path_line.delete()
                self.path_line = None
            except Exception as e:
                logger.error(f"Path line silme hatası: {e}")

        logger.info("Uçuş yolu temizlendi")

        # Harita güncellemesini tetikle
        if hasattr(self, 'map_widget') and self.map_widget:
            self.root.after(0, self._update_map)

    def create_gui(self):
        """GUI oluştur"""
        self.root = ctk.CTk()
        self.root.title("ZADA-GCS v2.0 (Failsafe Enabled)")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("green")
        self.root.minsize(1200, 800)

        # Ana layout
        self._create_button_frame()
        self._create_info_frame()
        self._create_map_frame()
        self._create_control_frame()
        self._create_video_frame()
        self._create_log_frame()
        self._create_status_frame()

        # Grid ağırlıkları
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=3)
        self.root.grid_columnconfigure(2, weight=2)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(4, weight=1)
        self.root.grid_rowconfigure(5, weight=0)

        # Kapatma protokolü
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        logger.info("GUI oluşturuldu")

    def _create_button_frame(self):
        """Buton frame'ini oluştur"""
        button_frame = ctk.CTkFrame(self.root)
        button_frame.grid(row=0, column=0, columnspan=3, padx=5, pady=5, sticky="ew")

        for i in range(16):
            button_frame.grid_columnconfigure(i, weight=1)

        # İlk sıra butonları
        row1_buttons = [
            ("Connect", lambda: asyncio.run_coroutine_threadsafe(self.init_drone(), self.loop)),
            ("Arm", self.arm_drone),
            ("Takeoff", self.takeoff_drone),
            ("Land", self.land_drone),
            ("DisArm", self.disarm_drone),
            ("RTL", self.rtl_drone),
            ("FW", self.transition_fw),
            ("MC", self.transition_mc),
            ("Camera", self.change_camera_mode),
            ("Mission", self.upload_mission_and_start),
        ]

        for i, (text, command) in enumerate(row1_buttons):
            btn = ctk.CTkButton(button_frame, text=text, command=command)
            btn.grid(row=0, column=i, padx=2, pady=2, sticky="ew")

        # İkinci sıra butonları

        row2_buttons = [
            ("Video Start", self.start_video_stream),
            ("Video Stop", self.stop_video_stream),

            ("Video Stop", self.stop_video_stream),
            ("Clear Path", self.clear_flight_path),
            ("Clear Bottles", self.clear_bottle_markers),  # YENİ BUTON EKLENDİ
        ]

        for i, (text, command) in enumerate(row2_buttons):
            btn = ctk.CTkButton(button_frame, text=text, command=command)
            btn.grid(row=1, column=i, padx=2, pady=2, sticky="ew")
        # Şişe tespit listesi butonu - YENİ BUTON
        bottle_list_btn = ctk.CTkButton(
            button_frame,
            text="🍼 Bottle List",
            command=self.show_bottle_detections,
            fg_color="blue",
            hover_color="darkblue"
        )
        bottle_list_btn.grid(row=1, column=4, padx=2, pady=2, sticky="ew")
        # FAILSAFE butonu - kırmızı ve büyük
        failsafe_btn = ctk.CTkButton(
            button_frame,
            text="🚨 FAILSAFE 🚨",
            command=self.manual_failsafe,
            fg_color="red",
            hover_color="darkred",
            font=("Arial", 14, "bold"),
            height=40
        )
        failsafe_btn.grid(row=1, column=3, columnspan=2, padx=2, pady=2, sticky="ew")

        # Arm kontrolleri devre dışı bırakma butonu
        arm_check_btn = ctk.CTkButton(
            button_frame,
            text="ARM CHECK DEVRE DIŞI",
            command=lambda: asyncio.run_coroutine_threadsafe(self.disable_arm_checks(True), self.loop),
            fg_color="orange",
            hover_color="darkorange",
            font=("Arial", 12, "bold")
        )
        arm_check_btn.grid(row=1, column=6, columnspan=2, padx=2, pady=2, sticky="ew")

        # Arm kontrollerini geri aç butonu
        arm_check_enable_btn = ctk.CTkButton(
            button_frame,
            text="ARM CHECK ETKİNLEŞTİR",
            command=lambda: asyncio.run_coroutine_threadsafe(self.disable_arm_checks(False), self.loop),
            fg_color="blue",
            hover_color="darkblue"
        )
        arm_check_enable_btn.grid(row=1, column=8, columnspan=2, padx=2, pady=2, sticky="ew")

        # Port seçimi
        ports = self.list_ports()
        if ports:
            port_menu = ctk.CTkOptionMenu(button_frame, values=ports, command=self.on_port_selected)
            port_menu.grid(row=1, column=5, columnspan=3, padx=2, pady=2, sticky="ew")

    def _create_info_frame(self):
        """Bilgi frame'ini oluştur"""
        info_frame = ctk.CTkFrame(self.root)
        info_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        info_title = ctk.CTkLabel(info_frame, text="UAV BİLGİLERİ", font=("Arial", 14, "bold"))
        info_title.pack(padx=10, pady=(10, 5))

        self.info_label = ctk.CTkLabel(info_frame, text="Bağlantı bekleniyor...",
                                       font=("Arial", 12), justify="left")
        self.info_label.pack(padx=10, pady=5, fill="both", expand=True)

    def _create_map_frame(self):
        """Harita frame'ini oluştur"""
        map_frame = ctk.CTkFrame(self.root)
        map_frame.grid(row=1, column=1, rowspan=3, padx=5, pady=5, sticky="nsew")

        map_title = ctk.CTkLabel(map_frame, text="HARİTA (🚁 Drone - 🍼 Şişe)", font=("Arial", 14, "bold"))
        map_title.pack(padx=10, pady=(10, 5))

        try:
            self.map_widget = tkintermapview.TkinterMapView(
                map_frame,
                width=400,
                height=300,
                corner_radius=0
            )
            self.map_widget.pack(fill="both", expand=True, padx=10, pady=(0, 10))
            self.map_widget.set_position(self.current_lat, self.current_lon)
            self.map_widget.set_zoom(15)

            # İlk drone marker'ı oluştur
            self.drone_marker = self.map_widget.set_marker(
                self.current_lat,
                self.current_lon,
                text="🚁 Drone",
                marker_color_circle="red"
            )

            # Path line'ı başlat (None olarak)
            self.path_line = None

            logger.info("Harita başarıyla oluşturuldu")

        except Exception as e:
            logger.error(f"Harita oluşturma hatası: {e}")
            # Harita yoksa placeholder
            placeholder = ctk.CTkLabel(map_frame, text=f"Harita yüklenemedi: {e}")
            placeholder.pack(fill="both", expand=True)
            self.map_widget = None
            self.drone_marker = None
            self.path_line = None

    def _create_control_frame(self):
        """Kontrol frame'ini oluştur"""
        control_frame = ctk.CTkFrame(self.root)
        control_frame.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")

        control_title = ctk.CTkLabel(control_frame, text="KONTROL", font=("Arial", 14, "bold"))
        control_title.pack(padx=10, pady=(10, 5))

        coords_frame = ctk.CTkFrame(control_frame)
        coords_frame.pack(fill="x", padx=10, pady=5)

        self.uav_target_info = ctk.CTkEntry(coords_frame, placeholder_text="Enlem Boylam İrtifa")
        self.uav_target_info.pack(side="left", fill="x", expand=True, padx=5, pady=5)

        go_to_button = ctk.CTkButton(coords_frame, text="Go to", command=self._goto_drone)
        go_to_button.pack(side="right", padx=5, pady=5)

    def _create_video_frame(self):
        """Video frame'ini oluştur"""
        video_frame = ctk.CTkFrame(self.root)
        video_frame.grid(row=1, column=2, rowspan=2, padx=5, pady=5, sticky="nsew")

        video_title = ctk.CTkLabel(video_frame, text="VİDEO AKIŞI (AI Tespit)", font=("Arial", 14, "bold"))
        video_title.pack(padx=10, pady=(10, 5))

        self.video_canvas = tkinter.Canvas(video_frame, width=320, height=240, bg="black")
        self.video_canvas.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def _create_log_frame(self):
        """Log frame'ini oluştur"""
        log_frame = ctk.CTkFrame(self.root)
        log_frame.grid(row=4, column=0, columnspan=3, padx=5, pady=5, sticky="nsew")

        log_title = ctk.CTkLabel(log_frame, text="SİSTEM LOGLARI", font=("Arial", 14, "bold"))
        log_title.pack(padx=10, pady=(10, 5))

        # Scrollable text widget için frame
        log_container = ctk.CTkFrame(log_frame)
        log_container.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        # Text widget oluştur
        self.log_text = ctk.CTkTextbox(log_container, height=100, font=("Courier", 10))
        self.log_text.pack(fill="both", expand=True)

        # Log temizleme butonu
        clear_log_btn = ctk.CTkButton(log_frame, text="Logları Temizle", command=self._clear_logs)
        clear_log_btn.pack(pady=(0, 10))

    def _clear_logs(self):
        """Logları temizle"""
        self.log_messages.clear()
        if self.log_text:
            self.log_text.delete("1.0", "end")
        logger.info("Loglar temizlendi")

    def _create_status_frame(self):
        """Durum frame'ini oluştur"""
        status_frame = ctk.CTkFrame(self.root)
        status_frame.grid(row=5, column=0, columnspan=3, padx=5, pady=5, sticky="ew")

        self.status_label = ctk.CTkLabel(status_frame, text="Hazır", font=("Arial", 12, "bold"))
        self.status_label.pack(padx=10, pady=5)

    def _goto_drone(self):
        """Drone'u hedefe gönder - Geliştirilmiş versiyon"""
        data = self.uav_target_info.get().split()
        try:
            target_lat = float(data[0])
            target_lon = float(data[1])
            target_alt = float(data[2])
            user_yaw = float(data[3]) if len(data) > 3 else None
        except (IndexError, ValueError):
            messagebox.showerror("Hata", "Lütfen geçerli enlem, boylam ve irtifa giriniz!")
            return

        if not self.drone:
            messagebox.showwarning("Uyarı", "Drone bağlı değil!")
            return

        async def goto():
            try:
                # Failsafe'e goto'nun aktif olduğunu bildir
                if self.failsafe_manager:
                    self.failsafe_manager.set_goto_active(True)

                # Debug log ekle
                logger.info(f"Goto komutu başlatıldı: lat={target_lat}, lon={target_lon}, alt={target_alt}")
                if self.failsafe_manager and self.failsafe_manager.drone_state:
                    logger.info(
                        f"Mevcut drone durumu - Batarya: {self.failsafe_manager.drone_state.battery_level}%, İrtifa: {self.failsafe_manager.drone_state.altitude}m")

                # Uçuş modunu kontrol et
                flight_mode = await self.drone.telemetry.flight_mode().__anext__()
                logger.info(f"Goto öncesi uçuş modu: {flight_mode}")

                if user_yaw is None:
                    # Yaw hesapla
                    delta_lon = math.radians(target_lon - self.current_lon)
                    current_lat_rad = math.radians(self.current_lat)
                    target_lat_rad = math.radians(target_lat)

                    y = math.sin(delta_lon) * math.cos(target_lat_rad)
                    x = math.cos(current_lat_rad) * math.sin(target_lat_rad) - \
                        math.sin(current_lat_rad) * math.cos(target_lat_rad) * math.cos(delta_lon)

                    bearing = math.atan2(y, x)
                    target_yaw = (math.degrees(bearing) + 360) % 360
                else:
                    target_yaw = user_yaw

                logger.info(f"Hedefe gidiyor: {target_lat}, {target_lon}, {target_alt}m, {target_yaw:.1f}°")

                # Goto komutunu gönder
                await self.drone.action.goto_location(target_lat, target_lon, target_alt, target_yaw)

                logger.info("Goto komutu başarıyla gönderildi")
                self._update_status_label("Hedefe gidiyor...")

                # Komut sonrası uçuş modunu tekrar kontrol et
                await asyncio.sleep(2)
                flight_mode = await self.drone.telemetry.flight_mode().__anext__()
                logger.info(f"Goto sonrası uçuş modu: {flight_mode}")

                # Hedefe varış kontrolü (isteğe bağlı)
                await asyncio.sleep(10)  # 10 saniye sonra
                if self.failsafe_manager:
                    self.failsafe_manager.set_goto_active(False)

            except Exception as e:
                logger.error(f"Goto hatası: {e}")
                messagebox.showerror("Hata", f"Hedefe gidilemedi: {e}")

                # Hata durumunda failsafe'i normale döndür
                if self.failsafe_manager:
                    self.failsafe_manager.set_goto_active(False)

        asyncio.run_coroutine_threadsafe(goto(), self.loop)

    def _on_closing(self):
        """Uygulama kapatılırken"""
        logger.info("Uygulama kapatılıyor...")

        if self.failsafe_manager:
            self.failsafe_manager.stop_monitoring()

        if self.cap and self.cap.isOpened():
            self.cap.release()

        self.video_processor.stop_processing()
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.root.destroy()

    def run(self):
        """Uygulamayı çalıştır"""
        self.create_gui()
        logger.info("ZADA-GCS v2.0 Güncellenmiş Versiyon başlatıldı")
        self.root.mainloop()


# Ana program
if __name__ == "__main__":
    try:
        gcs = DroneGCS()
        gcs.run()
    except Exception as e:
        logger.critical(f"Kritik hata: {e}")
        messagebox.showerror("Kritik Hata", f"Uygulama başlatılamadı: {e}")
        logger.error("Uygulama başlatılamadı, lütfen logları kontrol edin.")

