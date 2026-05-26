import sys
import os
import time
import threading
import logging
import cv2
import numpy as np
import serial
import gc

# Modüllerimizi içe aktaralım
# Python'ın dosyayı doğrudan çalıştırma durumunu desteklemek için path eklemesi yapalım
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.protocol import IDAParser, MSG_STM32_TELEMETRY, unpack_stm32_telemetry, pack_heartbeat, MSG_HEARTBEAT, IDAPacket, MODE_IDLE, MODE_AUTO
from src.telemetry_logger import AsyncLoggerManager
from src.detector import BuoyDetector
from src.costmap import LocalCostmap
from src.mission_control import MissionController, STATE_PARKUR1

# Logger Setup
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s")
logger = logging.getLogger("IDA_Main")

class MockSerial:
    """
    Test bilgisayarlarında STM32 bağlı değilken yazılımın çökmemesini ve
    test edilebilmesini sağlayan sahte seri port sınıfı (Failsafe & Simülasyon).
    """
    def write(self, data):
        pass
    def read(self, size=1):
        time.sleep(0.01)
        return b""
    def close(self):
        pass

class VideoGrabber(threading.Thread):
    """
    Kamera okuma işleminin (cap.read) işletim sistemi seviyesinde kilitlenerek
    ana otonomi döngüsünü dondurmasını engellemek için asenkron okuyucu thread (Hata: 255 çözümü).
    """
    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap = cap
        self.ret = False
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        
    def run(self):
        self.running = True
        while self.running:
            if self.cap.isOpened():
                try:
                    ret, frame = self.cap.read()
                    with self.lock:
                        self.ret = ret
                        if ret:
                            self.frame = frame.copy()
                except Exception as e:
                    logger.error(f"Kamera okuma hatası: {e}")
                    with self.lock:
                        self.ret = False
            time.sleep(0.01) # Maks 100 FPS
            
    def read(self):
        with self.lock:
            return self.ret, self.frame
            
    def stop(self):
        self.running = False


class YOLOInferenceWorker(threading.Thread):
    """
    YOLO çıkarımının (detector.detect) ana otonomi döngüsünü (24Hz) 
    bloklamasını engellemek amacıyla asenkron çıkarım yapan thread sınıfı.
    """
    def __init__(self, detector):
        super().__init__(daemon=True)
        self.detector = detector
        self.running = False
        self.lock = threading.Lock()
        
        # Giriş verileri
        self.frame = None
        self.pitch = 0.0
        self.roll = 0.0
        self.new_frame_available = False
        
        # Çıkış verileri
        self.latest_detections = []
        
    def update_frame(self, frame, pitch, roll):
        with self.lock:
            self.frame = frame.copy() if frame is not None else None
            self.pitch = pitch
            self.roll = roll
            self.new_frame_available = True
            
    def get_latest_detections(self):
        with self.lock:
            return list(self.latest_detections)
            
    def run(self):
        self.running = True
        while self.running:
            frame_to_process = None
            p, r = 0.0, 0.0
            
            with self.lock:
                if self.new_frame_available and self.frame is not None:
                    frame_to_process = self.frame
                    p = self.pitch
                    r = self.roll
                    self.new_frame_available = False
                    
            if frame_to_process is not None:
                try:
                    # YOLO/HSV tespiti gerçekleştir
                    dets = self.detector.detect(frame_to_process, pitch=p, roll=r)
                    with self.lock:
                        self.latest_detections = dets
                except Exception as e:
                    logger.error(f"YOLO Thread çıkarım hatası: {e}")
                    
            time.sleep(0.005) # CPU'yu aşırı yormamak için kısa bekleme
            
    def stop(self):
        self.running = False


class IDANode:
    def __init__(self, serial_port: str = "/dev/ttyACM0", baudrate: int = 115200, 
                 model_path: str = None, video_source=0):
        
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.video_source = video_source
        self.running = False
        
        # Load config.json dynamically
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "config.json")
        
        self.config = {
            "p1_wps": [
                [40.732501, 29.831201],
                [40.732702, 29.831502],
                [40.732903, 29.831203],
                [40.732704, 29.830904]
            ],
            "p2_wps": [
                [40.733100, 29.831500],
                [40.733500, 29.831500]
            ],
            "home_wp": [40.732501, 29.831201],
            "nominal_speed_ms": 1.3,
            "max_speed_ms": 2.0,
            "min_speed_ms": 0.5,
            "waypoint_tolerance_m": 0.6,
            "inflation_radius_m": 1.0,
            "costmap_size_m": 40.0,
            "costmap_resolution": 0.25,
            "target_color": "target_red",
            "state_timeout_seconds": 300.0,
            "max_speed_accel": 0.8,
            "max_yaw_rate": 180.0,
            "usb_log_dir": None,
            "yolo_classes": {
                "0": "orange_gate",
                "1": "yellow_obstacle",
                "2": "target_red",
                "3": "target_green",
                "4": "target_blue"
            }
        }
        
        if os.path.exists(config_path):
            try:
                import json
                with open(config_path, "r", encoding="utf-8") as f:
                    loaded_config = json.load(f)
                    self.config.update(loaded_config)
                logger.info(f"Yapılandırma dosyası başarıyla yüklendi: {config_path}")
            except Exception as e:
                logger.error(f"Yapılandırma dosyası yüklenirken hata oluştu: {e}")
        else:
            logger.warning(f"Yapılandırma dosyası bulunamadı, varsayılanlar oluşturuluyor: {config_path}")
            try:
                import json
                with open(config_path, "w", encoding="utf-8") as f:
                    json.dump(self.config, f, indent=4)
            except Exception as e:
                logger.error(f"Varsayılan yapılandırma dosyası yazılamadı: {e}")
 
        # 1. Asenkron Loglama Yöneticisi (Şartnamedeki 3 Dosya Çıktısı İçin)
        self.logger_manager = AsyncLoggerManager(
            output_dir="./ida_logs",
            secondary_output_dir=self.config.get("usb_log_dir", None)
        )
        
        # 2. Seri Port Bağlantısı
        self.ser = None
        self._init_serial()
        
        # 3. Seri Protokol Parser
        self.parser = IDAParser(callback=self.on_packet_received)
        
        # 4. Görev Kontrolcü
        self.mission = MissionController(self.logger_manager, self, self.config)
        
        # 5. Duba Dedektörü (Yedekli Model + HSV)
        self.detector = BuoyDetector(
            model_path=model_path, 
            image_width=640, 
            image_height=480,
            classes_dict=self.config.get("yolo_classes", None)
        )
        
        # Kamera donma/kopma kontrolü değişkenleri
        self.camera_lost = False
        self.camera_frozen = False
        self.last_frame = None
        self.frozen_frames_counter = 0
        self.grabber = None
        
        # 6. YOLO Asenkron İşçi Thread'i
        self.yolo_worker = YOLOInferenceWorker(self.detector)
        
        # 6. Yerel Engel Haritası (Costmap)
        self.costmap = LocalCostmap(
            size_m=self.config.get("costmap_size_m", 40.0),
            resolution=self.config.get("costmap_resolution", 0.25),
            inflation_radius_m=self.config.get("inflation_radius_m", 1.0)
        )
        
        # Görev noktalarını tanımla
        p1_wps = self.config.get("p1_wps", [])
        p2_wps = self.config.get("p2_wps", [])
        home_wp = self.config.get("home_wp", [40.732501, 29.831201])
        
        self.mission.set_waypoints(p1_wps, p2_wps, home_wp)

    def _init_serial(self):
        # [Performans Optimizasyonu]: Linux'ta USB seri gecikmesini 1ms'ye indir (Düşük Gecikmeli Seri Haberleşme)
        if sys.platform.startswith('linux'):
            try:
                dev_name = os.path.basename(self.serial_port)
                latency_path = f"/sys/bus/usb-serial/devices/{dev_name}/latency_timer"
                if os.path.exists(latency_path):
                    with open(latency_path, "w") as f:
                        f.write("1")
                    logger.info(f"Performans Optimizasyonu: USB Seri gecikme süresi {dev_name} için 1ms olarak ayarlandı.")
            except Exception as e:
                logger.warning(f"USB Gecikme süresi otomatik ayarlanamadı (Sudo yetkisi gerekebilir): {e}")

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            logger.info(f"Seri port bağlantısı başarılı: {self.serial_port}")
        except Exception as e:
            logger.error(f"Seri port açılamadı ({e}). Sahte (Mock) seri haberleşme başlatılıyor.")
            self.ser = MockSerial()

    def send_packet(self, payload: bytes, msg_id: int = 0x02):
        """
        Gövdeyi paketleyip seri porttan STM32'ye gönderir.
        """
        packet = IDAPacket(msg_id, payload)
        try:
            self.ser.write(packet.pack())
        except Exception as e:
            logger.error(f"Packet send error: {e}")

    def on_packet_received(self, msg_id: int, payload: bytes):
        """
        Seri porttan geçerli bir paket ayrıştırıldığında çağrılan callback.
        """
        if msg_id == MSG_STM32_TELEMETRY:
            try:
                telemetry = unpack_stm32_telemetry(payload)
                self.mission.update_telemetry(telemetry)
            except Exception as e:
                logger.error(f"Failed to unpack telemetry: {e}")

    def _serial_read_loop(self):
        """
        Seri porttan sürekli veri okuyan ve parser'a besleyen thread.
        Hata durumunda otomatik yeniden bağlanma (reconnect) mantığı içerir.
        """
        while self.running:
            try:
                # Sahte (Mock) seri modunda ise basitçe bekle ve veri beslemeyi sürdür
                if isinstance(self.ser, MockSerial):
                    data = self.ser.read(32)
                    if data:
                        self.parser.feed_data(data)
                    time.sleep(0.04) # ~25Hz
                    continue

                data = self.ser.read(32)
                if data:
                    self.parser.feed_data(data)
            except Exception as e:
                logger.error(f"Seri port okuma hatası: {e}. Yeniden bağlanmaya çalışılıyor...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(1.0)
                # Yeniden bağlanma (reconnect) girişimi
                try:
                    if sys.platform.startswith('linux'):
                        dev_name = os.path.basename(self.serial_port)
                        latency_path = f"/sys/bus/usb-serial/devices/{dev_name}/latency_timer"
                        if os.path.exists(latency_path):
                            with open(latency_path, "w") as f:
                                f.write("1")
                    self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
                    logger.info("Seri port bağlantısı başarıyla yeniden kuruldu.")
                except Exception as recon_err:
                    logger.error(f"Seri port yeniden bağlanma başarısız: {recon_err}")

    def _heartbeat_loop(self):
        """
        STM32'ye 10 Hz frekansta kalp atışı (heartbeat) paketi gönderir (F-35 Failsafe standardı).
        """
        rate = 1.0 / 10.0 # 10 Hz
        while self.running:
            # 1 status (OK), 1 auto mode (aktif göreve göre)
            sys_status = 1
            sys_mode = MODE_AUTO if "PARKUR" in self.mission.state else MODE_IDLE
            hb_payload = pack_heartbeat(sys_status, sys_mode)
            self.send_packet(hb_payload, msg_id=MSG_HEARTBEAT)
            time.sleep(rate)

    def start(self):
        self.running = True
        
        # [Performans Optimizasyonu]: Ana otonomi thread'ini Snapdragon 845'in Kryo Gold (büyük) çekirdeklerine kilitle
        if hasattr(os, "sched_setaffinity"):
            try:
                # Cores 4-7: Kryo Gold (Büyük performans çekirdekleri)
                os.sched_setaffinity(0, {4, 5, 6, 7})
                logger.info("Performans Optimizasyonu: Ana otonomi iş parçacığı büyük CPU çekirdeklerine (4-7) kilitlendi.")
            except Exception as e:
                logger.warning(f"CPU Çekirdek kilitlemesi başarısız oldu: {e}")
        
        # [Performans Optimizasyonu]: Bellek sızıntılarını ve OOM (Hafıza Tükenmesi) durumlarını önlemek için GC açık tutulur.
        gc.enable()
        gc.collect()
        logger.info("Performans Optimizasyonu: Otomatik Çöp Toplayıcı (GC) bellek sızıntılarını engellemek amacıyla aktif tutuldu.")
        
        # 1. Logları Başlat
        self.logger_manager.start(frame_width=640, frame_height=480, fps=24.0)
        
        # 2. Seri Okuma Threadini Başlat
        self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.read_thread.start()
        
        # 3. Heartbeat Threadini Başlat
        self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.hb_thread.start()
        
        # 4. Kamera Başlatılması
        cap = cv2.VideoCapture(self.video_source)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 24) # Şartname minimum 24 FPS
        
        # Görev 1.8: Kamera beyaz dengesi ve otomatik pozlamanın kilitlenmesi
        if cap.isOpened():
            # OpenCV üzerinden manuel mod ayarları (Destekleyen kameralar için)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # 1 = Manual Mode
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)       # 0 = Manual Mode
            
            # Linux altında V4L2 ile kesin kilitleme
            if sys.platform.startswith('linux'):
                try:
                    os.system("v4l2-ctl -c exposure_auto=1")
                    os.system("v4l2-ctl -c white_balance_temperature_auto=0")
                    logger.info("Performans Optimizasyonu: Linux V4L2 üzerinden otomatik pozlama ve beyaz dengesi kilitlendi.")
                except Exception as e:
                    logger.warning(f"V4L2 kamera kilitleme komutu başarısız: {e}")
        else:
            logger.error("Kamera açılamadı! Sistem yedek (simüle) görüntü moduna geçiyor.")
            
        # Asenkron kamera grabber thread'ini başlat (Hata: 255 çözümü)
        if cap.isOpened():
            self.grabber = VideoGrabber(cap)
            self.grabber.start()
            logger.info("Asenkron Kamera Grabber thread başlatıldı.")
        else:
            self.grabber = None
            
        # Asenkron YOLO çıkarım thread'ini başlat (Görev 3)
        self.yolo_worker.start()
        logger.info("Asenkron YOLO Çıkarım Worker thread başlatıldı.")
            
        logger.info("İDA otonomi düğümü başlatıldı. Görev tetiklenmesi bekleniyor...")
        
        # Otonomi Döngüsü (24 FPS Kontrol)
        frame_time = 1.0 / 24.0
        
        # Test amaçlı otomatik göreve başlama komutu (Normalde YKİ'den veya RC'den gelir)
        # 3 saniye sonra otomatik Parkur 1'i başlatalım
        start_time = time.time()
        auto_started = False
        
        try:
            while self.running:
                loop_start = time.time()
                
                # STM32 bağlı değilse sahte telemetri besle (Failsafe ve Çevrimdışı Test Desteği)
                if isinstance(self.ser, MockSerial):
                    mock_telemetry = {
                        "lat": 40.732501,
                        "lon": 29.831201,
                        "sog": 1.0,
                        "cog": 0.0,
                        "gps_lock": 1,
                        "roll": 0.0,
                        "pitch": 0.0,
                        "yaw": 0.0,
                        "roll_rate": 0.0,
                        "pitch_rate": 0.0,
                        "yaw_rate": 0.0,
                        "battery": 12.0,
                        "mode": 1
                    }
                    self.mission.update_telemetry(mock_telemetry)
                
                # Test/Simülasyon başlatma tetiği
                if not auto_started and (loop_start - start_time > 3.0):
                    logger.info("Otonom Görev Tetiklendi!")
                    self.mission.transition_to(STATE_PARKUR1)
                    auto_started = True
                
                # Görev 1.6: Kamera Bağlantı Durum Kontrolü
                if not cap.isOpened() or self.grabber is None:
                    if isinstance(self.ser, MockSerial):
                        # Simülasyonda yapay görüntü üret
                        ret, frame = True, self._create_test_frame()
                    else:
                        # Gerçek yarışmada kamera bağlantısı koptu
                        logger.error("Failsafe: Kamera bağlantısı koptu!")
                        self.camera_lost = True
                        ret, frame = False, None
                else:
                    ret, frame = self.grabber.read()
                    if not ret:
                        if isinstance(self.ser, MockSerial):
                            ret, frame = True, self._create_test_frame()
                        else:
                            logger.error("Failsafe: Kameradan kare okunamıyor (kablo çıkmış olabilir)!")
                            self.camera_lost = True
                
                if ret and frame is not None:
                    # Görev 1.6: Görüntü Donması Kontrolü
                    if self.last_frame is not None and not isinstance(self.ser, MockSerial):
                        diff = cv2.absdiff(frame, self.last_frame)
                        mean_diff = np.mean(diff)
                        if mean_diff < 0.05:  # Kareler birebir aynı veya çok yakınsa donma algılanır
                            self.frozen_frames_counter += 1
                            if self.frozen_frames_counter >= 24:  # ~1 saniye (24 kare) boyunca donma
                                self.camera_frozen = True
                                logger.error("Failsafe: Kamera görüntüsü dondu!")
                        else:
                            self.frozen_frames_counter = 0
                    self.last_frame = frame.copy()
                    
                    # Görüntü İşleme ve Duba Tespiti (Görev 1.5: Pitch/Roll yalpalama telafisi dahil)
                    pitch = self.mission.current_pitch
                    roll = self.mission.current_roll
                    
                    # YOLO worker thread'ine en güncel kareyi ve yönelim verilerini besle (Görev 3)
                    self.yolo_worker.update_frame(frame, pitch, roll)
                    
                    # YOLO worker'dan o anki en güncel duba listesini asenkron olarak al
                    detections = self.yolo_worker.get_latest_detections()
                    
                    # Görev Durum Makinesi Adımı (Görüntü + Harita + Planlama)
                    self.mission.process_step(detections, self.costmap)
                    
                    # Tespitleri ekrana çiz (MP4 video kaydı için)
                    annotated_frame = self.detector.draw_detections(frame, detections)
                    
                    # Log kuyruğuna çerçeveyi asenkron yazılmak üzere ekle
                    self.logger_manager.log_frame(annotated_frame)
                    
                    # Görsel arayüz (Ekranlı testler için - telefonda arka planda çalışırken kapatılabilir)
                    if "DISPLAY" in os.environ:
                        cv2.imshow("IDA Autonomy Monitor", annotated_frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                else:
                    # Kamera hatası veya kopması durumunda failsafe durum makinesi adımı (Failsafe ve Log devamlılığı)
                    detections = []
                    self.mission.process_step(detections, self.costmap)
                    
                    # Boş bir hata ekranı oluşturup logluyoruz
                    err_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(err_frame, "KAMERA BAGLANTISI KOPUK / HATA", (80, 240), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.logger_manager.log_frame(err_frame)
                            
                # 24 FPS kararlılığı için bekleme süresini ayarla
                elapsed = time.time() - loop_start
                sleep_time = frame_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            logger.info("Kullanıcı tarafından durduruldu.")
        finally:
            self.stop()
            if cap.isOpened():
                cap.release()
            cv2.destroyAllWindows()

    def _create_test_frame(self):
        """
        Kamera bağlı değilken boş test çerçevesi üreterek programın çalışmasını sağlar.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Ortaya yapay bir turuncu duba çizelim (Test amaçlı görsel algılama testi)
        # Turuncu BGR: (0, 165, 255)
        cv2.circle(frame, (320, 240), 20, (0, 165, 255), -1)
        # Bir sarı duba çizelim
        cv2.circle(frame, (150, 200), 15, (0, 255, 255), -1)
        return frame

    def stop(self):
        logger.info("Sistem kapatılıyor, güvenli moda geçiliyor...")
        self.running = False
        
        # Grabber thread'ini durdur
        if hasattr(self, "grabber") and self.grabber is not None:
            self.grabber.stop()
            try:
                self.grabber.join(timeout=1.0)
            except Exception as e:
                logger.error(f"Grabber thread join hatası: {e}")
                
        # YOLO worker thread'ini durdur (Görev 3)
        if hasattr(self, "yolo_worker") and self.yolo_worker is not None:
            self.yolo_worker.stop()
            try:
                self.yolo_worker.join(timeout=1.0)
            except Exception as e:
                logger.error(f"YOLO worker thread join hatası: {e}")
        
        # Logları kapat
        self.logger_manager.stop()
        
        # Seri okuma ve heartbeat threadlerini durdur (D6 düzeltmesi)
        if hasattr(self, "read_thread") and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        if hasattr(self, "hb_thread") and self.hb_thread.is_alive():
            self.hb_thread.join(timeout=1.0)
        
        # Seri portu kapat
        if self.ser is not None:
            self.ser.close()
            
        # [Performans Optimizasyonu]: GC'yi tekrar aç ve elle temizle
        gc.enable()
        gc.collect()
        logger.info("Performans Optimizasyonu: Çöp toplayıcı (GC) yeniden etkinleştirildi ve manuel temizlik yapıldı.")
            
        logger.info("Sistem başarıyla durduruldu.")

if __name__ == "__main__":
    # Örnek çalıştırma parametreleri
    # OnePlus 6 üzerinde çalışırken: python main.py /dev/ttyACM0 115200
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    # Otomatik model algılama
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_model = os.path.join(script_dir, "en_iyi_duba_modeli.onnx")
    model_path = default_model if os.path.exists(default_model) else None
    
    if model_path:
        logger.info(f"Otomatik duba tespit modeli bulundu ve yüklenecek: {model_path}")
    else:
        logger.warning("YOLO ONNX model dosyası ('en_iyi_duba_modeli.onnx') bulunamadı. HSV yedek modunda başlatılıyor.")
        
    node = IDANode(serial_port=port, baudrate=baud, model_path=model_path)
    node.start()
