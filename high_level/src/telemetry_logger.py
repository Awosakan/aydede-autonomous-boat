import os
import csv
import json
import time
import queue
import threading
import logging
import cv2

# Logger Setup
logger = logging.getLogger("IDA_Logger")
logger.setLevel(logging.INFO)

class AsyncLoggerManager:
    """
    Tüm loglama işlemlerini asenkron olarak arka planda yöneten sınıf.
    Ana döngünün disk I/O işlemlerinden dolayı duraksamasını (lag) önler.
    """
    def __init__(self, output_dir: str, secondary_output_dir: str = None):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Dosya yolları
        self.csv_path = os.path.join(output_dir, "dosya2_telemetri.csv")
        self.costmap_path = os.path.join(output_dir, "dosya3_costmap.jsonl")
        self.video_path = os.path.join(output_dir, "dosya1_kamera.mp4")
        
        # Yedek / USB Dosya yolları
        self.secondary_output_dir = secondary_output_dir
        self.sec_csv_path = None
        self.sec_costmap_path = None
        self.sec_video_path = None
        self.sec_video_writer = None
        
        if secondary_output_dir:
            try:
                os.makedirs(secondary_output_dir, exist_ok=True)
                # Yazma testi yapalım (F-35 Failsafe Standardı)
                test_file = os.path.join(secondary_output_dir, ".write_test")
                with open(test_file, "w") as f:
                    f.write("test")
                os.remove(test_file)
                
                self.sec_csv_path = os.path.join(secondary_output_dir, "dosya2_telemetri.csv")
                self.sec_costmap_path = os.path.join(secondary_output_dir, "dosya3_costmap.jsonl")
                self.sec_video_path = os.path.join(secondary_output_dir, "dosya1_kamera.mp4")
                logger.info(f"Harici USB loglama dizini aktif edildi: {secondary_output_dir}")
            except Exception as e:
                logger.error(f"Harici USB loglama dizini ({secondary_output_dir}) hazırlanamadı: {e}. Sadece ana dizine loglanacak.")
                self.secondary_output_dir = None
        
        # Telemetri ve Costmap kuyrukları
        self.telemetry_queue = queue.Queue()
        self.costmap_queue = queue.Queue()
        
        # Çalışma bayrağı
        self.running = False
        
        # CSV Başlıkları
        self.csv_headers = [
            "Timestamp", "Latitude", "Longitude", "Speed", 
            "Roll", "Pitch", "Heading", 
            "SpeedSetpoint", "HeadingSetpoint"
        ]
        
        # Video yazıcı bileşenleri
        self.video_writer = None
        self.video_queue = queue.Queue(maxsize=100) # OOM önlemek için maks 100 kare (yaklaşık 4 saniye tampon)
        
        # Yazıcı threadleri
        self.writer_thread = None
        self.video_thread = None

    def start(self, frame_width=640, frame_height=480, fps=24.0):
        self.running = True
        
        # CSV dosyasını başlat ve başlıkları yaz
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_headers)
                
        # Yedek CSV dosyasını başlat ve başlıkları yaz
        if self.sec_csv_path and not os.path.exists(self.sec_csv_path):
            try:
                with open(self.sec_csv_path, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(self.csv_headers)
            except Exception as e:
                logger.error(f"Yedek CSV dosyası başlatılamadı: {e}")
                self.sec_csv_path = None
                
        # Video yazıcıyı başlat
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, fps, (frame_width, frame_height))
        
        # Yedek Video yazıcıyı başlat
        if self.sec_video_path:
            try:
                self.sec_video_writer = cv2.VideoWriter(self.sec_video_path, fourcc, fps, (frame_width, frame_height))
            except Exception as e:
                logger.error(f"Yedek Video yazıcı başlatılamadı: {e}")
                self.sec_video_writer = None
        
        # Threadleri başlat
        self.writer_thread = threading.Thread(target=self._telemetry_writer_loop, daemon=True)
        self.video_thread = threading.Thread(target=self._video_writer_loop, daemon=True)
        
        self.writer_thread.start()
        self.video_thread.start()
        logger.info("Asenkron loglama sistemi başlatıldı.")

    def log_telemetry(self, lat: float, lon: float, speed: float, 
                      roll: float, pitch: float, heading: float, 
                      speed_sp: float, heading_sp: float):
        """
        Telemetri verisini kuyruğa ekler (>= 1 Hz çağrılmalıdır).
        """
        timestamp = time.time()
        data = [timestamp, lat, lon, speed, roll, pitch, heading, speed_sp, heading_sp]
        self.telemetry_queue.put(data)

    def log_costmap(self, grid_data: list, origin_x: float, origin_y: float, 
                    resolution: float, width: int, height: int):
        """
        Costmap / Engel haritasını kuyruğa ekler (>= 1 Hz çağrılmalıdır).
        """
        timestamp = time.time()
        payload = {
            "timestamp": timestamp,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "resolution": resolution,
            "width": width,
            "height": height,
            "grid": grid_data  # 1D veya 2D engel matrisi/koordinat listesi
        }
        self.costmap_queue.put(payload)

    def log_frame(self, frame):
        """
        Görüntü çerçevesini kuyruğa ekler. Çerçeveye asenkron olarak zaman damgası basılacaktır.
        """
        if frame is not None:
            try:
                # Görüntünün kopyasını kuyruğa ekle (referans sorunlarını önlemek için)
                # Kuyruk doluysa otonomi döngüsünü geciktirmemek için put_nowait kullanılır
                self.video_queue.put_nowait((time.time(), frame.copy()))
            except queue.Full:
                # Disk yazma hızı yetişemediğinde hafızanın şişmesini (OOM) engellemek için kareyi atla (drop)
                pass

    def _telemetry_writer_loop(self):
        """
        Telemetri ve Costmap verilerini diske yazan arka plan döngüsü.
        """
        while self.running or not self.telemetry_queue.empty() or not self.costmap_queue.empty():
            try:
                # Telemetri Yazımı
                try:
                    data = self.telemetry_queue.get(timeout=0.1)
                    # Ana CSV'ye yaz
                    with open(self.csv_path, mode='a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(data)
                    # Yedek CSV'ye yaz
                    if self.sec_csv_path:
                        try:
                            with open(self.sec_csv_path, mode='a', newline='') as f:
                                writer = csv.writer(f)
                                writer.writerow(data)
                        except Exception as e:
                            logger.error(f"Yedek USB CSV yazma hatası: {e}")
                    self.telemetry_queue.task_done()
                except queue.Empty:
                    pass
                
                # Costmap Yazımı
                try:
                    map_data = self.costmap_queue.get(timeout=0.1)
                    map_str = json.dumps(map_data) + "\n"
                    # Ana Costmap'e yaz
                    with open(self.costmap_path, mode='a') as f:
                        f.write(map_str)
                    # Yedek Costmap'e yaz
                    if self.sec_costmap_path:
                        try:
                            with open(self.sec_costmap_path, mode='a') as f:
                                f.write(map_str)
                        except Exception as e:
                            logger.error(f"Yedek USB Costmap yazma hatası: {e}")
                    self.costmap_queue.task_done()
                except queue.Empty:
                    pass
                    
            except Exception as e:
                logger.error(f"Error in telemetry writer loop: {e}")
                time.sleep(0.1)

    def _video_writer_loop(self):
        """
        Görüntülere zaman damgası basan ve MP4 formatında kaydeden arka plan döngüsü.
        """
        while self.running or not self.video_queue.empty():
            try:
                t, frame = self.video_queue.get(timeout=0.1)
                
                # Zaman damgası yazısını oluştur (Örn: 2026-05-20 17:09:46.123)
                local_time = time.localtime(t)
                milliseconds = int((t - int(t)) * 1000)
                time_str = time.strftime("%Y-%m-%d %H:%M:%S", local_time) + f".{milliseconds:03d}"
                
                # Zaman etiketini videonun sol üst köşesine yaz
                # Font, konum, boyut ve renk ayarları
                font = cv2.FONT_HERSHEY_SIMPLEX
                position = (10, 30)
                font_scale = 0.8
                color = (0, 255, 0)  # Yeşil renk
                thickness = 2
                
                # Arka plan için siyah bir kutu çiz (okunabilirliği artırmak için)
                text_size, _ = cv2.getTextSize(time_str, font, font_scale, thickness)
                cv2.rectangle(frame, (5, 5), (15 + text_size[0], 40), (0, 0, 0), -1)
                
                cv2.putText(frame, time_str, position, font, font_scale, color, thickness, cv2.LINE_AA)
                
                # Videoya yaz
                if self.video_writer is not None:
                    self.video_writer.write(frame)
                if self.sec_video_writer is not None:
                    try:
                        self.sec_video_writer.write(frame)
                    except Exception as e:
                        logger.error(f"Yedek USB Video yazma hatası: {e}")
                    
                self.video_queue.task_done()
            except queue.Empty:
                pass
            except Exception as e:
                logger.error(f"Error in video writer loop: {e}")
                time.sleep(0.1)

    def flush(self):
        """
        Kuyruklarda kalan tüm verilerin diske yazılmasını bekler (Maksimum 500 ms).
        """
        timeout = 0.5
        start_time = time.time()
        while not self.telemetry_queue.empty() or not self.costmap_queue.empty() or not self.video_queue.empty():
            if time.time() - start_time > timeout:
                break
            time.sleep(0.01)

    def stop(self):
        self.running = False
        
        # Threadlerin bitmesini bekle
        if self.writer_thread is not None:
            self.writer_thread.join()
        if self.video_thread is not None:
            self.video_thread.join()
            
        # Video yazıcıyı kapat
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        if self.sec_video_writer is not None:
            try:
                self.sec_video_writer.release()
            except Exception as e:
                logger.error(f"Yedek USB Video kapatma hatası: {e}")
            self.sec_video_writer = None
            
        logger.info("Loglama sistemi durduruldu ve dosyalar kapatıldı.")
