import cv2
import numpy as np
import logging
import math

logger = logging.getLogger("IDA_Detector")
logger.setLevel(logging.INFO)

class BuoyDetector:
    """
    Duba algılama ve konum kestirim sınıfı.
    YOLO ONNX ve HSV Renk Eşikleme olmak üzere çift kanallı yedekli çalışır (F-35 Failsafe standardı).
    """
    def __init__(self, model_path: str = None, 
                 image_width: int = 640, 
                 image_height: int = 480,
                 hfov: float = 80.0,  # Derece cinsinden Yatay Görüş Açısı (Horizontal Field of View)
                 conf_threshold: float = 0.35,
                 nms_threshold: float = 0.6,
                 classes_dict: dict = None):
        
        self.image_width = image_width
        self.image_height = image_height
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        # Kamera Parametreleri (Odak Uzaklığı - Focal Length Hesaplama)
        self.hfov_rad = math.radians(hfov)
        self.focal_length_px = (self.image_width / 2.0) / math.tan(self.hfov_rad / 2.0)
        
        # Fiziksel Duba Boyutları (Şartnameye göre çapı 30 cm = 0.3 metre)
        self.BUOY_REAL_WIDTH_M = 0.30 
        
        # Sınıflar (YOLO ve Renk Filtrelemede Ortak)
        self.classes = {
            0: "orange_gate",      # Şartname Turuncu Duba (RAL 2003)
            1: "yellow_obstacle",  # Şartname Sarı Duba (RAL 1026)
            2: "target_red",       # Parkur 3 Kamikaze Hedef Kırmızı
            3: "target_green",     # Parkur 3 Kamikaze Hedef Yeşil
            4: "target_blue"       # Parkur 3 Kamikaze Hedef Mavi
        }
        if classes_dict:
            try:
                self.classes = {int(k): v for k, v in classes_dict.items()}
            except Exception as e:
                logger.error(f"Sınıf eşlemeleri yüklenirken hata: {e}")
        
        # Model Yükleme
        self.net = None
        self.use_fallback = True
        
        if model_path:
            try:
                # OpenCV DNN ile ONNX modelini yükle
                self.net = cv2.dnn.readNetFromONNX(model_path)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                
                # [GPU Hızlandırma Optimizasyonu]: Adreno 630 GPU üzerinde OpenCL veya Vulkan ile çalıştır
                try:
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)
                    logger.info("Performans Optimizasyonu: YOLO Çıkarımı GPU (OpenCL) üzerine yönlendirildi.")
                except Exception:
                    try:
                        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_VULKAN)
                        logger.info("Performans Optimizasyonu: YOLO Çıkarımı GPU (Vulkan) üzerine yönlendirildi.")
                    except Exception:
                        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                        logger.info("Performans Optimizasyonu: GPU hedef atanamadı, çıkarım CPU (ARM NEON) üzerinde yapılacak.")
                
                self.use_fallback = False
                logger.info(f"YOLO ONNX modeli başarıyla yüklendi: {model_path}")
            except Exception as e:
                logger.error(f"YOLO modeli yüklenemedi: {e}. HSV Renk Filtreleme moduna geçiliyor.")
                self.use_fallback = True
        else:
            logger.info("Model dosyası belirtilmedi. HSV Renk Filtreleme modunda çalışılıyor.")
            self.use_fallback = True

        # --- Gelişmiş Emniyet Filtreleri (Suda 10 Kötü Senaryo Önlemleri) ---
        # Senaryo 5 (Su Sıçraması / Kamera Kapanması) Kontrolü
        self.frame_count = 0
        self.camera_blocked = False
        
        # Görev 1.1 & 1.4: Çoklu duba takibi ve Jitter yumuşatma için Centroid Tracker veri yapıları
        self.next_track_id = 0
        self.tracks = []  # Aktif izler listesi
        self.filter_alpha = 0.35  # Jitter filtreleme EMA ağırlığı (0.0: tam sönümleme, 1.0: filtre yok)

    def check_lens_obstruction(self, frame) -> bool:
        """
        [Kötü Senaryo 5]: Merceğe su gelmesi veya kameranın tamamen kapanması durumunu kontrol eder.
        Görüntüdeki renk varyansını (kontrastı) ve ortalama parlaklığı ölçer.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        std_deviation = np.std(gray)
        
        # Lens kapandıysa veya su damlasından dolayı görüntü aşırı bulanıklaştıysa standart sapma çok düşer.
        if std_deviation < 5.0 or mean_brightness < 8.0:
            if not self.camera_blocked:
                logger.warning(f"ACİL DURUM: Kamera merceği kapandı veya su sıçradı! Kontrast: {std_deviation:.1f}, Parlaklık: {mean_brightness:.1f}")
                self.camera_blocked = True
            return True
            
        self.camera_blocked = False
        return False

    def temporal_filter(self, raw_detections: list) -> list:
        """
        [Kötü Senaryo 6 & Görev 1.1 & 1.4 & 1.2]: Çoklu nesne takibi (Centroid Tracking),
        jitter yumuşatma filtresi (EMA) ve ekran kenarı kesilme telafisi.
        """
        confirmed_detections = []
        matched_track_indices = set()
        matched_det_indices = set()
        
        # 1. Mevcut izler ile yeni tespitleri eşleştirmeye çalış
        matches = []
        for det_idx, det in enumerate(raw_detections):
            cls = det["class"]
            dist = det["distance"]
            bearing = det["bearing"]
            
            # Kartezyen koordinatlar (İDA referanslı)
            x_det = dist * math.sin(bearing)
            y_det = dist * math.cos(bearing)
            
            for track_idx, track in enumerate(self.tracks):
                if track["class"] != cls:
                    continue
                
                # İz geçmişindeki son konum
                last_dist, last_bearing = track["history"][-1]
                x_track = last_dist * math.sin(last_bearing)
                y_track = last_dist * math.cos(last_bearing)
                
                # Öklid mesafesi
                distance_m = math.sqrt((x_det - x_track)**2 + (y_det - y_track)**2)
                
                # Eşleşme limitleri: 3.0 metre ve 20 derece açı sınırı
                if distance_m < 3.0 and abs(bearing - last_bearing) < math.radians(20.0):
                    matches.append((distance_m, det_idx, track_idx))
                    
        # Mesafeye göre küçükten büyüğe sırala
        matches.sort(key=lambda val: val[0])
        
        # En yakın eşleşmeleri kesinleştir
        for dist_m, det_idx, track_idx in matches:
            if det_idx in matched_det_indices or track_idx in matched_track_indices:
                continue
                
            matched_det_indices.add(det_idx)
            matched_track_indices.add(track_idx)
            
            track = self.tracks[track_idx]
            det = raw_detections[det_idx]
            
            # Görev 1.2: Ekran Sınır Kontrolü (Edge Truncation) telafisi
            # Eğer yeni tespit ekran kenarındaysa (truncated), mesafe patlamasını önlemek için 
            # iz geçmişindeki son güvenilir mesafeyi kullan.
            if det.get("is_truncated", False) and len(track["history"]) > 0:
                det["distance"] = track["history"][-1][0]
            
            # Görev 1.4: BB/Mesafe Jitter Filtresi (EMA - Exponential Moving Average)
            if "filtered_distance" in track:
                track["filtered_distance"] = self.filter_alpha * det["distance"] + (1.0 - self.filter_alpha) * track["filtered_distance"]
                track["filtered_bearing"] = self.filter_alpha * det["bearing"] + (1.0 - self.filter_alpha) * track["filtered_bearing"]
            else:
                track["filtered_distance"] = det["distance"]
                track["filtered_bearing"] = det["bearing"]
                
            # Filtrelenmiş değerleri geri yaz
            det["distance"] = track["filtered_distance"]
            det["bearing"] = track["filtered_bearing"]
            
            # Geçmişe ekle
            track["history"].append((det["distance"], det["bearing"]))
            if len(track["history"]) > 5:
                track["history"].pop(0)
            track["missed_frames"] = 0
            
            # 5 karede en az 3 kez görüldüyse izi onayla
            if len(track["history"]) >= 3:
                track["confirmed"] = True
                
            if track["confirmed"]:
                confirmed_detections.append(det)
                
        # 2. Eşleşmeyen yeni tespitler için yeni izler oluştur
        for det_idx, det in enumerate(raw_detections):
            if det_idx not in matched_det_indices:
                new_track = {
                    "id": self.next_track_id,
                    "class": det["class"],
                    "history": [(det["distance"], det["bearing"])],
                    "filtered_distance": det["distance"],
                    "filtered_bearing": det["bearing"],
                    "confirmed": False,
                    "missed_frames": 0
                }
                self.next_track_id += 1
                self.tracks.append(new_track)
                
        # 3. Eşleşmeyen eski izleri yaşlandır ve temizle (3 kare boyunca görülmezse sil)
        remaining_tracks = []
        for track_idx, track in enumerate(self.tracks):
            if track_idx not in matched_track_indices:
                track["missed_frames"] += 1
                if track["missed_frames"] <= 3:
                    remaining_tracks.append(track)
            else:
                remaining_tracks.append(track)
                
        self.tracks = remaining_tracks
        return confirmed_detections

    def detect(self, frame, pitch: float = 0.0, roll: float = 0.0) -> list:
        """
        Görüntüde duba algılar ve açı/mesafe hesaplar. Emniyet filtrelerinden geçirir.
        """
        if frame is None:
            return []
            
        # Dinamik çözünürlük uyarlaması (Hata: 159 çözümü)
        h, w = frame.shape[:2]
        if w != self.image_width or h != self.image_height:
            self.image_width = w
            self.image_height = h
            self.focal_length_px = (self.image_width / 2.0) / math.tan(self.hfov_rad / 2.0)
            logger.info(f"Kamera çözünürlüğü dinamik olarak güncellendi: {w}x{h}, Odak Uzaklığı: {self.focal_length_px:.1f} px")
            
        self.frame_count += 1
        
        # [Senaryo 5 Önlemi] Lens tıkanıklık kontrolü
        if self.check_lens_obstruction(frame):
            return []
            
        if self.use_fallback:
            raw_dets = self._detect_hsv(frame, pitch, roll)
        else:
            raw_dets = self._detect_yolo(frame, pitch, roll)
            
        # [Senaryo 6 Önlemi] Zamansal doğrulama filtresi uygula
        return self.temporal_filter(raw_dets)

    def _detect_yolo(self, frame, pitch: float = 0.0, roll: float = 0.0) -> list:
        """
        OpenCV DNN ile YOLOv8 ONNX modeli kullanarak çıkarım yapar.
        """
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (640, 640), swapRB=True, crop=False)
        self.net.setInput(blob)
        
        outputs = self.net.forward()
        outputs = np.transpose(outputs[0], (1, 0))
        
        boxes = []
        confidences = []
        class_ids = []
        
        for row in outputs:
            classes_scores = row[4:]
            class_id = np.argmax(classes_scores)
            confidence = classes_scores[class_id]
            
            if confidence >= self.conf_threshold:
                x_center, y_center, w, h = row[0:4]
                x_factor = self.image_width / 640.0
                y_factor = self.image_height / 640.0
                
                x = int((x_center - w/2) * x_factor)
                y = int((y_center - h/2) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                
                boxes.append([x, y, width, height])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        
        detections = []
        for i in indices:
            idx = i[0] if isinstance(i, (list, np.ndarray)) else i
            box = boxes[idx]
            class_id = class_ids[idx]
            conf = confidences[idx]
            
            distance, bearing, is_truncated = self.estimate_distance_and_bearing(box, pitch, roll)
            
            detections.append({
                "class": self.classes.get(class_id, "unknown"),
                "bbox": box,
                "confidence": conf,
                "distance": distance,
                "bearing": bearing,
                "is_truncated": is_truncated
            })
            
        return detections

    def _detect_hsv(self, frame, pitch: float = 0.0, roll: float = 0.0) -> list:
        """
        Yedek algılama mekanizması: HSV renk eşikleme ve kontur analizi.
        Dinamik HSV eşikleme: Işık ve bulut durumlarına göre S ve V alt limitleri uyarlanır (Görev 15).
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        
        # Görüntü ortalama parlaklığını hesapla
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        
        # Parlaklığa bağlı alt limit kaymaları (128 referans alınarak)
        # Hava karanlıksa/bulutluysa (mean_brightness < 128) offsetler negatif olur ve eşikler gevşetilir.
        v_offset = int((mean_brightness - 128.0) * 0.4)
        s_offset = int((mean_brightness - 128.0) * 0.2)
        
        color_ranges = {
            "orange_gate": [((5, 120, 100), (15, 255, 255)), ((165, 120, 100), (175, 255, 255))],
            "yellow_obstacle": [((20, 100, 100), (35, 255, 255))],
            "target_red": [((0, 150, 80), (10, 255, 255)), ((170, 150, 80), (180, 255, 255))],
            "target_green": [((40, 80, 80), (80, 255, 255))],
            "target_blue": [((100, 120, 80), (130, 255, 255))]
        }
        
        for name, ranges in color_ranges.items():
            mask = None
            for lower, upper in ranges:
                # Eşikleri parlaklığa göre dinamik uyarla
                low_h, low_s, low_v = lower
                up_h, up_s, up_v = upper
                
                # S alt sınırını 40, V alt sınırını 30'un altına düşürmeyecek şekilde sınırla
                adj_low_s = max(40, min(255, low_s + s_offset))
                adj_low_v = max(30, min(255, low_v + v_offset))
                
                m = cv2.inRange(hsv, np.array([low_h, adj_low_s, adj_low_v]), np.array([up_h, up_s, up_v]))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
                
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 150:
                    continue
                    
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                if aspect_ratio < 0.2 or aspect_ratio > 1.8:
                    continue
                
                # Görev 1.3: Dalga köpüğü gürültü filtresi (Extent / Doluluk oranı kontrolü)
                # Yuvarlak duba konturları yüksek doluluğa sahiptir (>0.45). Düzensiz köpükler ise elenir.
                extent = area / float(w * h)
                if extent < 0.45:
                    continue
                    
                box = [x, y, w, h]
                distance, bearing, is_truncated = self.estimate_distance_and_bearing(box, pitch, roll)
                
                detections.append({
                    "class": name,
                    "bbox": box,
                    "confidence": 0.85,
                    "distance": distance,
                    "bearing": bearing,
                    "is_truncated": is_truncated
                })
                
        return detections

    def estimate_distance_and_bearing(self, bbox: list, pitch: float = 0.0, roll: float = 0.0) -> tuple:
        """
        Duba piksel genişliğinden mesafe ve merkez sapmasından açı çıkarımı yapar.
        Görev 1.2 (Kenar kesilmesi telafisi) ve Görev 1.5 (Roll/Pitch yalpalama düzeltmesi) içerir.
        """
        x, y, w, h = bbox
        
        # Görev 1.2: Ekran sınır kontrolü (Truncation) tespiti
        # Bounding box sol veya sağ kenara 2 pikselden yakınsa kırpılmış kabul edilir.
        is_truncated = (x <= 2) or (x + w >= self.image_width - 2)
        
        # Eğer kırpılma varsa, mesafe patlamasını önlemek için genişlik yerine yüksekliği referans alıyoruz
        if is_truncated:
            w_px = max(1, h)
        else:
            w_px = max(1, w)
        
        # Ham mesafe hesabı
        distance = (self.focal_length_px * self.BUOY_REAL_WIDTH_M) / w_px
        
        # Görev 1.5: Yalpalama (pitch) açı telafisi
        # Tekne öne/arkaya şahlandığında perspektif sıkışmasını düzelt
        if abs(pitch) > 0.1:
            distance = distance * math.cos(math.radians(pitch))
            
        box_center_x = x + w / 2.0
        box_center_y = y + h / 2.0
        
        offset_x = box_center_x - (self.image_width / 2.0)
        offset_y = box_center_y - (self.image_height / 2.0)
        
        # Görev 1.5: Yalpalama (roll) açı telafisi
        # Tekne sola/sağa yattığında görüntünün dönmesini düzelt (koordinatları ters döndür)
        if abs(roll) > 0.1:
            roll_rad = math.radians(-roll) # Eksen dönüşü tersi yönünde
            offset_x_corr = offset_x * math.cos(roll_rad) - offset_y * math.sin(roll_rad)
            offset_x = offset_x_corr
            
        bearing = math.atan2(offset_x, self.focal_length_px)
        return distance, bearing, is_truncated

    def draw_detections(self, frame, detections: list):
        for det in detections:
            x, y, w, h = det["bbox"]
            label = f"{det['class']} ({det['confidence']:.2f})"
            dist_label = f"Dist: {det['distance']:.2f}m, Ang: {math.degrees(det['bearing']):.1f}deg"
            
            if "orange" in det["class"]:
                color = (0, 165, 255)
            elif "yellow" in det["class"]:
                color = (0, 255, 255)
            elif "red" in det["class"]:
                color = (0, 0, 255)
            elif "green" in det["class"]:
                color = (0, 255, 0)
            elif "blue" in det["class"]:
                color = (255, 0, 0)
            else:
                color = (255, 255, 255)
                
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(frame, dist_label, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return frame
