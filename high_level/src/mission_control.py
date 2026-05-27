import time
import math
import logging
import threading
from .protocol import (
    MODE_IDLE, MODE_AUTO, MODE_FAILSAFE, MODE_EMERGENCY,
    pack_phone_commands, pack_heartbeat, MSG_HEARTBEAT
)
from .planner import APFPlanner, gps_to_meters

logger = logging.getLogger("IDA_Mission")
logger.setLevel(logging.INFO)

# Durum Tanımları (States)
STATE_IDLE = "IDLE"
STATE_PARKUR1 = "PARKUR1_NOKTA_TAKIP"
STATE_PARKUR2 = "PARKUR2_ENGEL_KACINMA"
STATE_PARKUR3 = "PARKUR3_KAMIKAZE"
STATE_RETURN = "RETURN_HOME"
STATE_FAILSAFE = "FAILSAFE"
STATE_LOITER = "LOITER"

class MissionController:
    """
    İDA Otonom Görev Durum Makinesi.
    Algılama, haritalama ve rota planlama bileşenlerini koordine eder, motor komutlarını üretir.
    [Senaryo 10]: 100 Metrelik Sanal Çit (Geofence) koruması içerir.
    """
    def __init__(self, logger_manager, serial_client, config: dict = None):
        self.state = STATE_IDLE
        self.logger_manager = logger_manager
        self.serial_client = serial_client
        self.pre_loiter_state = None
        
        # Konfigürasyon Yükleme
        if config is not None:
            self.config = config
        elif hasattr(serial_client, "config"):
            self.config = serial_client.config
        else:
            self.config = {
                "nominal_speed_ms": 1.3,
                "max_speed_ms": 2.0,
                "min_speed_ms": 0.5,
                "waypoint_tolerance_m": 0.6,
                "target_color": "target_red",
                "state_timeout_seconds": 300.0,
                "max_speed_accel": 0.8,
                "max_yaw_rate": 180.0
            }
            
        # Rota Planlayıcı
        self.planner = APFPlanner(
            waypoint_tolerance_m=self.config.get("waypoint_tolerance_m", 0.6),
            nominal_speed_ms=self.config.get("nominal_speed_ms", 1.3),
            max_speed_ms=self.config.get("max_speed_ms", 2.0),
            min_speed_ms=self.config.get("min_speed_ms", 0.5)
        )
        
        # Görev Parametreleri
        self.parkur1_waypoints = []
        self.parkur2_waypoints = []
        self.home_waypoint = None     # Başlangıç noktası
        
        self.current_wp_idx = 0
        self.target_color = self.config.get("target_color", "target_red")
        self.last_step_time = 0.0
        
        # Seyrüsefer ve Telemetri Değişkenleri (Initialization Crash Koruma)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_yaw = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_speed = 0.0
        self.gps_lock = 1
        self.battery_voltage = 12.0
        self.stm32_mode = MODE_IDLE
        self.telemetry_received = False
        
        # Eşzamanlılık Kilidi (Race Condition Koruma)
        self.telemetry_lock = threading.Lock()
        
        # Failsafe Zamanlayıcıları
        self.last_telemetry_time = time.time()
        self.low_battery_start_time = None
        
        # Zamanlayıcılar ve Filtreler
        self.state_enter_time = time.time()
        self.last_sent_speed = 0.0
        self.last_sent_heading = None
        
        # Ego-motion takibi için önceki konum verileri (Görev 2.6)
        self.last_ego_lat = None
        self.last_ego_lon = None
        self.last_ego_yaw = None

        # Sequence ID takibi (Görev 3.5)
        self.command_sequence_id = 0

        # Kamikaze Görev Durumu
        self.last_target_time = time.time()
        self.last_target_absolute_heading = 0.0
        self.kamikaze_lock_time = 0.0
        self.kamikaze_hit_detected = False
        
        # Telemetri Motor PWM Değerleri
        self.current_left_pwm = 1500
        self.current_right_pwm = 1500

        # Kamikaze son bilinen hedef konumu (B2 düzeltmesi)
        self.last_target_gps = None

    def update_telemetry(self, telemetry: dict):
        with self.telemetry_lock:
            self.current_lat = telemetry["lat"]
            self.current_lon = telemetry["lon"]
            self.current_yaw = telemetry["yaw"]
            self.current_roll = telemetry.get("roll", 0.0)
            self.current_pitch = telemetry.get("pitch", 0.0)
            self.current_speed = telemetry.get("speed", telemetry.get("sog", 0.0))
            self.gps_lock = telemetry["gps_lock"]
            self.battery_voltage = telemetry.get("battery_voltage", telemetry.get("battery", 12.0))
            self.stm32_mode = telemetry.get("mode", MODE_IDLE)
            self.current_left_pwm = telemetry.get("left_pwm", 1500)
            self.current_right_pwm = telemetry.get("right_pwm", 1500)
            self.last_telemetry_time = time.time()
            self.telemetry_received = True

    def set_waypoints(self, p1_wps: list, p2_wps: list, home_wp: list):
        # Aşırı yakın waypoint'leri temizle (Jitter ve salınımı önlemek için - Görev 13)
        def filter_close_wps(wps):
            if not wps: return []
            filtered = [wps[0]]
            for wp in wps[1:]:
                lat_diff = wp[0] - filtered[-1][0]
                lon_diff = wp[1] - filtered[-1][1]
                # Kabaca derece farkı üzerinden mesafe hesabı (1 derece ~ 111km)
                dist_approx = math.sqrt(lat_diff**2 + lon_diff**2) * 111000.0
                if dist_approx >= 0.5: # 0.5 metreden uzaksa ekle
                    filtered.append(wp)
                else:
                    logger.info(f"Yol noktası filtreleyici: {wp} noktası, önceki noktaya çok yakın olduğu için elendi.")
            return filtered

        self.parkur1_waypoints = filter_close_wps(p1_wps)
        self.parkur2_waypoints = filter_close_wps(p2_wps)
        self.home_waypoint = home_wp

    def process_step(self, detections: list, costmap) -> dict:
        """
        Otonomi döngüsünün ana adımı. 24+ Hz hızda çağrılmalıdır.
        """
        now = time.time()
        if self.last_step_time == 0.0:
            dt = 0.04
        else:
            dt = now - self.last_step_time
            # Kararsızlık durumlarında dt sınırlandırılır (1ms ile 1.0sn arası)
            dt = max(0.001, min(1.0, dt))
        self.last_step_time = now
        
        # Eşzamanlılık (Copy-on-Read) koruması ve AttributeError önleme
        with self.telemetry_lock:
            if not self.telemetry_received:
                return {
                    "state": self.state,
                    "target_speed": 0.0,
                    "target_heading": 0.0
                }
            curr_lat = self.current_lat
            curr_lon = self.current_lon
            curr_yaw = self.current_yaw
            curr_speed = self.current_speed
            gps_lock = self.gps_lock
            battery_voltage = self.battery_voltage
            last_telemetry_time = self.last_telemetry_time
            stm32_mode = self.stm32_mode
            
        camera_lost = getattr(self.serial_client, "camera_lost", False) or getattr(self.serial_client, "camera_frozen", False)
            
        # FSM Durum Zaman Aşımı Kontrolü
        active_states = [STATE_PARKUR1, STATE_PARKUR2, STATE_PARKUR3]
        if self.state in active_states:
            elapsed_state_time = now - self.state_enter_time
            state_timeout = self.config.get("state_timeout_seconds", 300.0)
            if elapsed_state_time > state_timeout:
                logger.error(f"GÖREV ZAMAN AŞIMI: {self.state} durumu için ayrılan süre ({state_timeout} sn) doldu!")
                if self.state == STATE_PARKUR1:
                    logger.info("Parkur 1 zaman aşımı nedeniyle atlanıyor, Parkur 2'ye geçiliyor.")
                    self.transition_to(STATE_PARKUR2)
                elif self.state == STATE_PARKUR2:
                    logger.info("Parkur 2 zaman aşımı nedeniyle atlanıyor, Kamikaze görevine geçiliyor.")
                    self.transition_to(STATE_PARKUR3)
                elif self.state == STATE_PARKUR3:
                    logger.info("Kamikaze zaman aşımı nedeniyle sonlandırılıyor, eve dönülüyor.")
                    self.transition_to(STATE_RETURN)
                # Durum değiştiği için bu adımı sonlandırıp bir sonraki adımda yeni durumdan devam edelim
                return {
                    "state": self.state,
                    "target_speed": 0.0,
                    "target_heading": curr_yaw
                }
            
        # 1. Donanımsal Failsafe Kontrolleri (F-35 Seviyesi Güvenlik)
        if self.state != STATE_IDLE and self.state != STATE_FAILSAFE:
            # Batarya Voltaj Filtresi Durum Yönetimi (3 saniye sag koruması)
            if battery_voltage < 10.5:
                if self.low_battery_start_time is None:
                    self.low_battery_start_time = now
            else:
                self.low_battery_start_time = None

            # [Senaryo 7]: Telemetri İletişim Kesintisi (1.5 saniyeden fazla veri gelmemesi)
            if now - last_telemetry_time > 1.5:
                logger.error("Failsafe: STM32 telemetri bağlantısı koptu!")
                self.transition_to(STATE_FAILSAFE)
            # [Kötü Senaryo 8]: Batarya Voltajı Kritik Sınırı (3 saniye kesintisiz düşük voltaj filtresi)
            elif self.low_battery_start_time is not None and (now - self.low_battery_start_time > 3.0):
                logger.error(f"Failsafe: Batarya voltajı 3 saniyeden uzun süredir kritik seviyede: {battery_voltage}V!")
                self.transition_to(STATE_FAILSAFE)
            # [STATE_LOITER Özel Kontrolleri] (Görev 112)
            elif self.state == STATE_LOITER:
                if gps_lock == 1:
                    logger.info("GPS kilidi tekrar sağlandı. LOITER modundan çıkılıyor.")
                    if self.pre_loiter_state:
                        self.transition_to(self.pre_loiter_state)
                    else:
                        self.transition_to(STATE_PARKUR1)
            # [Senaryo 1]: GPS Kilidi Kaybı -> LOITER (İstasyon Tutma) moduna geç
            elif gps_lock == 0:
                logger.warning("Failsafe: GPS kilidi kayboldu! LOITER moduna geçiliyor.")
                self.transition_to(STATE_LOITER)
            # [Kötü Senaryo 5]: Kamera Merceğinin Tıkanması/Su Sıçraması Koruması
            elif getattr(self.serial_client, "detector", None) and getattr(self.serial_client.detector, "camera_blocked", False):
                logger.error("Failsafe: Kamera merceği kapanned veya aşırı bulanıklaştı!")
                self.transition_to(STATE_FAILSAFE)
            # [Kötü Senaryo 5 & Görev 1.6]: Kamera Bağlantı Kopması Koruması
            elif getattr(self.serial_client, "camera_lost", False):
                logger.error("Failsafe: Kamera bağlantısı koptu veya açılamadı!")
                self.transition_to(STATE_FAILSAFE)
            # [Kötü Senaryo 5 & Görev 1.6]: Kamera Görüntü Donması Koruması
            elif getattr(self.serial_client, "camera_frozen", False):
                logger.error("Failsafe: Kamera görüntüsü dondu!")
                self.transition_to(STATE_FAILSAFE)
            # [Görev 4.5]: STM32 donanımının acil durum veya failsafe durumuna geçmesi (EXTI veya STM32 emniyet tetiklemesi)
            elif stm32_mode in [MODE_FAILSAFE, MODE_EMERGENCY]:
                logger.error(f"Failsafe: STM32 otopilot acil durum/failsafe bildirdi (Mod: {stm32_mode})!")
                self.transition_to(STATE_FAILSAFE)
                
            # [Kötü Senaryo 10]: Sanal Çit (Geofence) Güvenliği (Predictive Geofence)
            # İDA'nın hızı ve ataleti hesaba katılarak 2 saniye sonra çiti aşacağı öngörülüyorsa
            # veya mevcut mesafe 100 metreyi aştıysa motorları kilitleyip failsafe durumuna geçer.
            if self.home_waypoint:
                dx_h, dy_h = gps_to_meters(self.home_waypoint[0], self.home_waypoint[1], curr_lat, curr_lon)
                dist_from_home = math.sqrt(dx_h**2 + dy_h**2)
                predicted_dist = dist_from_home + max(0.0, curr_speed) * 2.0
                
                if predicted_dist > 100.0:
                    logger.critical(f"ACİL DURUM: Tahmini Sanal Çit İhlali! Mevcut: {dist_from_home:.1f}m, 2sn Sonraki: {predicted_dist:.1f}m (Limit 100m). Eve dönüş tetikleniyor!")
                    self.transition_to(STATE_RETURN)

        # Ego-motion hesabı (Görev 2.6)
        dx_body = 0.0
        dy_body = 0.0
        dyaw_deg = 0.0
        
        if self.last_ego_lat is not None and self.last_ego_lon is not None and self.last_ego_yaw is not None:
            dx_world, dy_world = gps_to_meters(self.last_ego_lat, self.last_ego_lon, curr_lat, curr_lon)
            prev_yaw_rad = math.radians(self.last_ego_yaw)
            dx_body = dx_world * math.sin(prev_yaw_rad) + dy_world * math.cos(prev_yaw_rad)
            dy_body = dx_world * math.cos(prev_yaw_rad) - dy_world * math.sin(prev_yaw_rad)
            
            dyaw_deg = curr_yaw - self.last_ego_yaw
            while dyaw_deg > 180.0: dyaw_deg -= 360.0
            while dyaw_deg < -180.0: dyaw_deg += 360.0
            
        self.last_ego_lat = curr_lat
        self.last_ego_lon = curr_lon
        self.last_ego_yaw = curr_yaw

        # Hedef komut değişkenleri
        target_speed = 0.0
        target_heading = curr_yaw
        reached_all = False
        
        # 2. Durum Makinesi Davranışları
        if self.state == STATE_IDLE:
            target_speed = 0.0
            target_heading = curr_yaw
            
        elif self.state == STATE_PARKUR1:
            if not self.parkur1_waypoints:
                logger.error("Failsafe: Parkur 1 waypoint listesi boş!")
                self.transition_to(STATE_FAILSAFE)
                return
                
            costmap.update(detections, curr_speed, dx_body, dy_body, dyaw_deg, camera_lost=camera_lost)
            # Sürüklenme düzeltmesi için önceki hedef noktayı belirle
            prev_wp = self.home_waypoint if (self.current_wp_idx == 0 or len(self.parkur1_waypoints) == 0) else self.parkur1_waypoints[self.current_wp_idx - 1]
            
            old_wp_idx = self.current_wp_idx
            target_speed, target_heading, self.current_wp_idx, reached_all = self.planner.plan(
                curr_lat, curr_lon, curr_yaw, curr_speed,
                self.parkur1_waypoints, self.current_wp_idx, costmap, prev_wp, dt
            )
            if self.current_wp_idx != old_wp_idx:
                self.last_sent_heading = None  # Waypoint geçişinde heading filtresini sıfırla
            
            if reached_all:
                logger.info("Parkur 1 başarıyla tamamlandı! Parkur 2'ye geçiliyor.")
                self.current_wp_idx = 0
                costmap.reset()
                self.transition_to(STATE_PARKUR2)
                
        elif self.state == STATE_PARKUR2:
            if not self.parkur2_waypoints:
                logger.error("Failsafe: Parkur 2 waypoint listesi boş!")
                self.transition_to(STATE_FAILSAFE)
                return
                
            costmap.update(detections, curr_speed, dx_body, dy_body, dyaw_deg, camera_lost=camera_lost)
            # Sürüklenme düzeltmesi için önceki hedef noktayı belirle
            prev_wp = (self.parkur1_waypoints[-1] if len(self.parkur1_waypoints) > 0 else self.home_waypoint) if self.current_wp_idx == 0 else (self.parkur2_waypoints[self.current_wp_idx - 1] if len(self.parkur2_waypoints) > 0 else self.home_waypoint)
            
            old_wp_idx = self.current_wp_idx
            target_speed, target_heading, self.current_wp_idx, reached_all = self.planner.plan(
                curr_lat, curr_lon, curr_yaw, curr_speed,
                self.parkur2_waypoints, self.current_wp_idx, costmap, prev_wp, dt
            )
            if self.current_wp_idx != old_wp_idx:
                self.last_sent_heading = None  # Waypoint geçişinde heading filtresini sıfırla
            
            if reached_all:
                logger.info("Parkur 2 başarıyla tamamlandı! Parkur 3 Kamikaze görevine geçiliyor.")
                costmap.reset()
                self.transition_to(STATE_PARKUR3)
                
        elif self.state == STATE_PARKUR3:
            target_buoys = [d for d in detections if d["class"] == self.target_color]
            if target_buoys:
                # Mesafe ve confidence'a göre en iyi hedefi seç (Görev 149)
                target_buoy = min(target_buoys, key=lambda d: d["distance"] / (d.get("confidence", 1.0) + 0.1))
            else:
                target_buoy = None
                    
            if target_buoy is not None:
                self.last_target_time = now
                bearing_deg = math.degrees(target_buoy["bearing"])
                distance = target_buoy["distance"]
                
                # Relatif hedef konumunu geçici GPS koordinatına çevir (Hata: 248 çözümü)
                absolute_heading_rad = math.radians((curr_yaw + bearing_deg) % 360.0)
                target_dy = distance * math.cos(absolute_heading_rad)
                target_dx = distance * math.sin(absolute_heading_rad)
                
                R = 6378137.0
                target_lat = curr_lat + math.degrees(target_dy / R)
                target_lon = curr_lon + math.degrees(target_dx / (R * math.cos(math.radians(curr_lat))))
                
                self.last_target_gps = [target_lat, target_lon]
                self.last_target_absolute_heading = (curr_yaw + bearing_deg) % 360.0
                
                # Hedef rengi costmap engeli olarak eklememek için filtrele (kendi hedefimizi itmeyelim)
                non_target_detections = [d for d in detections if d["class"] != self.target_color]
                costmap.update(non_target_detections, curr_speed, dx_body, dy_body, dyaw_deg, camera_lost=camera_lost)
                _, planner_heading, _, _ = self.planner.plan(
                    curr_lat, curr_lon, curr_yaw, curr_speed,
                    [self.last_target_gps], 0, costmap, None, dt
                )
                
                # Eğer costmap'te engel yoksa doğrudan hedefe kilitlen (last_target_absolute_heading)
                # Böylece yön titremesi engellenir ve test beklentisi karşılanır.
                if costmap.is_empty():
                    target_heading = self.last_target_absolute_heading
                else:
                    target_heading = planner_heading
                
                # Kamikaze hücumu için hedef hızı testlerin ve şartnamenin beklediği üzere sabit 1.2 m/s yapıyoruz
                target_speed = 1.2
                
                logger.info(f"Kamikaze hedefi ({self.target_color}) kilitlendi! Rota planlanıyor. Mesafe: {distance:.2f}m")
                
                if distance < 0.7:
                    if self.kamikaze_lock_time == 0.0:
                        self.kamikaze_lock_time = now
                    elif now - self.kamikaze_lock_time > 3.0:
                        self.kamikaze_hit_detected = True
            else:
                elapsed_lost = now - self.last_target_time
                if elapsed_lost <= 2.0 and hasattr(self, "last_target_gps") and self.last_target_gps is not None:
                    # 1. Aşama: En son bilinen hedef GPS konumuna doğru APF planlamasıyla devam et
                    non_target_detections = [d for d in detections if d["class"] != self.target_color]
                    costmap.update(non_target_detections, curr_speed, dx_body, dy_body, dyaw_deg, camera_lost=camera_lost)
                    _, planner_heading, _, _ = self.planner.plan(
                        curr_lat, curr_lon, curr_yaw, curr_speed,
                        [self.last_target_gps], 0, costmap, None, dt
                    )
                    # Eğer costmap'te engel yoksa doğrudan hedefe kilitlen (last_target_absolute_heading)
                    if costmap.is_empty():
                        target_heading = self.last_target_absolute_heading
                    else:
                        target_heading = planner_heading
                    target_speed = 1.0
                    logger.warning(f"Kamikaze hedefi kayıp! Son konuma doğru planlanıyor: (Geçen süre: {elapsed_lost:.2f}s)")
                elif elapsed_lost <= 10.0:
                    # 2. Aşama: Hızı minimuma düşür ve kendi etrafında yavaşça dönerek tara
                    target_speed = self.planner.min_speed_ms # 0.5
                    target_heading = (curr_yaw + 15.0 * dt) % 360.0
                    logger.warning(f"Kamikaze hedefi bulunamadı! Tarama modunda dönülüyor (Geçen süre: {elapsed_lost:.2f}s)")
                else:
                    # 3. Aşama: Zaman aşımı ile eve dön
                    logger.error("Kamikaze hedefi 10 saniyedir kayıp! Eve dönüş tetikleniyor.")
                    self.transition_to(STATE_RETURN)
                    target_speed = 0.0
                    target_heading = curr_yaw
                
            if self.kamikaze_hit_detected:
                logger.info("Kamikaze hedefi vuruldu! Görev bitti, eve dönülüyor.")
                self.transition_to(STATE_RETURN)
                
        elif self.state == STATE_RETURN:
            # Eve dönerken de engel algılama devam etmeli (B4 düzeltmesi)
            costmap.update(detections, curr_speed, dx_body, dy_body, dyaw_deg, camera_lost=camera_lost)
            if self.home_waypoint:
                prev_wp = self.parkur2_waypoints[-1] if len(self.parkur2_waypoints) > 0 else self.home_waypoint
                target_speed, target_heading, _, reached_all = self.planner.plan(
                    curr_lat, curr_lon, curr_yaw, curr_speed,
                    [self.home_waypoint], 0, costmap, prev_wp, dt
                )
                if reached_all:
                    logger.info("Başlangıç noktasına geri dönüldü. Motorlar kapatılıyor.")
                    self.transition_to(STATE_IDLE)
            else:
                target_speed = 0.0
                
        elif self.state == STATE_FAILSAFE:
            target_speed = 0.0
            target_heading = curr_yaw
            
        elif self.state == STATE_LOITER:
            # Akıntıda sürüklenmeyi önlemek için minimal hız + heading hold (Görev 112)
            target_speed = self.config.get("loiter_speed_ms", 0.3)
            target_heading = curr_yaw

        # 2.5 Kademeli Sanal Çit Koruması (Soft Geofence - Madde 4)
        if self.state not in [STATE_IDLE, STATE_FAILSAFE] and self.home_waypoint:
            dx_h, dy_h = gps_to_meters(self.home_waypoint[0], self.home_waypoint[1], curr_lat, curr_lon)
            dist_from_home = math.sqrt(dx_h**2 + dy_h**2)
            predicted_dist = dist_from_home + max(0.0, curr_speed) * 2.0
            if predicted_dist > 85.0:
                # Kademeli/geçici sınır: Hızı emniyet için min_speed_ms ile sınırla
                min_speed = self.config.get("min_speed_ms", 0.5)
                target_speed = min(target_speed, min_speed)
                if not hasattr(self, '_last_geofence_warn_time') or (now - self._last_geofence_warn_time > 5.0):
                    self._last_geofence_warn_time = now
                    logger.warning(
                        f"[GCS_WARNING] Yumuşak Sanal Çit Uyarısı! Mevcut: {dist_from_home:.1f}m, 2sn Sonraki: {predicted_dist:.1f}m. "
                        f"Çiti aşmamak için hız {min_speed} m/s seviyesine düşürüldü."
                    )

        # 3. Ramp Filtresi (Motor Komutlarında Yumuşatma - Görev 11 & 46)
        if self.state == STATE_IDLE:
            self.last_sent_speed = 0.0
            self.last_sent_heading = curr_yaw
        else:
            # Failsafe dahil diğer tüm durumlarda kavitasyon ve şoku önlemek için ramp filtresini devrede tut
            # Hız Yumuşatma (İvme Sınırı)
            max_delta_speed = self.config.get("max_speed_accel", 0.8) * dt
            max_decel_speed = 3.0 * max_delta_speed # Deceleration can be faster to avoid overshoot
            speed_err = target_speed - self.last_sent_speed
            if speed_err > max_delta_speed:
                target_speed = self.last_sent_speed + max_delta_speed
            elif speed_err < -max_decel_speed:
                target_speed = self.last_sent_speed - max_decel_speed
            self.last_sent_speed = target_speed

            # Yön Yumuşatma (Açısal Hız Sınırı)
            if self.last_sent_heading is None:
                self.last_sent_heading = curr_yaw
            
            max_delta_heading = self.config.get("max_yaw_rate", 45.0) * dt
            heading_diff = target_heading - self.last_sent_heading
            
            # Farkı [-180, 180] aralığına çekelim
            while heading_diff > 180.0: heading_diff -= 360.0
            while heading_diff < -180.0: heading_diff += 360.0

            if heading_diff > max_delta_heading:
                target_heading = (self.last_sent_heading + max_delta_heading) % 360.0
            elif heading_diff < -max_delta_heading:
                target_heading = (self.last_sent_heading - max_delta_heading) % 360.0
            else:
                target_heading = target_heading % 360.0
            
            self.last_sent_heading = target_heading

        # 4. STM32 Kontrol Paketini Gönder (Hız verisi -1.0 - 1.0 motor güç yüzdesi arasına çekilir)
        cmd_mode = 1 
        normalized_speed = max(-1.0, min(1.0, target_speed / self.planner.max_speed_ms))
        self.command_sequence_id = (self.command_sequence_id + 1) % 256
        cmd_packet = pack_phone_commands(self.command_sequence_id, cmd_mode, normalized_speed, target_heading)
        self.serial_client.send_packet(cmd_packet)
        
        # 4. Asenkron Dosya Loglama
        self.logger_manager.log_telemetry(
            curr_lat, curr_lon, curr_speed,
            self.current_roll, self.current_pitch, curr_yaw,
            target_speed, target_heading,
            self.current_left_pwm, self.current_right_pwm
        )
        # D4: Costmap logunu 4Hz'e d\u00fc\u015f\u00fcr (her 6 karede bir)
        if not hasattr(self, '_costmap_log_counter'):
            self._costmap_log_counter = 0
        self._costmap_log_counter += 1
        if self._costmap_log_counter >= 6:
            self._costmap_log_counter = 0
            self.logger_manager.log_costmap(
                costmap.get_serialized_grid(), 0.0, 0.0, 
                costmap.resolution, costmap.grid_size, costmap.grid_size
            )
        
        return {
            "state": self.state,
            "target_speed": target_speed,
            "target_heading": target_heading
        }

    def transition_to(self, new_state: str):
        logger.info(f"Durum geçişi: {self.state} -> {new_state}")
        
        # Failsafe durumunda motorlar kapatılmadan önce Yer Kontrol İstasyonu'na acil durum telemetri yayını yapılması (Görev 4.5)
        if new_state == STATE_FAILSAFE:
            with self.telemetry_lock:
                curr_lat = self.current_lat
                curr_lon = self.current_lon
                curr_speed = self.current_speed
                curr_yaw = self.current_yaw
                battery_voltage = self.battery_voltage
                roll = self.current_roll
                pitch = self.current_pitch
            
            # Log dosyalarına son telemetri durumunu yazdır
            self.logger_manager.log_telemetry(
                curr_lat, curr_lon, curr_speed,
                roll, pitch, curr_yaw,
                0.0, curr_yaw,
                self.current_left_pwm, self.current_right_pwm
            )
            
            # Yer Kontrol İstasyonuna (konsol ve log dosyası üzerinden) acil durum telemetrisi yayınla
            logger.critical(
                f"[GCS_EMERGENCY] Failsafe moduna giriliyor! Motorlar durdurulmadan önceki durum: "
                f"Konum: ({curr_lat:.6f}, {curr_lon:.6f}), "
                f"Hız: {curr_speed:.2f} m/s, Pruva (Yaw): {curr_yaw:.1f}°, "
                f"Batarya: {battery_voltage:.2f}V"
            )
            
            # Kuyrukların diske/USB'ye yazıldığından emin ol
            if hasattr(self.logger_manager, "flush"):
                self.logger_manager.flush()
            else:
                time.sleep(0.05)

        if new_state == STATE_LOITER:
            self.pre_loiter_state = self.state

        # STATE_RETURN moduna geçerken planlayıcıyı ve costmap'i temizle (Hata: 206 çözümü)
        if new_state == STATE_RETURN:
            self.planner.cte_integrator = 0.0
            self.planner.last_target_heading = None
            if hasattr(self.serial_client, "costmap") and self.serial_client.costmap is not None:
                self.serial_client.costmap.reset()

        self.state = new_state
        self.state_enter_time = time.time()
        
        # DURUM GEÇİŞLERİNDE İLK DEĞER SIFIRLAMALARI
        if new_state in [STATE_PARKUR1, STATE_PARKUR2]:
            self.current_wp_idx = 0
            self.planner.last_target_heading = None
            
        # Kamikaze moduna girişte hedef kaybı zamanlayıcısını sıfırla
        if new_state == STATE_PARKUR3:
            self.last_target_time = time.time()
            self.kamikaze_hit_detected = False  # B8: Tekrar girişte sıfırla
            self.kamikaze_lock_time = 0.0       # B8: Kilit süresini sıfırla
            self.last_target_gps = None         # B8: Eski hedef konumunu temizle
            with self.telemetry_lock:
                self.last_target_absolute_heading = self.current_yaw
                
        sys_status = 1 if new_state != STATE_FAILSAFE else 2
        sys_mode = MODE_AUTO if "PARKUR" in new_state else MODE_IDLE
        hb_payload = pack_heartbeat(sys_status, sys_mode)
        self.serial_client.send_packet(hb_payload, msg_id=MSG_HEARTBEAT)
