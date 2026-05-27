import math
import logging

logger = logging.getLogger("IDA_Planner")
logger.setLevel(logging.INFO)

def gps_to_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple:
    """
    WGS-84 referans elipsoidi kullanarak iki GPS koordinatı arasındaki mesafeyi metre cinsinden hesaplar.
    dx: Doğu (East) yönünde mesafe (m)
    dy: Kuzey (North) yönünde mesafe (m)
    """
    lat_avg = math.radians((lat1 + lat2) / 2.0)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    R = 6378137.0
    dy = dlat * R
    dx = dlon * R * math.cos(lat_avg)
    return dx, dy

class APFPlanner:
    """
    Yapay Potansiyel Alanlar (Artificial Potential Field) rota planlayıcı.
    Hedefe doğru çekici kuvvet (attractive), engellerden zıt yönde itici kuvvet (repulsive) üretir.
    Akıntı sürüklenmesine karşı enine sapma entegral (Cross-Track Error Integral) terimini içerir (Senaryo 4).
    """
    def __init__(self, waypoint_tolerance_m: float = 2.5, 
                 nominal_speed_ms: float = 1.5, 
                 max_speed_ms: float = 2.5,
                 min_speed_ms: float = 0.5):
        
        self.waypoint_tolerance_m = waypoint_tolerance_m
        self.nominal_speed_ms = nominal_speed_ms
        self.max_speed_ms = max_speed_ms
        self.min_speed_ms = min_speed_ms
        
        # Çekici ve itici kuvvet katsayıları
        self.K_attractive = 2.0
        
        # --- Akıntı ve Rüzgar Sapma Düzeltmesi (Cross-Track Error) ---
        self.K_cte_i = 0.05       # CTE Entegral kazancı (sürüklenme düzeltmesi)
        self.cte_integrator = 0.0 # Birikmiş sürüklenme hatası
        self.max_cte_i = 1.5     # Maksimum düzeltme doyumu (windup engelleme)

        # --- Rota Yön Yumuşatma Filtresi (EMA) ---
        self.last_target_heading = None
        self.heading_ema_alpha = 0.25 # Açısal yumuşatma katsayısı (0.25 = düşük geçiren filtre)

        # --- GPS Gürültü Filtresi (Ucuz GPS sapmaları için dairesel tampon) ---
        self.gps_history_lat = []
        self.gps_history_lon = []
        self.gps_filter_size = 5

    def plan(self, current_lat: float, current_lon: float, current_yaw_deg: float, current_speed: float,
             waypoints: list, current_wp_idx: int, costmap, prev_wp_gps: list = None, dt: float = 0.04) -> tuple:
        """
        Rota ve hız planlaması yapar.
        """
        if not waypoints or current_wp_idx >= len(waypoints):
            return 0.0, current_yaw_deg, current_wp_idx, True
            
        # Ucuz GPS gürültüsünü sönümlemek için konum verisini hareketli ortalamayla filtrele
        # 15 karelik pencere (yaklaşık 0.6 sn) 2 metrelik ani sapmaları çok daha iyi emer
        if not hasattr(self, 'filtered_lat_ema'):
            self.filtered_lat_ema = current_lat
            self.filtered_lon_ema = current_lon
            self.gps_filter_size = 15
            
        self.gps_history_lat.append(current_lat)
        self.gps_history_lon.append(current_lon)
        if len(self.gps_history_lat) > self.gps_filter_size:
            self.gps_history_lat.pop(0)
            self.gps_history_lon.pop(0)
            
        avg_lat = sum(self.gps_history_lat) / len(self.gps_history_lat)
        avg_lon = sum(self.gps_history_lon) / len(self.gps_history_lon)
        
        # EMA + Hareketli Ortalama hibrit filtre (Gürültüyü çok iyi keser)
        alpha_gps = 0.3
        self.filtered_lat_ema = alpha_gps * avg_lat + (1 - alpha_gps) * self.filtered_lat_ema
        self.filtered_lon_ema = alpha_gps * avg_lon + (1 - alpha_gps) * self.filtered_lon_ema
        
        filtered_lat = self.filtered_lat_ema
        filtered_lon = self.filtered_lon_ema

        target_lat, target_lon = waypoints[current_wp_idx]
        
        # 1. Hedef noktaya olan mesafeyi ve bağıl konumu metre cinsinden hesapla
        dx_m, dy_m = gps_to_meters(filtered_lat, filtered_lon, target_lat, target_lon)
        dist_to_wp = math.sqrt(dx_m**2 + dy_m**2)

        # Rota hedefinin costmap üzerindeki hücresinin maliyetini kontrol et (Görev 108)
        # Hedefin İDA'ya göre bağıl body-frame koordinatlarını hesapla (Görev 2.6 ile uyumlu)
        yaw_rad = math.radians(current_yaw_deg)
        x_body_raw = dx_m * math.sin(yaw_rad) + dy_m * math.cos(yaw_rad)
        y_body_raw = dx_m * math.cos(yaw_rad) - dy_m * math.sin(yaw_rad)
        
        wp_row = costmap.center_idx - int(x_body_raw / costmap.resolution) if hasattr(costmap, "center_idx") else -1
        wp_col = costmap.center_idx + int(y_body_raw / costmap.resolution) if hasattr(costmap, "center_idx") else -1
        
        active_tolerance = self.waypoint_tolerance_m
        if hasattr(costmap, "grid_obstacles") and 0 <= wp_row < costmap.grid_size and 0 <= wp_col < costmap.grid_size:
            wp_cost = costmap.grid_obstacles[wp_row, wp_col]
            if wp_cost > 45: # Hedef duba/engel üzerinde veya çok yakınında
                active_tolerance = max(active_tolerance, 2.2) # Toleransı 2.2 metreye genişlet
                logger.warning(f"Hedef yol noktası ({target_lat}, {target_lon}) sarı engel bölgesinde (maliyet={wp_cost})! Geçiş toleransı {active_tolerance:.1f}m yapıldı.")

        # Noktaya ulaşıldı mı kontrolü
        reached = False
        if dist_to_wp < 0.6:  # Güvenli yakınlık yedek kontrolü
            logger.info(f"Waypoint {current_wp_idx} çok yakın toleransla ulaşıldı! Bir sonraki noktaya geçiliyor.")
            reached = True
        elif prev_wp_gps:
            # Geçiş düzlemi kontrolü (perpendicular plane crossing):
            # İki waypoint arasındaki rota çizgisine göre hedefin geçilip geçilmediğini kontrol eder.
            # Böylece kapılardan geçişte erken dönüp kapıyı kaçırma (corner cutting) engellenir.
            line_dx, line_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], target_lat, target_lon)
            line_len = math.sqrt(line_dx**2 + line_dy**2)
            if line_len > 1.0:
                u_x = line_dx / line_len
                u_y = line_dy / line_len
                boat_dx, boat_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], filtered_lat, filtered_lon)
                along_track = boat_dx * u_x + boat_dy * u_y
                
                # Enine sapma mesafesi (Cross-Track Error)
                cte = boat_dx * (-u_y) + boat_dy * u_x
                
                # Kapı çizgisini / dikey düzlemi geçtik mi? 
                # Yanal sapmalarda sonsuz döngüyü önlemek için along-track kontrolü.
                # Ancak yanal sapmanın çok büyük olmaması (abs(cte) < 4.0) gerekir (Görev 7).
                if along_track >= line_len and abs(cte) < 4.0:
                    logger.info(f"Waypoint {current_wp_idx} geçiş düzlemi (along-track={along_track:.2f}m >= limit={line_len:.2f}m, yanal={cte:.2f}m) üzerinden başarıyla ulaşıldı!")
                    reached = True
                    
        # Hiçbir referans yoksa veya fallback olarak normal/aktif toleransı kullan
        if not reached and dist_to_wp < active_tolerance:
            logger.info(f"Waypoint {current_wp_idx} standart tolerans ({dist_to_wp:.2f}m < {active_tolerance:.1f}m) ile ulaşıldı!")
            reached = True
            
        if reached:
            current_wp_idx += 1
            self.cte_integrator = 0.0 # Yeni hedef noktada sürüklenme entegralini sıfırla
            self.last_target_heading = None
            return self.min_speed_ms, current_yaw_deg, current_wp_idx, (current_wp_idx >= len(waypoints))

        # 2. [Senaryo 4 Önlemi]: Enine Sapma (Cross-Track Error) Hesaplama ve Entegral Düzeltmesi
        # İki nokta arasındaki ideal rota çizgisine olan dikey sapmayı hesaplar.
        cte_offset_x = 0.0
        cte_offset_y = 0.0
        
        if prev_wp_gps:
            # Önceki WP ile hedef WP arasındaki rota hattı vektörü (Metre cinsinden)
            line_dx, line_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], target_lat, target_lon)
            line_len = math.sqrt(line_dx**2 + line_dy**2)
            
            if line_len > 1.0:
                # İdeal rota hattının birim vektörü
                u_x = line_dx / line_len
                u_y = line_dy / line_len
                
                # Botun önceki WP'ye göre bağıl konumu (Metre)
                boat_dx, boat_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], filtered_lat, filtered_lon)
                
                # Enine sapma mesafesi (Cross-Track Error) - Rota hattına dik olan mesafe
                # Vektörel çarpım (2D cross product): boat_vector x line_unit_vector
                cte = boat_dx * (-u_y) + boat_dy * u_x
                
                # Rota çizgisi ortasından geçişte (sign change) integral birikimini sıfırla veya sönümle 
                # (Over-shooting ve salınımı engellemek için - Görev 85 & 136)
                if (cte > 0.0 and self.cte_integrator < 0.0) or (cte < 0.0 and self.cte_integrator > 0.0):
                    self.cte_integrator *= 0.25 # Hızlı sönümleme
                
                # Enine sapma yönünde entegral düzeltme biriktir (dinamik dt kullanılır)
                self.cte_integrator += cte * dt
                # Anti-windup koruması (Limit 1.5m)
                self.cte_integrator = max(-self.max_cte_i, min(self.cte_integrator, self.max_cte_i))
                
                # İntegral düzeltme ile akıntı sürüklenmesini kararlı şekilde düzelt
                cte_correction = self.cte_integrator * self.K_cte_i
                
                # Düzeltme yönü ideal rotaya çekmek için hattın dik birim vektörüdür (Sürüklenmenin tersi)
                cte_offset_x = (u_y) * cte_correction
                cte_offset_y = (-u_x) * cte_correction

        # 3. Koordinat Dönüşümü: Doğu-Kuzey (EN) koordinatlarından Bot Gövde Koordinatlarına (Body Frame)
        yaw_rad = math.radians(current_yaw_deg)
        
        # Çekici kuvvet yönüne enine sapma düzeltmesini ekle (Dünya koordinatlarında)
        total_dx = dx_m + cte_offset_x
        total_dy = dy_m + cte_offset_y
        total_dist = math.sqrt(total_dx**2 + total_dy**2)
        
        # Bot gövde eksenindeki ileri (x_body) ve sağ (y_body) çekici yön
        x_body = total_dx * math.sin(yaw_rad) + total_dy * math.cos(yaw_rad)
        y_body = total_dx * math.cos(yaw_rad) - total_dy * math.sin(yaw_rad)
        
        # 4. Çekici Kuvvet (Attractive Force) Hesabı
        if total_dist > 0.1:
            att_x = self.K_attractive * (x_body / total_dist)
            att_y = self.K_attractive * (y_body / total_dist)
        else:
            att_x, att_y = 0.0, 0.0
            
        # 5. İtici Kuvvet (Repulsive Force) Hesabı
        rep_x, rep_y = costmap.get_obstacle_forces()
        
        # 6. APF Yerel Minimum (Local Minima) Kilitlerini Çözmek İçin Sanal Teğet Kuvveti - Görev 2.1
        rep_dist = math.sqrt(rep_x**2 + rep_y**2)
        att_dist = math.sqrt(att_x**2 + att_y**2)
        
        total_force_x = att_x + rep_x
        total_force_y = att_y + rep_y
        total_force_mag = math.sqrt(total_force_x**2 + total_force_y**2)
        
        stuck = False
        # Rota hedefinden uzakken bileşke kuvvetin sıfırlanıp kalması durumunda (Local Minima - Görev 2.1)
        if total_force_mag < 0.15 and dist_to_wp > 1.5:
            if att_dist > 0.1 and rep_dist > 0.1:
                cos_theta = (att_x * rep_x + att_y * rep_y) / (att_dist * rep_dist)
                if cos_theta < -0.90: # Zıt yönlü ve birbirini neredeyse tamamen sönümleyen kuvvetler
                    stuck = True
            else:
                stuck = True
            
        if stuck and att_dist > 0.001:
            # Sağ tarafın kapalı olup olmadığını costmap'ten sorgula (Görev 2.3 ile entegre)
            right_blocked = False
            if hasattr(costmap, "is_right_blocked"):
                right_blocked = costmap.is_right_blocked()
            
            # Sanal teğet kuvveti büyüklüğü
            pert_mag = 1.8
            if not right_blocked:
                # Sağa doğru teğetsel kuvvet (+y_body yönünde döndürür)
                pert_x = -att_y * (pert_mag / att_dist)
                pert_y = att_x * (pert_mag / att_dist)
            else:
                # Sola doğru teğetsel kuvvet (-y_body yönünde döndürür)
                pert_x = att_y * (pert_mag / att_dist)
                pert_y = -att_x * (pert_mag / att_dist)
                
            total_force_x += pert_x
            total_force_y += pert_y
            logger.warning(f"APF Local Minima tespit edildi! Teğet kuvveti uygulandı: pert_x={pert_x:.2f}, pert_y={pert_y:.2f} (Sağ engelli={right_blocked})")
        
        # 7. Kontrol Komutları Üretimi
        steer_angle_rad = math.atan2(total_force_y, total_force_x)
        target_heading_deg = (current_yaw_deg + math.degrees(steer_angle_rad)) % 360.0
        
        # Rota Yön Yumuşatma Filtresi (EMA) - Görev 2.2
        if self.last_target_heading is None:
            self.last_target_heading = target_heading_deg
        else:
            diff = target_heading_deg - self.last_target_heading
            while diff > 180.0: diff -= 360.0
            while diff < -180.0: diff += 360.0
            self.last_target_heading = (self.last_target_heading + self.heading_ema_alpha * diff) % 360.0
            target_heading_deg = self.last_target_heading
            
        # Hız kontrolü:
        angle_factor = math.cos(steer_angle_rad)
        
        # Geri Vites (Reverse Thrust) Planlama Desteği - Görev 2.4
        # Eğer önümüzde engel varsa ve bizi geriye itiyorsa (rep_x < -0.5) geri git
        if rep_x < -0.5:
            # Geri Vites: APF kuvvetini hız birimine dönüştür (kuvvet → m/s)
            reverse_speed = max(-self.nominal_speed_ms, -self.nominal_speed_ms * min(1.0, abs(rep_x) / 3.0))
            target_speed = max(-self.nominal_speed_ms, min(reverse_speed, self.max_speed_ms))
        else:
            if angle_factor < 0:
                # Keskin dönüşlerde akıntı sürüklenmesini engellemek için steerage way'i koruyacak şekilde 
                # asgari hızı biraz daha yüksek tutuyoruz (Görev 9)
                target_speed = max(self.min_speed_ms, 0.65) 
            else:
                target_speed = self.nominal_speed_ms * (angle_factor ** 2)
                if dist_to_wp < 5.0:
                    target_speed = min(target_speed, 0.5 + 0.2 * dist_to_wp)
                    
            target_speed = max(self.min_speed_ms, min(target_speed, self.max_speed_ms))
            
            # Eğer rotada hiç engel yoksa ve hedefe gidiyorsak tam hıza çıkabiliriz
            if abs(rep_x) < 0.1 and abs(rep_y) < 0.1 and dist_to_wp > 8.0:
                target_speed = self.nominal_speed_ms * max(self.min_speed_ms / self.nominal_speed_ms, angle_factor)
                
        return target_speed, target_heading_deg, current_wp_idx, False
