import numpy as np
import math
import logging
import cv2

logger = logging.getLogger("IDA_Costmap")
logger.setLevel(logging.INFO)

class LocalCostmap:
    """
    İDA merkezli çift katmanlı yerel engel haritası (Occupancy Grid / Costmap).
    Turuncu kapı dubaları ve Sarı engel dubaları için iki ayrı katman tutar.
    - Kapı Dubaları (Orange Gates): Kapının tam ortasından geçmek için simetrik itme uygular.
    - Sarı Engeller (Yellow Obstacles): COLREGs kuralları gereği sağdan (sancak) geçmek için asimetrik itme uygular.
    """
    def __init__(self, size_m: float = 40.0, resolution: float = 0.25, inflation_radius_m: float = 1.0):
        self.size_m = size_m
        self.resolution = resolution
        self.grid_size = int(size_m / resolution)
        self.center_idx = self.grid_size // 2
        
        # Çift Katmanlı Harita Izgarası
        self.grid_gates = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.grid_obstacles = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        # Birleşik görsel ızgara (Loglama ve görselleştirme için)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        self.base_inflation_radius_m = inflation_radius_m
        self.inflation_radius_cells = int(inflation_radius_m / resolution)
        self.decay_factor = 0.85
        self.min_cost_threshold = 15

    def is_right_blocked(self, max_dist_m: float = 6.0) -> bool:
        """
        Sağ tarafta (sancak) ve ön-sağda kıyı şeridi veya sığlık engellerini denetler.
        Eğer bu bölgedeki engel yoğunluğu eşiği aşarsa True döner.
        """
        r_limit = int(max_dist_m / self.resolution)
        blocked_cells = 0
        
        # Ön-sağ çeyrek daire (Ahead and Right):
        # dx_m >= 0.0 (ahead, dr <= 0), dy_m > 0.0 (right, dc > 0)
        for dr in range(-r_limit, 1):
            for dc in range(1, r_limit + 1):
                if dr**2 + dc**2 <= r_limit**2:
                    row = self.center_idx + dr
                    col = self.center_idx + dc
                    if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                        if self.grid_obstacles[row, col] > 35:
                            blocked_cells += 1
                            
        return blocked_cells > 5

    def update(self, detections: list, current_speed: float = 0.0, 
               dx_body: float = 0.0, dy_body: float = 0.0, dyaw_deg: float = 0.0,
               camera_lost: bool = False):
        """
        Gelen yeni duba tespitlerinin sınıfına göre ilgili katmanı günceller.
        Bot dönüşlerinde costmap rotasyon matrisi ve öteleme (Ego-motion compensation) uygular.
        İDA'nın hızına duyarlı dinamik şişirme yarıçapı kullanır.
        """
        # 1. Ego-motion Compensation (Ego-hareket telafisi) - Görev 2.6
        if abs(dx_body) > 0.001 or abs(dy_body) > 0.001 or abs(dyaw_deg) > 0.001:
            # Hücre cinsinden ötelemeler
            row_shift = dx_body / self.resolution
            col_shift = -dy_body / self.resolution
            
            # Dönüş ve öteleme matrisi oluştur (OpenCV koordinat sisteminde X=sütun, Y=satır)
            M = cv2.getRotationMatrix2D((self.center_idx, self.center_idx), dyaw_deg, 1.0)
            M[0, 2] += col_shift
            M[1, 2] += row_shift
            
            # Katmanları warp et
            self.grid_gates = cv2.warpAffine(self.grid_gates, M, (self.grid_size, self.grid_size), flags=cv2.INTER_NEAREST)
            self.grid_obstacles = cv2.warpAffine(self.grid_obstacles, M, (self.grid_size, self.grid_size), flags=cv2.INTER_NEAREST)

        # 2. Eski harita katmanlarını sönümle (Decay - Görev 150)
        decay = 0.70 if camera_lost else self.decay_factor
        self.grid_gates = (self.grid_gates * decay).astype(np.uint8)
        self.grid_gates[self.grid_gates < self.min_cost_threshold] = 0
        
        self.grid_obstacles = (self.grid_obstacles * decay).astype(np.uint8)
        self.grid_obstacles[self.grid_obstacles < self.min_cost_threshold] = 0
        
        # 3. Dinamik Şişirme Yarıçapı Hesaplama - Görev 2.5
        # Hız arttıkça şişirme yarıçapını doğrusal olarak artır (maksimum 2.5 metre limitli)
        dynamic_radius_m = self.base_inflation_radius_m + 0.5 * max(0.0, current_speed)
        dynamic_radius_m = min(2.5, dynamic_radius_m)
        self.inflation_radius_cells = int(dynamic_radius_m / self.resolution)
        
        # 4. Yeni tespitleri sınıflarına göre dağıt
        for det in detections:
            cls = det["class"]
            distance = det["distance"]
            bearing = det["bearing"]
            
            x_rel = distance * math.cos(bearing)
            y_rel = distance * math.sin(bearing)
            
            row = self.center_idx - int(x_rel / self.resolution)
            col = self.center_idx + int(y_rel / self.resolution)
            
            if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                if cls == "orange_gate":
                    self.grid_gates[row, col] = 100
                    self._inflate_obstacle(self.grid_gates, row, col)
                else:
                    # Sarı dubalar ve diğer hedefler engel kabul edilir
                    self.grid_obstacles[row, col] = 100
                    self._inflate_obstacle(self.grid_obstacles, row, col)
                    
        # Loglama ve uyumluluk için birleşik haritayı güncelle
        self.grid = np.maximum(self.grid_gates, self.grid_obstacles)

    def _inflate_obstacle(self, target_grid, row: int, col: int):
        r_cells = self.inflation_radius_cells
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                dist_cells = math.sqrt(dr**2 + dc**2)
                if dist_cells <= r_cells:
                    target_row = row + dr
                    target_col = col + dc
                    
                    if 0 <= target_row < self.grid_size and 0 <= target_col < self.grid_size:
                        cost = int(100 * (1.0 - (dist_cells / (r_cells + 1.0))))
                        current_cost = target_grid[target_row, target_col]
                        target_grid[target_row, target_col] = max(current_cost, cost)

    def get_obstacle_forces(self) -> tuple:
        """
        Yapay Potansiyel Alanlar (APF) için bileşke itici kuvveti hesaplar.
        - Kapı dubaları için tam simetrik itme (orta hattı korur).
        - Engeller için asimetrik (sağa kaçış) itme uygular.
        - Kıyı sınırı mantığı ile sağ taraf engelli ise asimetrik sağ büküm devre dışı bırakılır (Görev 2.3).
        """
        rep_x = 0.0
        rep_y = 0.0
        
        K_repulsive = 5.0
        influence_distance_gates = 2.5
        influence_distance_obstacles = 5.0
        
        # Sağ tarafın engel durumu (Görev 2.3)
        right_blocked = self.is_right_blocked(max_dist_m=6.0)
        
        # 1. Kapı Dubaları (Orange Gates) İtme Hesabı (Simetrik - Dubaların Ortasından Geçiş Sağlar)
        # O(N^2) tam ızgara taraması yerine sadece maliyeti min_cost_threshold'dan büyük hücreleri NumPy ile bulup döngüye sokuyoruz (C6 optimizasyonu)
        rows_g, cols_g = np.where(self.grid_gates > self.min_cost_threshold)
        for r, c in zip(rows_g, cols_g):
            cost = self.grid_gates[r, c]
            dx_m = (self.center_idx - r) * self.resolution
            dy_m = (c - self.center_idx) * self.resolution
            dist = math.sqrt(dx_m**2 + dy_m**2)
            if dist < 0.1: continue
            
            if dist <= influence_distance_gates:
                force_mag = K_repulsive * (cost / 100.0) * ((1.0 / dist) - (1.0 / influence_distance_gates)) * (1.0 / dist**2)
                # APF İtici Kuvvet Sınırsızlığı Koruması (Görev 61)
                force_mag = min(15.0, force_mag)
                
                # Simetrik itme (Botu tam zıt yöne iter, böylece sol ve sağ duba kuvvetleri ortada dengelenir)
                rep_x += - (dx_m / dist) * force_mag
                rep_y += - (dy_m / dist) * force_mag
                        
        # 2. Sarı Engeller (Yellow Obstacles) İtme Hesabı (Asimetrik COLREGs - Sağa Sancak Kaçışı Sağlar)
        # O(N^2) tam ızgara taraması yerine sadece maliyeti min_cost_threshold'dan büyük hücreleri NumPy ile bulup döngüye sokuyoruz (C6 optimizasyonu)
        rows_o, cols_o = np.where(self.grid_obstacles > self.min_cost_threshold)
        for r, c in zip(rows_o, cols_o):
            cost = self.grid_obstacles[r, c]
            dx_m = (self.center_idx - r) * self.resolution
            dy_m = (c - self.center_idx) * self.resolution
            dist = math.sqrt(dx_m**2 + dy_m**2)
            if dist < 0.1: continue
            
            if dist <= influence_distance_obstacles:
                force_mag = K_repulsive * (cost / 100.0) * ((1.0 / dist) - (1.0 / influence_distance_obstacles)) * (1.0 / dist**2)
                # APF İtici Kuvvet Sınırsızlığı Koruması (Görev 61)
                force_mag = min(15.0, force_mag)
                
                ux = - (dx_m / dist)
                uy = - (dy_m / dist)
                
                # Eğer engel önümüzde, ön-solumuz veya ön-sağımızda doğrudan karşı karşıya (head-on / abs(dy_m) <= 1.2m) ise 
                # sağa kaçışı (sancak) tetikleyecek asimetrik itme uyguluyoruz (COLREGs Kural 14 uyumu).
                # Ancak sağ taraf kapalı/engelli ise kıyı şeridi güvenliği için bunu devre dışı bırakıyoruz (Görev 2.3).
                if not right_blocked and dx_m > 0.0 and abs(dy_m) <= 1.2:
                    # ~22 derecelik rotasyon (cos(22) = 0.927, sin(22) = 0.374)
                    cos_t = 0.927
                    sin_t = 0.374
                    ux_rot = ux * cos_t + uy * sin_t
                    uy_rot = -ux * sin_t + uy * cos_t
                    ux, uy = ux_rot, uy_rot
                    
                rep_x += ux * force_mag
                rep_y += uy * force_mag
                        
        return rep_x, rep_y

    def is_empty(self) -> bool:
        """
        Haritada herhangi bir engel olup olmadığını hızlıca kontrol eder (NumPy tabanlı - O(1) tahsisat).
        """
        return not np.any(self.grid > 0)

    def get_serialized_grid(self) -> list:
        rows, cols = np.where(self.grid > 0)
        serialized = []
        for r, c in zip(rows, cols):
            serialized.append([int(r), int(c), int(self.grid[r, c])])
        return serialized

    def reset(self):
        self.grid_gates.fill(0)
        self.grid_obstacles.fill(0)
        self.grid.fill(0)
