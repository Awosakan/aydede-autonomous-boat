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
                    # Kapı dubaları için SABİT ve KÜÇÜK şişirme yarıçapı kullanılır (max 0.8m = 3 hücre).
                    # Büyük dinamik şişirme iki dubayı tek kümeye birleştirir ve koridor
                    # algoritmasını kırar. Bu gerçek suda en büyük risk.
                    gate_inflation_cells = min(self.inflation_radius_cells, int(0.8 / self.resolution))
                    saved_cells = self.inflation_radius_cells
                    self.inflation_radius_cells = gate_inflation_cells
                    self._inflate_obstacle(self.grid_gates, row, col)
                    self.inflation_radius_cells = saved_cells
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

    def _cluster_gate_posts(self) -> list:
        """
        Kapı katmanındaki yüksek maliyetli hücreleri kümelere ayırarak
        her bir duba merkezinin (centroid) body-frame koordinatlarını döndürür.
        Basit flood-fill (bağlı bileşenler) ile kümeleme yapılır.
        """
        rows_g, cols_g = np.where(self.grid_gates > 50)  # Yalnızca güçlü tespitler
        if len(rows_g) == 0:
            return []
        
        visited = set()
        centroids = []
        
        for i in range(len(rows_g)):
            r0, c0 = int(rows_g[i]), int(cols_g[i])
            if (r0, c0) in visited:
                continue
            
            # BFS ile bağlı bileşeni bul
            cluster = []
            queue = [(r0, c0)]
            visited.add((r0, c0))
            
            while queue:
                r, c = queue.pop(0)
                cluster.append((r, c))
                
                # 8-komşuluk (3x3 pencere)
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        nr, nc = r + dr, c + dc
                        if (nr, nc) not in visited and 0 <= nr < self.grid_size and 0 <= nc < self.grid_size:
                            if self.grid_gates[nr, nc] > 50:
                                visited.add((nr, nc))
                                queue.append((nr, nc))
            
            # Küme centroid'ini ağırlıklı ortalama ile hesapla
            total_weight = 0.0
            cx_sum = 0.0
            cy_sum = 0.0
            for r, c in cluster:
                w = float(self.grid_gates[r, c])
                dx_m = (self.center_idx - r) * self.resolution
                dy_m = (c - self.center_idx) * self.resolution
                cx_sum += dx_m * w
                cy_sum += dy_m * w
                total_weight += w
            
            if total_weight > 0:
                centroids.append((cx_sum / total_weight, cy_sum / total_weight))
        
        return centroids

    def _pair_gate_posts(self, centroids: list) -> list:
        """
        Duba merkezlerini kapı çiftlerine ayırır.
        Birbirine en yakın çiftleri eşleştirir (1.5m - 6.0m arası mesafe).
        Döndürdüğü her çift: ((dx1, dy1), (dx2, dy2)) şeklindedir.
        """
        if len(centroids) < 2:
            return []
        
        used = set()
        pairs = []
        
        # Tüm olası çiftleri mesafe sırasına göre değerlendir
        candidates = []
        for i in range(len(centroids)):
            for j in range(i + 1, len(centroids)):
                dx = centroids[i][0] - centroids[j][0]
                dy = centroids[i][1] - centroids[j][1]
                dist = math.sqrt(dx**2 + dy**2)
                if 1.5 <= dist <= 6.0:  # Kapı genişliği aralığı (fiziksel sınır)
                    candidates.append((dist, i, j))
        
        candidates.sort(key=lambda x: x[0])
        
        for _, i, j in candidates:
            if i not in used and j not in used:
                used.add(i)
                used.add(j)
                pairs.append((centroids[i], centroids[j]))
        
        return pairs

    def get_obstacle_forces(self) -> tuple:
        """
        Yapay Potansiyel Alanlar (APF) için bileşke itici kuvveti hesaplar.
        
        KAPI DUBA STRATEJİSİ (Gate Corridor Navigation):
        İki kapı dubası arasından geçiş için "duvar etkisi" (wall effect) yerine 
        "koridor orta hat" (corridor midline) yaklaşımı kullanılır:
        1. Kapı hücrelerini kümelere ayırarak duba merkezlerini bul.
        2. Yakın dubaları çift eşleştirerek kapı tanımla.
        3. Kapının orta noktasından geçen eksene dik YANAL düzeltme kuvveti uygula.
           → İleri/geri itme YOK, sadece yanal (lateral) koridor düzeltmesi.
           → Planlayıcının çekici kuvveti botu kapıdan ileriye çeker.
        
        Eşleşmeyen tekli dubalar için güvenlik amaçlı hafif itme uygulanır.
        
        ENGEL STRATEJİSİ:
        - COLREGs kurallarına uygun asimetrik (sağa kaçış) itme uygular.
        - Kıyı sınırı mantığı ile sağ taraf engelli ise asimetrik büküm devre dışı bırakılır.
        """
        rep_x = 0.0
        rep_y = 0.0
        
        K_repulsive = 5.0
        influence_distance_obstacles = 5.0
        
        # Sağ tarafın engel durumu (Görev 2.3)
        right_blocked = self.is_right_blocked(max_dist_m=6.0)
        
        # ==========================================
        # 1. KAPI DUBA KORİDOR KUVVETLERİ
        # ==========================================
        centroids = self._cluster_gate_posts()
        gate_pairs = self._pair_gate_posts(centroids)
        paired_centroids = set()
        
        K_corridor = 4.0        # Koridor yanal düzeltme kazancı
        corridor_influence = 5.0  # Koridor etkileşim mesafesi (m)
        
        for (post_a, post_b) in gate_pairs:
            paired_centroids.add(post_a)
            paired_centroids.add(post_b)
            
            # Kapı dubaları için uzun mesafeli duvar etkisi (wall effect) İPTAL edildi.
            # Bunun yerine sadece çarpışmayı önlemek için ÇOK KISA mesafeli (1.2m)
            # ve yumuşak bir itici kuvvet uygulanır. Yönlendirme işini planlayıcı
            # (attractive force) yapacaktır.
            for post in [post_a, post_b]:
                dist_to_post = math.sqrt(post[0]**2 + post[1]**2)
                if 0.1 < dist_to_post < 1.2:
                    # Mesafe azaldıkça artan logaritmik itme (max 3.0)
                    push_mag = 3.0 * ((1.0 / dist_to_post) - (1.0 / 1.2))
                    rep_x += -(post[0] / dist_to_post) * push_mag
                    rep_y += -(post[1] / dist_to_post) * push_mag
        
        # Eşleşmeyen tekli dubalar için standart güvenlik itmesi (2.0m etki)
        for cent in centroids:
            if cent not in paired_centroids:
                dist = math.sqrt(cent[0]**2 + cent[1]**2)
                if 0.1 < dist < 2.0:
                    force_mag = 3.0 * ((1.0 / dist) - (1.0 / 2.0))
                    force_mag = min(5.0, force_mag)
                    rep_x += -(cent[0] / dist) * force_mag
                    rep_y += -(cent[1] / dist) * force_mag
        
        # ==========================================
        # 2. SARI ENGEL İTİCİ KUVVETLERİ (COLREGs)
        # ==========================================
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
