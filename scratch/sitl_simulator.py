import sys
import os
import time
import math
import numpy as np
import cv2
import logging

# Logger Ayarı
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SITL")

# Modülleri içe aktarmak için ana dizini ekle
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from high_level.src.costmap import LocalCostmap
from high_level.src.mission_control import MissionController, STATE_PARKUR1
from high_level.src.protocol import pack_stm32_telemetry, unpack_phone_commands
from high_level.src.planner import gps_to_meters

class IDASimulator:
    """
    Yazılım-Döngüde (SITL) İDA Fizik ve Çevre Simülatörü.
    Katamaran itki fiziğini, su sürtünmesini, akıntı/rüzgar sürüklenmesini ve kamera görüşünü modeller.
    """
    def __init__(self):
        # 1. Simülasyon Dünya Koordinat Tanımları (Metre)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0      # Derece (0: Kuzey, 90: Doğu, vb.)
        self.speed = 0.0    # m/s
        self.yaw_rate = 0.0 # deg/s
        
        # Katamaran Boyutları
        self.width = 1.2    # m
        self.mass = 25.0    # kg
        self.inertia = 5.0  # kg*m^2
        
        # Su ve Rüzgar Akıntı Etkileri (F-35 Hassasiyetinde Çevre Modellemesi)
        self.drift_x = 0.15 # Doğu yönünde 0.15 m/s sabit akıntı
        self.drift_y = -0.05 # Güney yönünde 0.05 m/s akıntı
        
        # Motor Değerleri (Thrusts)
        self.left_motor = 0.0
        self.right_motor = 0.0
        
        # 2. Sanal Çevre: Dubalar ve Koordinatları (Metre cinsinden dünya koordinatları)
        self.buoys = [
            # Parkur 1 Kapıları (Çift Turuncu Dubalar - Sol/Sağ Kapı Sınırları)
            {"class": "orange_gate", "x": -1.5, "y": 8.0},  # Kapı 1 Sol
            {"class": "orange_gate", "x": 1.5, "y": 8.0},   # Kapı 1 Sağ
            
            {"class": "orange_gate", "x": 3.5, "y": 15.0},  # Kapı 2 Sol
            {"class": "orange_gate", "x": 6.5, "y": 15.0},  # Kapı 2 Sağ
            
            {"class": "orange_gate", "x": 8.5, "y": 10.0},  # Kapı 3 Sol
            {"class": "orange_gate", "x": 11.5, "y": 10.0}, # Kapı 3 Sağ
            
            {"class": "orange_gate", "x": 13.5, "y": 5.0},  # Kapı 4 Sol
            {"class": "orange_gate", "x": 16.5, "y": 5.0},  # Kapı 4 Sağ
            
            # Parkur 2 Engelleri (Sarı Dubalar)
            {"class": "yellow_obstacle", "x": 20.0, "y": 12.0},
            {"class": "yellow_obstacle", "x": 24.0, "y": 20.0},
            {"class": "yellow_obstacle", "x": 28.0, "y": 15.0},
            
            # Parkur 3 Hedefleri (Kamikaze Dubaları)
            {"class": "target_red", "x": 38.0, "y": 28.0},
            {"class": "target_green", "x": 42.0, "y": 28.0},
            {"class": "target_blue", "x": 46.0, "y": 28.0}
        ]
        
        # Rota Waypoint Tanımları (Simüle edilen GPS koordinatlarına dönüştürülecek)
        self.REF_LAT = 40.732501
        self.REF_LON = 29.831201
        
        self.p1_gps = [self._meters_to_gps(wp[0], wp[1]) for wp in [(0, 8), (5, 15), (10, 10), (15, 5)]]
        self.p2_gps = [self._meters_to_gps(wp[0], wp[1]) for wp in [(20, 15), (32, 22)]]
        self.home_gps = [self.REF_LAT, self.REF_LON]

    def _meters_to_gps(self, dx: float, dy: float) -> list:
        R = 6378137.0
        lat = self.REF_LAT + math.degrees(dy / R)
        lon = self.REF_LON + math.degrees(dx / (R * math.cos(math.radians(self.REF_LAT))))
        return [lat, lon]

    def get_simulated_gps_imu(self) -> dict:
        lat, lon = self._meters_to_gps(self.x, self.y)
        # Sensör gürültüsü ekle
        lat += np.random.normal(0, 0.000002)
        lon += np.random.normal(0, 0.000002)
        yaw_noisy = (self.yaw + np.random.normal(0, 0.5)) % 360.0
        
        return {
            "lat": lat,
            "lon": lon,
            "sog": self.speed + np.random.normal(0, 0.05),
            "cog": self.yaw,
            "gps_lock": 1,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": yaw_noisy,
            "roll_rate": 0.0,
            "pitch_rate": 0.0,
            "yaw_rate": self.yaw_rate,
            "battery": 11.9,
            "mode": 1
        }

    def get_simulated_camera_detections(self) -> list:
        detections = []
        max_view_dist = 16.0  
        camera_fov_deg = 80.0 
        
        for buoy in self.buoys:
            dx = buoy["x"] - self.x
            dy = buoy["y"] - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist > max_view_dist:
                continue
                
            buoy_angle_deg = math.degrees(math.atan2(dx, dy)) % 360.0
            
            bearing_deg = buoy_angle_deg - self.yaw
            while bearing_deg > 180.0:  bearing_deg -= 360.0
            while bearing_deg < -180.0: bearing_deg += 360.0
            
            if abs(bearing_deg) <= (camera_fov_deg / 2.0):
                dist_noisy = dist + np.random.normal(0, 0.1)
                bearing_rad_noisy = math.radians(bearing_deg + np.random.normal(0, 0.8))
                
                detections.append({
                    "class": buoy["class"],
                    "distance": dist_noisy,
                    "bearing": bearing_rad_noisy,
                    "bbox": [100, 100, 30, 45]
                })
                
        return detections

    def step(self, dt: float):
        F_left = self.left_motor * 15.0
        F_right = self.right_motor * 15.0
        
        total_thrust = F_left + F_right
        torque = (F_left - F_right) * (self.width / 2.0)
        
        damping_linear = 8.0 
        damping_angular = 6.0 
        
        accel = (total_thrust - damping_linear * self.speed) / self.mass
        alpha = (torque - damping_angular * self.yaw_rate * (math.pi / 180.0)) / self.inertia
        
        self.speed += accel * dt
        self.yaw_rate += math.degrees(alpha) * dt
        
        self.yaw = (self.yaw + self.yaw_rate * dt) % 360.0
        
        yaw_rad = math.radians(self.yaw)
        vx = self.speed * math.sin(yaw_rad) + self.drift_x
        vy = self.speed * math.cos(yaw_rad) + self.drift_y
        
        self.x += vx * dt
        self.y += vy * dt

# --- Dahili PID Kontrol Modülü (C ile aynı mantıkta çalışır) ---
class SimplePID:
    def __init__(self):
        self.kp = 0.8
        self.ki = 0.05
        self.kd = 0.2
        self.integrator = 0.0
        self.last_error = 0.0
        self.max_integrator = 0.3
        self.last_yaw = -999.0

    def update(self, current_yaw: float, target_yaw: float, target_speed: float, dt: float) -> tuple:
        error = target_yaw - current_yaw
        while error > 180.0:  error -= 360.0
        while error < -180.0: error += 360.0
        
        p_term = self.kp * error
        
        self.integrator += error * dt
        self.integrator = max(-self.max_integrator, min(self.integrator, self.max_integrator))
        i_term = self.ki * self.integrator
        
        if self.last_yaw == -999.0:
            self.last_yaw = current_yaw
            
        yaw_diff = current_yaw - self.last_yaw
        while yaw_diff > 180.0:  yaw_diff -= 360.0
        while yaw_diff < -180.0: yaw_diff += 360.0
        
        if dt > 0.0:
            derivative = -yaw_diff / dt
        else:
            derivative = 0.0
            
        self.last_yaw = current_yaw
        self.last_error = error
        
        # Low-pass filter the derivative term to prevent setpoint kick and noise amplification
        if not hasattr(self, "filtered_deriv"):
            self.filtered_deriv = 0.0
        self.filtered_deriv = 0.8 * self.filtered_deriv + 0.2 * derivative
        d_term = self.kd * self.filtered_deriv
        
        steer_cmd = p_term + i_term + d_term
        steer_cmd = max(-1.0, min(steer_cmd, 1.0))
        
        max_speed_allowed = 1.0 - abs(steer_cmd)
        min_speed_allowed = -1.0 + abs(steer_cmd)
        
        adjusted_speed = target_speed
        if adjusted_speed > max_speed_allowed:
            adjusted_speed = max_speed_allowed
        elif adjusted_speed < min_speed_allowed:
            adjusted_speed = min_speed_allowed
            
        left_thrust = adjusted_speed + steer_cmd
        right_thrust = adjusted_speed - steer_cmd
        
        if abs(target_speed) < 0.05 and abs(error) < 5.0:
            left_thrust = 0.0
            right_thrust = 0.0
            
        return max(-1.0, min(left_thrust, 1.0)), max(-1.0, min(right_thrust, 1.0))

class DummySerial:
    def __init__(self, sim):
        self.sim = sim
        self.pid = SimplePID()
        
    def send_packet(self, payload: bytes, msg_id: int = 0x02):
        if msg_id == 0x02: # MSG_PHONE_COMMANDS
            seq_id, control_mode, target_speed, target_heading = unpack_phone_commands(payload)
            # STM32 iç PID kontrolcüsünü koşturarak motor itkilerini elde et
            left, right = self.pid.update(self.sim.yaw, target_heading, target_speed, 0.02)
            self.sim.left_motor = left
            self.sim.right_motor = right

class DummyLogger:
    def log_telemetry(self, *args): pass
    def log_costmap(self, *args): pass
    def log_frame(self, *args): pass

def run_simulation():
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    logger_mock = DummyLogger()
    
    mission = MissionController(logger_mock, serial_mock)
    mission.set_waypoints(sim.p1_gps, sim.p2_gps, sim.home_gps)
    mission.target_color = "target_red" 
    
    costmap = LocalCostmap(size_m=40.0, resolution=0.25, inflation_radius_m=1.0)
    
    win_size = 800
    scale = 10.0 
    center_offset = win_size // 2
    
    mission.transition_to(STATE_PARKUR1)
    
    dt = 0.04 
    logger.info("SITL Simülatörü Başlatılıyor. Çıkış için pencere üzerindeyken 'q' tuşuna basın.")
    
    while True:
        sim.step(dt)
        
        telemetry = sim.get_simulated_gps_imu()
        mission.update_telemetry(telemetry)
        
        detections = sim.get_simulated_camera_detections()
        mission.process_step(detections, costmap)
        
        canvas = np.zeros((win_size, win_size, 3), dtype=np.uint8)
        
        for i in range(0, win_size, 50):
            cv2.line(canvas, (i, 0), (i, win_size), (30, 30, 30), 1)
            cv2.line(canvas, (0, i), (win_size, i), (30, 30, 30), 1)
            
        def to_screen(wx, wy):
            screen_x = int(center_offset + wx * scale)
            screen_y = int(center_offset - wy * scale)
            return screen_x, screen_y
            
        for buoy in sim.buoys:
            sx, sy = to_screen(buoy["x"], buoy["y"])
            if buoy["class"] == "orange_gate":
                color = (0, 165, 255)
                radius = 6
            elif buoy["class"] == "yellow_obstacle":
                color = (0, 255, 255)
                radius = 6
            elif buoy["class"] == "target_red":
                color = (0, 0, 255)
                radius = 10
            elif buoy["class"] == "target_green":
                color = (0, 255, 0)
                radius = 10
            elif buoy["class"] == "target_blue":
                color = (255, 0, 0)
                radius = 10
            else:
                color = (255, 255, 255)
                radius = 5
                
            cv2.circle(canvas, (sx, sy), radius, color, -1)
            cv2.putText(canvas, buoy["class"].split("_")[-1], (sx-15, sy-12), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        # Rota Waypointlerini Çiz
        all_wps = sim.p1_gps + sim.p2_gps + [sim.home_gps]
        for idx, gps_wp in enumerate(all_wps):
            dx, dy = gps_to_meters(sim.REF_LAT, sim.REF_LON, gps_wp[0], gps_wp[1])
            sx, sy = to_screen(dx, dy)
            cv2.circle(canvas, (sx, sy), 4, (180, 180, 180), -1)
            cv2.putText(canvas, f"WP{idx}", (sx+8, sy+5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        bx, by = to_screen(sim.x, sim.y)
        yaw_rad = math.radians(sim.yaw)
        
        boat_len = 1.6 * scale
        boat_width = 1.0 * scale
        
        p1 = (int(bx + boat_len * math.sin(yaw_rad)), int(by - boat_len * math.cos(yaw_rad)))
        p2 = (int(bx - boat_width * math.cos(yaw_rad)), int(by - boat_width * math.sin(yaw_rad)))
        p3 = (int(bx + boat_width * math.cos(yaw_rad)), int(by + boat_width * math.sin(yaw_rad)))
        
        cv2.drawContours(canvas, [np.array([p1, p2, p3])], 0, (0, 255, 0), -1)
        
        cv2.arrowedLine(canvas, (40, 750), (int(40 + sim.drift_x * 200), int(750 - sim.drift_y * 200)), 
                        (0, 150, 255), 2, tipLength=0.3)
        cv2.putText(canvas, "Akinti/Ruzgar", (20, 725), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 255), 1)

        info = [
            f"State: {mission.state}",
            f"X: {sim.x:.2f} m, Y: {sim.y:.2f} m",
            f"Yaw: {sim.yaw:.1f} deg",
            f"Speed: {sim.speed:.2f} m/s",
            f"Motor L: {sim.left_motor:.2f}, R: {sim.right_motor:.2f}",
            f"Active WP Index: {mission.current_wp_idx}"
        ]
        for offset, text in enumerate(info):
            cv2.putText(canvas, text, (20, 30 + offset * 22), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("IDA SITL Simulator", canvas)
        if cv2.waitKey(int(dt * 1000)) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_simulation()
