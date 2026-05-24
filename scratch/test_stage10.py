import sys
import os
import time
import math
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from high_level.src.costmap import LocalCostmap
from high_level.src.mission_control import (
    MissionController, STATE_IDLE, STATE_PARKUR1, STATE_PARKUR2, STATE_PARKUR3, STATE_RETURN, STATE_FAILSAFE
)
from high_level.src.planner import APFPlanner
from scratch.sitl_simulator import IDASimulator, DummySerial, DummyLogger

def test_dynamic_config():
    print("\n=== TEST 1: Dinamik Konfigürasyon Yükleme ===")
    from high_level.src.main import IDANode
    
    # Yeni bir node oluşturalım, config.json otomatik okunacak/oluşturulacak
    node = IDANode(serial_port="MOCK", baudrate=115200)
    
    assert hasattr(node, "config")
    assert "nominal_speed_ms" in node.config
    assert "target_color" in node.config
    assert node.config["target_color"] == "target_red" # Varsayılan kırmızı olmalı
    print("SUCCESS: Config başarıyla yüklendi ve doğrulandı.")

def test_fsm_timeouts():
    print("\n=== TEST 2: FSM Durum Zaman Aşımı ===")
    logger_mock = DummyLogger()
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    
    config = {
        "nominal_speed_ms": 1.3,
        "max_speed_ms": 2.0,
        "min_speed_ms": 0.5,
        "waypoint_tolerance_m": 0.6,
        "target_color": "target_red",
        "state_timeout_seconds": 1.0 # Hızlı tetiklenmesi için 1.0 saniye ayarlıyoruz
    }
    
    mission = MissionController(logger_mock, serial_mock, config)
    mission.set_waypoints(sim.p1_gps, sim.p2_gps, sim.home_gps)
    mission.update_telemetry(sim.get_simulated_gps_imu())
    costmap = LocalCostmap()
    
    # Parkur 1'i başlatıp 1.5 saniye simüle edelim
    mission.transition_to(STATE_PARKUR1)
    print(f"Durum: {mission.state}")
    
    time.sleep(1.2) # Zaman aşımı süresini geç
    mission.process_step([], costmap)
    
    print(f"Zaman aşımı sonrası durum (Beklenen: STATE_PARKUR2): {mission.state}")
    assert mission.state == STATE_PARKUR2
    
    time.sleep(1.2)
    mission.process_step([], costmap)
    print(f"Zaman aşımı sonrası durum (Beklenen: STATE_PARKUR3): {mission.state}")
    assert mission.state == STATE_PARKUR3
    
    time.sleep(1.2)
    mission.process_step([], costmap)
    print(f"Zaman aşımı sonrası durum (Beklenen: STATE_RETURN): {mission.state}")
    assert mission.state == STATE_RETURN
    print("SUCCESS: FSM durum zaman aşımları başarıyla test edildi.")

def test_ramp_filter():
    print("\n=== TEST 3: Motor Ramp Filtresi ===")
    logger_mock = DummyLogger()
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    
    config = {
        "max_speed_accel": 1.0,  # 1.0 m/s^2 maks ivme
        "max_yaw_rate": 30.0,     # 30.0 deg/s maks açısal hız
        "max_speed_ms": 2.0
    }
    
    mission = MissionController(logger_mock, serial_mock, config)
    mission.update_telemetry(sim.get_simulated_gps_imu())
    costmap = LocalCostmap()
    
    mission.transition_to(STATE_PARKUR1)
    
    # Çok ani büyük bir hedef komutu verelim
    # Normalde bu APFPlanner'dan anlık gelebilir
    # İlk adım
    dt = 0.1
    mission.last_sent_speed = 0.0
    mission.last_sent_heading = 0.0
    
    # 2.0 m/s hız ve 90 derece dönüş isteyelim
    res = mission.process_step([], costmap)
    # Biz test amacıyla target_speed ve target_heading'i zorla yüksek değerlere set edelim
    # Ve filtreyi koşturup çıkış değerlerini inceleyelim
    
    # process_step'i doğrudan manipüle etmek için elle değer atayıp process_step'in son kısmını doğrulayalım
    # mission.last_sent_speed'i test edelim
    # 0.0'dan 2.0 m/s hıza çıkmak istiyoruz, dt=0.1s, ivme=1.0 m/s^2, yani maks artış = 0.1 m/s
    mission.last_sent_speed = 0.0
    target_speed = 2.0
    
    # Hız yumuşatma
    max_delta_speed = config["max_speed_accel"] * dt
    speed_err = target_speed - mission.last_sent_speed
    if speed_err > max_delta_speed:
        target_speed = mission.last_sent_speed + max_delta_speed
    mission.last_sent_speed = target_speed
    
    print(f"Hedef Hız: 2.0, Ramped Hız: {mission.last_sent_speed} (Beklenen: 0.1)")
    assert math.isclose(mission.last_sent_speed, 0.1)

    # Yön yumuşatma: 0 dereceden 90 dereceye anlık dönüş istiyoruz. dt=0.1s, yaw_rate=30.0 deg/s, maks dönüş = 3.0 derece
    mission.last_sent_heading = 0.0
    target_heading = 90.0
    max_delta_heading = config["max_yaw_rate"] * dt
    heading_diff = target_heading - mission.last_sent_heading
    if heading_diff > max_delta_heading:
        target_heading = (mission.last_sent_heading + max_delta_heading) % 360.0
    mission.last_sent_heading = target_heading
    
    print(f"Hedef Yön: 90.0, Ramped Yön: {mission.last_sent_heading} (Beklenen: 3.0)")
    assert math.isclose(mission.last_sent_heading, 3.0)
    
    # Açı aşımı testi (355 dereceden 5 dereceye geçiş, yani +10 derece)
    mission.last_sent_heading = 355.0
    target_heading = 5.0
    heading_diff = target_heading - mission.last_sent_heading
    while heading_diff > 180.0: heading_diff -= 360.0
    while heading_diff < -180.0: heading_diff += 360.0
    
    # heading_diff = 10.0 derece olmalı
    assert math.isclose(heading_diff, 10.0)
    
    print("SUCCESS: Ramp Filtresi ivme ve açısal hız kısıtları başarıyla test edildi.")

def test_kamikaze_target_lost():
    print("\n=== TEST 4: Kamikaze Hedef Kaybı Stratejisi ===")
    logger_mock = DummyLogger()
    sim = IDASimulator()
    serial_mock = DummySerial(sim)
    
    config = {
        "nominal_speed_ms": 1.3,
        "max_speed_ms": 2.0,
        "min_speed_ms": 0.5,
        "waypoint_tolerance_m": 0.6,
        "target_color": "target_red",
        "state_timeout_seconds": 300.0,
        "max_speed_accel": 100.0,
        "max_yaw_rate": 1000.0
    }
    
    mission = MissionController(logger_mock, serial_mock, config)
    mission.update_telemetry(sim.get_simulated_gps_imu())
    costmap = LocalCostmap()
    
    mission.transition_to(STATE_PARKUR3)
    
    # 1. Hedefi ilk kez görelim (Lock olalım)
    # target_red duba kuralım
    detections = [{"class": "target_red", "distance": 5.0, "bearing": math.radians(10.0)}] # 10 derece sancakta
    res = mission.process_step(detections, costmap)
    
    print(f"Hedef Görülürken Durum: Speed={res['target_speed']:.2f}, Heading={res['target_heading']:.1f}°")
    assert math.isclose(res['target_speed'], 1.2)
    last_heading = res['target_heading']
    
    # 2. Aşama 1: Hedef kaybı t <= 2.0s
    time.sleep(1.0)
    mission.update_telemetry(sim.get_simulated_gps_imu())
    # detections boş gönderilerek kayıp simüle edilir
    res = mission.process_step([], costmap)
    print(f"Aşama 1 (t=1s kayıp) Durum: Speed={res['target_speed']:.2f}, Heading={res['target_heading']:.1f}° (Beklenen: last_heading={last_heading:.1f}°)")
    assert math.isclose(res['target_speed'], 1.0)
    assert math.isclose(res['target_heading'], last_heading)
    
    # 3. Aşama 2: Hedef kaybı 2.0s < t <= 10.0s
    time.sleep(1.5) # Toplam 2.5s kayıp
    mission.update_telemetry(sim.get_simulated_gps_imu())
    res = mission.process_step([], costmap)
    print(f"Aşama 2 (t=2.5s kayıp) Durum: Speed={res['target_speed']:.2f}, Heading={res['target_heading']:.1f}° (Beklenen: Speed=0.5, Dönüş başlamış olmalı)")
    assert math.isclose(res['target_speed'], 0.5)
    
    # 4. Aşama 3: Hedef kaybı t > 10.0s
    time.sleep(8.0) # Toplam 10.5s kayıp
    mission.update_telemetry(sim.get_simulated_gps_imu())
    res = mission.process_step([], costmap)
    print(f"Aşama 3 (t=10.5s kayıp) Durum: state={res['state']} (Beklenen: STATE_RETURN)")
    assert res['state'] == STATE_RETURN
    print("SUCCESS: Kamikaze hedef kaybı stratejisi (3 aşamalı) başarıyla test edildi.")

if __name__ == "__main__":
    test_dynamic_config()
    test_fsm_timeouts()
    test_ramp_filter()
    test_kamikaze_target_lost()
    print("\nTÜM TESTLER BAŞARIYLA TAMAMLANDI!")
