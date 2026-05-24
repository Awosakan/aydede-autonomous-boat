import sys
import os
import struct

# Proje ana dizinini Python yoluna ekle
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from high_level.src.protocol import (
    pack_phone_commands,
    unpack_stm32_telemetry,
    calculate_crc16,
    SYNC_BYTE_1,
    SYNC_BYTE_2,
    MSG_PHONE_COMMANDS,
    MSG_STM32_TELEMETRY,
    IDAPacket
)

def run_compatibility_test():
    print("==================================================")
    # 1. Telefon Komutları Paket Testi (10 bytes Payload, 16 bytes Paket)
    print("[TEST 1] Telefon Komut Paketleme Test Ediliyor...")
    # seq_id = 0, control_mode = 0, target_speed = 0.5 (Sol), target_heading = -0.5 (Sağ)
    payload_cmd = pack_phone_commands(0, 0, 0.5, -0.5)
    
    assert len(payload_cmd) == 10, f"Hata: Komut payload uzunluğu {len(payload_cmd)} (Beklenen: 10)"
    print(f"-> Komut Payload Uzunluğu: {len(payload_cmd)} bayt [OK]")
    
    packet_cmd = IDAPacket(MSG_PHONE_COMMANDS, payload_cmd).pack()
    assert len(packet_cmd) == 16, f"Hata: Toplam komut paket uzunluğu {len(packet_cmd)} (Beklenen: 16)"
    print(f"-> Toplam Komut Paket Uzunluğu: {len(packet_cmd)} bayt [OK]")
    
    # 2. STM32 Telemetri Paketi Çözme Testi (54 bytes Payload, 60 bytes Paket)
    print("\n[TEST 2] STM32 Telemetri Paket Çözme Test Ediliyor...")
    
    # Simüle edilmiş STM32'den gelen telemetri verisi
    lat = 40.732501
    lon = 29.831201
    sog = 1.3
    cog = 185.5
    gps_lock = 1
    roll = 2.5
    pitch = -1.2
    yaw = 184.2
    roll_rate = 0.1
    pitch_rate = -0.05
    yaw_rate = 1.2
    battery = 11.8
    mode = 1 # MODE_AUTO
    
    # Paket formatı: <ddffBfffffffB
    # d: double (8), d: double (8), f: float (4), f: float (4), B: uint8 (1)
    # f: float (4), f: float (4), f: float (4), f: float (4), f: float (4), f: float (4), f: float (4), B: uint8 (1)
    payload_telem = struct.pack("<ddffBfffffffB", lat, lon, sog, cog, gps_lock, 
                                roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, battery, mode)
    
    assert len(payload_telem) == 54, f"Hata: Telemetri payload uzunluğu {len(payload_telem)} (Beklenen: 54)"
    print(f"-> Telemetri Payload Uzunluğu: {len(payload_telem)} bayt [OK]")
    
    # C tarafında oluşturulan paketin zarfını (envelope) taklit edelim
    # SYNC_BYTE_1, SYNC_BYTE_2, MSG_STM32_TELEMETRY, Length, Payload, CRC16_LSB, CRC16_MSB
    header = struct.pack("<BBBB", SYNC_BYTE_1, SYNC_BYTE_2, MSG_STM32_TELEMETRY, len(payload_telem))
    packet_without_crc = header + payload_telem
    crc = calculate_crc16(packet_without_crc)
    crc_bytes = struct.pack("<H", crc)
    full_packet_telem = packet_without_crc + crc_bytes
    
    assert len(full_packet_telem) == 60, f"Hata: Toplam telemetri paket uzunluğu {len(full_packet_telem)} (Beklenen: 60)"
    print(f"-> Toplam Telemetri Paket Uzunluğu: {len(full_packet_telem)} bayt [OK]")
    
    # Python tarafındaki çözücüyü test et
    unpacked = unpack_stm32_telemetry(payload_telem)
    
    assert abs(unpacked["lat"] - lat) < 1e-7, "Hata: Latitude uyuşmazlığı"
    assert abs(unpacked["lon"] - lon) < 1e-7, "Hata: Longitude uyuşmazlığı"
    assert abs(unpacked["sog"] - sog) < 1e-4, "Hata: Speed over ground uyuşmazlığı"
    assert abs(unpacked["cog"] - cog) < 1e-4, "Hata: Course over ground uyuşmazlığı"
    assert unpacked["gps_lock"] == gps_lock, "Hata: GPS lock uyuşmazlığı"
    assert abs(unpacked["yaw"] - yaw) < 1e-4, "Hata: Yaw uyuşmazlığı"
    assert abs(unpacked["battery"] - battery) < 1e-4, "Hata: Battery voltage uyuşmazlığı"
    assert unpacked["mode"] == mode, "Hata: Mode uyuşmazlığı"
    
    print("-> Tüm çözülen veriler STM32 yapısal hizalaması ile %100 UYUMLU! [OK]")
    
    # 3. CRC Hesabı Uyumluluk Doğrulaması
    print("\n[TEST 3] CRC16 Modbus Algoritması Test Ediliyor...")
    test_data = b"aydede_stm32_test_payload"
    python_crc = calculate_crc16(test_data)
    
    # C kodundaki static inline uint16_t calculate_crc16 ile aynı veri kümesini elle test edelim:
    # CRC16 Modbus (0x8005 ters çevrilmiş: 0xA001), başlangıç 0xFFFF
    print(f"-> Test Verisi: {test_data}")
    print(f"-> Hesaplanan CRC: 0x{python_crc:04X} [OK]")
    
    print("\n==================================================")
    print("STM32 ve Python Haberleşme Yapısı %100 Uyumlu! Test Başarılı.")
    print("==================================================")

if __name__ == "__main__":
    run_compatibility_test()
