import os
import sys
import json
import time
import shutil
import subprocess

def run_test():
    print("=== DUAL-PATH LOGGING (USB & LOCAL) GÜVENLİK VE ENTEGRASYON TESTİ ===")
    
    # Dizinlerin tanımlanması
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(script_dir)
    config_path = os.path.join(project_dir, "high_level", "src", "config.json")
    
    local_log_dir = os.path.join(project_dir, "high_level", "src", "ida_logs")
    test_usb_dir = os.path.join(script_dir, "test_usb_dir")
    
    # Varsa önceki logları temizleyelim
    if os.path.exists(local_log_dir):
        shutil.rmtree(local_log_dir)
    if os.path.exists(test_usb_dir):
        shutil.rmtree(test_usb_dir)
        
    os.makedirs(test_usb_dir, exist_ok=True)
    
    # 1. config.json yedeğini alalım
    print("config.json yedeği alınıyor...")
    with open(config_path, "r", encoding="utf-8") as f:
        original_config = json.load(f)
        
    # 2. Config dosyasına usb_log_dir yolunu verelim
    test_config = original_config.copy()
    test_config["usb_log_dir"] = test_usb_dir
    
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(test_config, f, indent=4)
    print(f"config.json geçici olarak güncellendi. usb_log_dir = {test_usb_dir}")
    
    # 3. main.py uygulamasını başlatalım
    main_py_path = os.path.join(project_dir, "high_level", "src", "main.py")
    print(f"Otonom sistem (main.py) başlatılıyor: {main_py_path}")
    
    # DISPLAY= ile arayüzün açılmasını engelleyebiliriz veya pencereleri kapatabiliriz
    env = os.environ.copy()
    # Windows'ta arayüzün açılmasını engellemek için DISPLAY değişkenini boşaltalım
    if "DISPLAY" in env:
        del env["DISPLAY"]
        
    process = subprocess.Popen(
        [sys.executable, main_py_path, "MOCK", "115200"],
        cwd=os.path.join(project_dir, "high_level", "src"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env
    )
    
    # 6 saniye boyunca çalışmasını bekleyelim
    print("Sistem çalışıyor, logların yazılması için 6 saniye bekleniyor...")
    time.sleep(6)
    
    # 4. Süreci sonlandıralım
    print("Sistem sonlandırılıyor...")
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        print("Süreç terminate komutuna yanıt vermedi, kill ediliyor...")
        process.kill()
        process.wait()
        
    # 5. Dosyaları doğrulayalım
    expected_files = ["dosya1_kamera.mp4", "dosya2_telemetri.csv", "dosya3_costmap.jsonl"]
    
    local_ok = True
    usb_ok = True
    
    print("\n--- Yerel Log Klasörü Kontrolü (./ida_logs) ---")
    for fname in expected_files:
        path = os.path.join(local_log_dir, fname)
        if os.path.exists(path):
            size = os.path.getsize(path)
            print(f"[OK] {fname} mevcut, Boyut: {size} byte")
            if size == 0:
                print(f"[HATA] {fname} boyutu 0 byte!")
                local_ok = False
        else:
            print(f"[HATA] {fname} bulunamadı!")
            local_ok = False
            
    print("\n--- Harici USB Log Klasörü Kontrolü (test_usb_dir) ---")
    for fname in expected_files:
        path = os.path.join(test_usb_dir, fname)
        if os.path.exists(path):
            size = os.path.getsize(path)
            print(f"[OK] {fname} mevcut, Boyut: {size} byte")
            if size == 0:
                print(f"[HATA] {fname} boyutu 0 byte!")
                usb_ok = False
        else:
            print(f"[HATA] {fname} bulunamadı!")
            usb_ok = False
            
    # 6. config.json dosyasını eski haline getirelim
    print("\nOriginal config.json geri yükleniyor...")
    with open(config_path, "w", encoding="utf-8") as f:
        json.dump(original_config, f, indent=4)
        
    # Geçici dosyaları temizleyelim
    try:
        shutil.rmtree(test_usb_dir)
        print("Geçici USB test klasörü temizlendi.")
    except Exception as e:
        print(f"Geçici USB test klasörü temizlenirken hata oluştu: {e}")
        
    # Test sonucunu verelim
    if local_ok and usb_ok:
        print("\n=== TEST BAŞARIYLA TAMAMLANDI! ÇİFT DİZİNLİ ASENKRON LOGLAMA DOĞRULANDI! ===")
        sys.exit(0)
    else:
        print("\n=== TEST BAŞARISIZ! DOSYALAR OLUŞTURULAMADI VEYA BOYUTLARI 0 BYTE! ===")
        sys.exit(1)

if __name__ == "__main__":
    run_test()
