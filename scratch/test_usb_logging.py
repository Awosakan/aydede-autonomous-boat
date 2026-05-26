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
    print("Sistem çalışıyor, logların yazılması için 10 saniye bekleniyor...")
    time.sleep(10)
    
    # 4. Süreci sonlandıralım
    print("Sistem sonlandırılıyor...")
    try:
        stdout, stderr = process.communicate(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        stdout, stderr = process.communicate()
        
    print("\n--- main.py STDOUT ---")
    print(stdout.decode('utf-8', errors='ignore'))
    print("--- main.py STDERR ---")
    print(stderr.decode('utf-8', errors='ignore'))
        
    # 5. Dosyaları doğrulayalım (Zaman damgalı isimler olabileceği için ön ek kontrolü yapıyoruz)
    expected_prefixes = {
        "dosya1_kamera": ".mp4",
        "dosya2_telemetri": ".csv",
        "dosya3_costmap": ".jsonl"
    }
    
    local_ok = True
    usb_ok = True
    
    print("\n--- Yerel Log Klasörü Kontrolü (./ida_logs) ---")
    for prefix, ext in expected_prefixes.items():
        found = False
        if os.path.exists(local_log_dir):
            for file in os.listdir(local_log_dir):
                if file.startswith(prefix) and file.endswith(ext):
                    path = os.path.join(local_log_dir, file)
                    size = os.path.getsize(path)
                    print(f"[OK] {file} mevcut, Boyut: {size} byte")
                    if size > 0:
                        found = True
                        break
        if found:
            print(f"[OK] {prefix}* yerel dosyası doğrulandı.")
        else:
            print(f"[HATA] {prefix}* yerel dosyası bulunamadı veya boş!")
            local_ok = False
            
    print("\n--- Harici USB Log Klasörü Kontrolü (test_usb_dir) ---")
    for prefix, ext in expected_prefixes.items():
        found = False
        if os.path.exists(test_usb_dir):
            for file in os.listdir(test_usb_dir):
                if file.startswith(prefix) and file.endswith(ext):
                    path = os.path.join(test_usb_dir, file)
                    size = os.path.getsize(path)
                    print(f"[OK] {file} mevcut, Boyut: {size} byte")
                    if size > 0:
                        found = True
                        break
        if found:
            print(f"[OK] {prefix}* USB dosyası doğrulandı.")
        else:
            print(f"[HATA] {prefix}* USB dosyası bulunamadı veya boş!")
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
