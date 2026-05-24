#!/bin/bash

echo "===================================================="
echo "          İDA Otomatik Başlatma Kurulumu            "
echo "===================================================="

# 1. Termux:Boot dizinlerini oluştur ve scripti kopyala
echo "[+] Termux:Boot başlangıç dizini oluşturuluyor..."
mkdir -p ~/.termux/boot

SRC_SCRIPT="/data/data/com.termux/files/home/aydede/high_level/src/ida_start.sh"
DEST_SCRIPT="$HOME/.termux/boot/ida_start.sh"

if [ -f "$SRC_SCRIPT" ]; then
    echo "[+] Başlangıç scripti kopyalanıyor..."
    cp "$SRC_SCRIPT" "$DEST_SCRIPT"
    chmod +x "$DEST_SCRIPT"
    echo "[✔] Başlangıç scripti başarıyla kuruldu!"
else
    SRC_SCRIPT_ALT="$HOME/aydede/high_level/src/ida_start.sh"
    if [ -f "$SRC_SCRIPT_ALT" ]; then
        echo "[+] Başlangıç scripti kopyalanıyor..."
        cp "$SRC_SCRIPT_ALT" "$DEST_SCRIPT"
        chmod +x "$DEST_SCRIPT"
        echo "[✔] Başlangıç scripti başarıyla kuruldu!"
    else
        echo "[X] Hata: ida_start.sh dosyası bulunamadı!"
        exit 1
    fi
fi

# 2. USB Güç Bağlandığında Otomatik Açılma (Auto-Boot on Charge) Yapılandırması
echo ""
echo "[+] USB Gücü Bağlandığında Otomatik Açılma Yapılandırması:"
echo "----------------------------------------------------"
echo "Telefon kapalıyken USB kablosu (güç) takıldığı an cihazın otomatik açılması için"
echo "iki farklı yöntem mevcuttur. Root yetkisiyle en kararlı olanı kurmaya çalışacağız."
echo ""

# Root (su) yetkisi kontrolü
if command -v su >/dev/null 2>&1; then
    echo "[+] Root erişimi sorgulanıyor..."
    su -c "
        # Sistem bölümünü yazılabilir olarak yeniden bağlamayı dene
        mount -o rw,remount / 2>/dev/null
        mount -o rw,remount /system 2>/dev/null
        
        # 1. Yöntem: playlpm / lpm / charger dosyalarını reboot ile değiştirme
        for file in /system/bin/playlpm /system/bin/lpm /system/bin/kpoc_charger; do
            if [ -f \"\$file\" ] && [ ! -L \"\$file\" ]; then
                echo '[*] Şarj binary dosyası bulundu: '\$file
                mv \"\$file\" \"\$file.bak\" 2>/dev/null
                echo -e '#!/system/bin/sh\n/system/bin/reboot' > \"\$file\"
                chmod 755 \"\$file\"
                echo '[✔] Otomatik açılış yaması uygulandı: '\$file
            fi
        done
    " 2>/dev/null
    
    echo "[✔] Telefon içi otomatik açılış modifikasyonları denendi."
else
    echo "[!] Root yetkisi Termux'ta bulunamadı (Magisk üzerinden Termux'a SU izni verin)."
fi

echo ""
echo "----------------------------------------------------"
echo "ℹ BİLGİLENDİRME (Kritik Bootloader Ayarı):"
echo "Eğer yukarıdaki otomatik açılma yaması Android sürümünüz nedeniyle çalışmazsa,"
echo "en kesin ve standart yöntem cihazı bilgisayara bağlayıp fastboot modunda şu"
echo "komutu çalıştırmaktır:"
echo ""
echo "   fastboot oem off-mode-charge 0"
echo ""
echo "Bu ayar yapıldığında, telefon kapalıyken şarj kablosu takıldığı an doğrudan boot eder."
echo "===================================================="
