#!/bin/bash
# ==============================================================================
# İDA Chroot Ubuntu Kurulum Scripti (Telefon Tarafında Termux'ta Çalıştırılır)
# ==============================================================================
# Bu script, phone_assets/ altındaki Ubuntu Base tarball'unu kullanarak
# telefonun yerel diskinde (/data/local/ubuntu) bir chroot ortamı hazırlar.

if [ "$EUID" -ne 0 ]; then
  echo "[X] Hata: Lütfen bu scripti root (su) yetkileriyle çalıştırın!"
  exit 1
fi

CHROOT_DIR="/data/local/ubuntu"
TARBALL_PATH="/data/data/com.termux/files/home/aydede/phone_assets/ubuntu-base-22.04-base-arm64.tar.gz"

echo "===================================================="
# chroot dizinini hazırla
echo "[+] Chroot dizini hazırlanıyor: $CHROOT_DIR"
mkdir -p "$CHROOT_DIR"

# Temizlik (varsa eski kurulumu temizle)
if [ -d "$CHROOT_DIR/usr" ]; then
    echo "[!] Eski kurulum tespit edildi. Üzerine yazılıyor..."
fi

# Tarball'ı aç
echo "[+] Ubuntu Base arm64 kök dosya sistemi açılıyor..."
if [ -f "$TARBALL_PATH" ]; then
    tar -zxf "$TARBALL_PATH" -C "$CHROOT_DIR"
    echo "[✔] Tarball başarıyla açıldı."
else
    echo "[X] Hata: $TARBALL_PATH bulunamadı!"
    exit 1
fi

# Ağ/DNS ayarları kopyalanıyor
echo "[+] Ağ/DNS ayarları yapılandırılıyor..."
echo "nameserver 8.8.8.8" > "$CHROOT_DIR/etc/resolv.conf"
echo "nameserver 1.1.1.1" >> "$CHROOT_DIR/etc/resolv.conf"
echo "127.0.0.1 localhost" > "$CHROOT_DIR/etc/hosts"

# Proje dosyalarını chroot içine kopyala
echo "[+] Proje dosyaları chroot içerisine kopyalanıyor (/aydede)..."
rm -rf "$CHROOT_DIR/aydede"
cp -r "/data/data/com.termux/files/home/aydede" "$CHROOT_DIR/aydede"

# chroot_init.sh dosyasını chroot içine kopyala
cp "/data/data/com.termux/files/home/aydede/phone_assets/chroot_init.sh" "$CHROOT_DIR/chroot_init.sh"
chmod +x "$CHROOT_DIR/chroot_init.sh"

# Mount işlemlerini yap
echo "[+] Donanım ve çekirdek sanal dosya sistemleri bağlanıyor..."
sh "/data/data/com.termux/files/home/aydede/phone_assets/chroot_mount.sh" mount

# chroot_init.sh dosyasını chroot içinde çalıştır
echo "[+] Ubuntu içi yapılandırma başlatılıyor (chroot_init.sh)..."
chroot "$CHROOT_DIR" /bin/bash -c "sh /chroot_init.sh"

# Temizlik
rm -f "$CHROOT_DIR/chroot_init.sh"

echo ""
echo "===================================================="
echo "[✔] Chroot Ubuntu Kurulumu Tamamlandı!"
echo "Chroot ortamına girmek için: chroot $CHROOT_DIR /bin/bash"
echo "===================================================="
