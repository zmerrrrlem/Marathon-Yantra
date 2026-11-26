# Konfigurasi Unit Robot Altair (Multi-Robot Fleet)

Folder ini menyimpan profil konfigurasi individual untuk setiap unit fisik robot Altair.

## Tujuan
Setiap robot fisik memiliki toleransi mekanik, port USB, dan nilai offset kalibrasi servo yang berbeda. Dengan memisahkan konfigurasi per folder `robot_<id>`, **Divisi Hardware & Kalibrasi** dapat menyesuaikan offset masing-masing unit tanpa risiko konflik git saat di-merge.

## Struktur
- `robot_1/`: Konfigurasi Robot Unit 1 (e.g. Striker / Attacker)
- `robot_2/`: Konfigurasi Robot Unit 2 (e.g. Goalie / Kiper)
- `robot_<N>/`: Tambahkan folder baru jika ada robot unit tambahan.

## Cara Penggunaan
Set environment variable di komputer onboard robot (`~/.bashrc`):
```bash
export ROBOT_ID=1
export ROBOT_ROLE=striker
```
Sistem launch akan secara otomatis memuat file konfigurasi yang sesuai dengan `$ROBOT_ID`.
