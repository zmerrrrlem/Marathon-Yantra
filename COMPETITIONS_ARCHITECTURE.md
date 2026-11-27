# Panduan Arsitektur Multi-Lomba Robot Yantra (KRSBI Humanoid)

Dokumen ini menjelaskan struktur modular workspace `altair_ws_fix` yang dirancang agar dapat digunakan untuk berbagai cabang lomba tanpa saling tumpang tindih.

---

## 1. Filosofi Tiga Lapisan (Three-Layer Architecture)

Struktur workspace Yantra memisahkan antara **Modul Dasar (Core Modules)**, **Logika Lomba (Competitions / Behaviors)**, dan **Peluncur Utama (Main Launcher)**:

```text
altair_ws_fix/
│
├── 📁 core_modules (Modul Dasar Robot)
│   ├── yantra_hardware/         # Interface ke motor servo Dynamixel, STM32, dan sensor IMU
│   ├── yantra_motion/           # Quintic walk, ZMP walk, Inverse Kinematics, Bezier Kick, Head
│   ├── yantra_vision_deeplearning/ # Pipeline kamera USB & model AI OpenVINO (FL3 INT8)
│   ├── yantra_msgs/             # Definisi pesan ROS 2 kustom (RobotControlState, JointCommand, dll.)
│   └── yantra_misc/             # URDF robot description, MoveIt 2 config, bio_ik
│
├── 📁 competitions (Konfigurasi & Logika Cabang Lomba)
│   ├── yantra_bringup/config/competitions/
│   │   ├── penalty_kick.yaml    # Parameter khusus Penalty Kick (threshold bola & gawang)
│   │   ├── united_soccer.yaml   # Parameter khusus United Soccer (bola, kawan, gawang, GameController)
│   │   └── sprint.yaml          # Parameter khusus Sprint (langkah cepat garis lurus)
│   └── a_command_center/behavior_trees/
│       ├── simple_kick.xml      # Behavior Tree untuk Penalty Kick
│       └── barelang_2023.xml    # Behavior Tree untuk United Soccer match
│
└── 🚀 main_launcher (Peluncur Terpadu Satu Pintu)
    ├── ./start_lomba.sh         # Menu interaktif CLI saat robot dinyalakan
    ├── start_lomba.py           # Dashboard pemilihan mode cabang lomba
    └── ros2 launch yantra_bringup competition.launch.py mode:=<nama_lomba>
```

---

## 2. Cara Menjalankan Saat Lomba

Saat robot berada di lapangan lomba atau saat latihan, operator **tidak perlu menyentuh kode modul dasar**:

### Opsi A: Menggunakan Menu Interaktif (Sangat Direkomendasikan di Lapangan)
Cukup buka terminal dan jalankan:
```bash
cd ~/altair_ws_fix
./start_lomba.sh
```
Pilih angka sesuai kebutuhan:
- `[1]` : ⚽ **United Soccer**
- `[2]` : 🥅 **Penalty Kick**
- `[3]` : 🏃 **Sprint**
- `[4]` : 📷 **Live Vision Tester** (Cek kamera & deteksi sebelum tanding)

### Opsi B: Menggunakan Perintah ROS 2 Launch Langsung
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Untuk Penalty Kick:
ros2 launch yantra_bringup competition.launch.py mode:=penalty_kick video_device:=/dev/video2

# Untuk United Soccer:
ros2 launch yantra_bringup competition.launch.py mode:=united_soccer video_device:=/dev/video2

# Untuk menyertakan motor & motion sekalian:
ros2 launch yantra_bringup competition.launch.py mode:=penalty_kick with_motion:=true
```

---

## 3. Alur Kerja Git (Git Workflow)

Sesuai best practice:
1. **Gunakan Branch Berdasarkan Fitur, Bukan Berdasarkan Nama Lomba**:
   - `feature/faster-sprint-gait`
   - `feature/improved-ball-detection`
   - `feature/penalty-aim-logic`
2. **Keuntungan:** Ketika algoritme pendeteksi bola atau jalan diperbarui dan di-merge ke `main`, peningkatan tersebut **langsung dapat dinikmati oleh semua cabang lomba** (baik Penalty Kick maupun United Soccer).
