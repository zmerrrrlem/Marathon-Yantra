#!/usr/bin/env python3
"""Main Launcher for Altair Humanoid Robot Competitions.

Unified entrypoint to run different competition branches without touching core modules.
"""

import os
import subprocess
import sys

CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"

WS_DIR = os.path.dirname(os.path.abspath(__file__))


def run_cmd(command):
    """Run bash command with sourced ROS 2 and workspace overlay."""
    full_cmd = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {WS_DIR}/install/setup.bash && "
        f"{command}"
    )
    subprocess.run(["bash", "-c", full_cmd])


def print_header():
    os.system("clear")
    robot_id = os.environ.get("ROBOT_ID", "1")
    robot_role = os.environ.get("ROBOT_ROLE", "standalone").upper()
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    print(f"{CYAN}{BOLD}")
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║                 ALTAIR ROBOT COMPETITION LAUNCHER                    ║")
    print("║            Multi-Branch Humanoid Competition Framework               ║")
    print(f"║   Unit: ROBOT-{robot_id:<2} ({robot_role:<10}) | ROS_DOMAIN_ID: {domain_id:<6}             ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")
    print(f"{RESET}")


def get_camera_device():
    default_dev = "/dev/video2" if os.path.exists("/dev/video2") else "/dev/video0"
    print(f"{YELLOW}Pilihan Device Kamera (default: {default_dev}):{RESET}")
    user_input = input(f"Masukkan path device [{default_dev}]: ").strip()
    return user_input if user_input else default_dev


def main():
    while True:
        print_header()
        print(f"{BOLD}PILIH CABANG LOMBA / MODE:{RESET}")
        print(f"  {GREEN}[1]{RESET} ⚽  {BOLD}United Soccer{RESET} (Sepak Bola Tim / GameController)")
        print(f"  {GREEN}[2]{RESET} 🥅  {BOLD}Penalty Kick{RESET}  (Menggiring & Menendang Penalti)")
        print(f"  {GREEN}[3]{RESET} 🏃  {BOLD}Sprint{RESET}        (Lari Cepat / Jalur Lurus)")
        print()
        print(f"{BOLD}TOOLS & DIAGNOSTIK:{RESET}")
        print(f"  {CYAN}[4]{RESET} 📷  {BOLD}Live Vision Tester{RESET} (Buka Kamera Langsung + Deteksi AI)")
        print(f"  {CYAN}[5]{RESET} 📊  {BOLD}RQT Image View{RESET}     (Lihat Feed Anotasi ROS)")
        print(f"  {CYAN}[6]{RESET} 📡  {BOLD}Echo Ball Topic{RESET}    (Cek data /vision/ball_position)")
        print(f"  {CYAN}[7]{RESET} 🦾  {BOLD}Hardware DXL Check{RESET} (Periksa Servo Dynamixel)")
        print(f"  {CYAN}[8]{RESET} ⚙️   {BOLD}Configure Robot{RESET}   (configure.py real/sim/mock)")
        print()
        print(f"  {RED}[q]{RESET} ❌  Keluar")
        print()

        choice = input(f"{BOLD}Masukkan pilihan [1-8, q]: {RESET}").strip().lower()

        if choice == "1":
            cam = get_camera_device()
            print(f"\n{GREEN}Menjalankan mode: United Soccer dengan kamera {cam}...{RESET}")
            run_cmd(f"ros2 launch altair_bringup competition.launch.py mode:=united_soccer video_device:={cam}")

        elif choice == "2":
            cam = get_camera_device()
            print(f"\n{GREEN}Menjalankan mode: Penalty Kick dengan kamera {cam}...{RESET}")
            run_cmd(f"ros2 launch altair_bringup competition.launch.py mode:=penalty_kick video_device:={cam}")

        elif choice == "3":
            cam = get_camera_device()
            print(f"\n{GREEN}Menjalankan mode: Sprint dengan kamera {cam}...{RESET}")
            run_cmd(f"ros2 launch altair_bringup competition.launch.py mode:=sprint video_device:={cam}")

        elif choice == "4":
            cam = get_camera_device()
            print(f"\n{CYAN}Membuka GUI Live Tester pada {cam}...{RESET}")
            run_cmd(f"python3 {WS_DIR}/test_altair_vision.py --device {cam}")

        elif choice == "5":
            print(f"\n{CYAN}Membuka RQT Image View...{RESET}")
            run_cmd("ros2 run rqt_image_view rqt_image_view /vision/annotated_frame")

        elif choice == "6":
            print(f"\n{CYAN}Menampilkan data /vision/ball_position (Ctrl+C untuk stop)...{RESET}")
            run_cmd("ros2 topic echo /vision/ball_position")

        elif choice == "7":
            print(f"\n{CYAN}Menjalankan dxl_check.py...{RESET}")
            run_cmd(f"python3 {WS_DIR}/dxl_check.py")
            input("\nTekan Enter untuk kembali ke menu...")

        elif choice == "8":
            print(f"\n{CYAN}Menjalankan configure.py...{RESET}")
            run_cmd(f"python3 {WS_DIR}/configure.py -h")
            input("\nTekan Enter untuk kembali ke menu...")

        elif choice == "q":
            print(f"\n{YELLOW}Keluar dari Altair Competition Launcher.{RESET}")
            break
        else:
            print(f"\n{RED}Pilihan tidak valid!{RESET}")
            input("Tekan Enter untuk mencoba lagi...")


if __name__ == "__main__":
    main()
