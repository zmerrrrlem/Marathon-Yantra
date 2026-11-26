#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

# === Paths ===
MOVEGROUP_FILE = Path("src/altair_misc/altair_moveit_config/launch/move_group.launch.py")
CONTROLLER_FILE = Path("src/a_command_center/config/controller.yaml")
GAZEBO_XACRO_FILE = Path("src/altair_misc/altair_description/gazebo/altair.gazebo.xacro")
EXECUTE_MOTION_LAUNCH_FILE = Path("src/a_command_center/launch/execute_motion.launch.py")


def update_movegroup_file(real_mode, mock_mode, sim_mode, tuning_mode, dxl_unavailable, imu_unavailable, dynamixel_port, dynamixel_baudrate, stm32_port, stm32_baudrate):
    if not MOVEGROUP_FILE.exists():
        print(f"Launch file not found: {MOVEGROUP_FILE}")
        return

    content = MOVEGROUP_FILE.read_text()

    content = re.sub(r'"real_mode": "(true|false)"', f'"real_mode": "{real_mode}"', content)
    content = re.sub(r'"mock_mode": "(true|false)"', f'"mock_mode": "{mock_mode}"', content)
    content = re.sub(r'"sim_mode": "(true|false)"', f'"sim_mode": "{sim_mode}"', content)
    content = re.sub(r'"tuning_mode": "(true|false)"', f'"tuning_mode": "{tuning_mode}"', content)
    content = re.sub(r'"dxl_unavailable": "(true|false)"', f'"dxl_unavailable": "{dxl_unavailable}"', content)
    content = re.sub(r'"imu_unavailable": "(true|false)"', f'"imu_unavailable": "{imu_unavailable}"', content)
    content = re.sub(r'"dynamixel_port": "(.*)"', f'"dynamixel_port": "{dynamixel_port}"', content)
    content = re.sub(r'"dynamixel_baudrate": "(.*)"', f'"dynamixel_baudrate": "{dynamixel_baudrate}"', content)
    content = re.sub(r'"stm32_port": "(.*)"', f'"stm32_port": "{stm32_port}"', content)
    content = re.sub(r'"stm32_baudrate": "(.*)"', f'"stm32_baudrate": "{stm32_baudrate}"', content)

    MOVEGROUP_FILE.write_text(content)
    print(f"Updated modes in {MOVEGROUP_FILE}")


def update_CONTROLLER_FILE(sim_mode, mock_mode):
    if not CONTROLLER_FILE.exists():
        print(f"Quintic Controller YAML config file not found: {CONTROLLER_FILE}")
        return

    content = CONTROLLER_FILE.read_text()
    content = re.sub(r'use_sim_time: (true|false)', f'use_sim_time: {sim_mode}', content)
    
    # When in mock mode, comment out IMU sensor broadcaster sections
    if mock_mode == "true":
        # Handle the IMU sensor broadcaster inside controller_manager
        content = re.sub(
            r'(\s+)imu_sensor_broadcaster:\s*\n(\s+)type: imu_sensor_broadcaster/IMUSensorBroadcaster\s*\n(\s+)sensor_name: imu_sensor\s*\n(\s+)frame_id: imu_link', 
            r'\1# imu_sensor_broadcaster:\n\2# type: imu_sensor_broadcaster/IMUSensorBroadcaster\n\3# sensor_name: imu_sensor\n\4# frame_id: imu_link', 
            content
        )
        
        # Handle the standalone imu_sensor_broadcaster section
        content = re.sub(
            r'imu_sensor_broadcaster:\s*\n\s+ros__parameters:\s*\n\s+type: imu_sensor_broadcaster/IMUSensorBroadcaster\s*\n\s+sensor_name: imu_sensor\s*\n\s+frame_id: imu_link', 
            r'# imu_sensor_broadcaster:\n#   ros__parameters:\n#     type: imu_sensor_broadcaster/IMUSensorBroadcaster\n#     sensor_name: imu_sensor\n#     frame_id: imu_link', 
            content
        )
        print(f"Disabled IMU sensor broadcaster in {CONTROLLER_FILE}")
    else:
        # Ensure IMU sensor broadcaster is uncommented when not in mock mode
        # (This is optional if you want to automatically restore the configuration)
        content = re.sub(
            r'(\s+)# imu_sensor_broadcaster:\s*\n(\s+)# type: imu_sensor_broadcaster/IMUSensorBroadcaster\s*\n(\s+)# sensor_name: imu_sensor\s*\n(\s+)# frame_id: imu_link', 
            r'\1imu_sensor_broadcaster:\n\2type: imu_sensor_broadcaster/IMUSensorBroadcaster\n\3sensor_name: imu_sensor\n\4frame_id: imu_link', 
            content
        )
        
        content = re.sub(
            r'# imu_sensor_broadcaster:\s*\n#\s+ros__parameters:\s*\n#\s+type: imu_sensor_broadcaster/IMUSensorBroadcaster\s*\n#\s+sensor_name: imu_sensor\s*\n#\s+frame_id: imu_link', 
            r'imu_sensor_broadcaster:\n  ros__parameters:\n    type: imu_sensor_broadcaster/IMUSensorBroadcaster\n    sensor_name: imu_sensor\n    frame_id: imu_link', 
            content
        )
    
    CONTROLLER_FILE.write_text(content)
    print(f"Updated use_sim_time in {CONTROLLER_FILE}")

def update_execute_motion_launch_file(robot_name):
    if not EXECUTE_MOTION_LAUNCH_FILE.exists():
        print(f"Execute motion launch file not found: {EXECUTE_MOTION_LAUNCH_FILE}")
        return

    content = EXECUTE_MOTION_LAUNCH_FILE.read_text()
    # Replace robot_name assignment (Python or XML style)
    content, count = re.subn(
        r'(robot_name\s*=\s*["\'])([^"\']*)(["\'])',
        rf'\1{robot_name}\3',
        content
    )
    if count == 0:
        # Try XML style (if using launch XML)
        content, count = re.subn(
            r'(<arg\s+name="robot_name"\s+value=")[^"]*(")',
            rf'\1{robot_name}\2',
            content
        )
    EXECUTE_MOTION_LAUNCH_FILE.write_text(content)
    print(f"Updated robot_name to {robot_name} in {EXECUTE_MOTION_LAUNCH_FILE}")

def update_walk_package(walk_package):
    if not GAZEBO_XACRO_FILE.exists():
        print(f"Gazebo xacro file not found: {GAZEBO_XACRO_FILE}")
        return

    content = GAZEBO_XACRO_FILE.read_text()
    # Update the controller.yaml path to use the specified walk package
    content = re.sub(
        r'\$\(find [^)]+\)/config/gazebo_sim/controller\.yaml',
        f'$(find {walk_package})/config/gazebo_sim/controller.yaml',
        content
    )
    GAZEBO_XACRO_FILE.write_text(content)
    print(f"Updated walk package to {walk_package} in {GAZEBO_XACRO_FILE}")

    if not EXECUTE_MOTION_LAUNCH_FILE.exists():
            print(f"Execute motion launch file not found: {EXECUTE_MOTION_LAUNCH_FILE}")
            return

    content = EXECUTE_MOTION_LAUNCH_FILE.read_text()
    # Replace walk_package assignment (Python or XML style)
    content, count = re.subn(
        r'(walk_package\s*=\s*["\'])([^"\']*)(["\'])',
        rf'\1{walk_package}\3',
        content
    )
    if count == 0:
        # Try XML style (if using launch XML)
        content, count = re.subn(
            r'(<arg\s+name="walk_package"\s+value=")[^"]*(")',
            rf'\1{walk_package}\2',
            content
        )
    EXECUTE_MOTION_LAUNCH_FILE.write_text(content)
    print(f"Updated walk_package to {walk_package} in {EXECUTE_MOTION_LAUNCH_FILE}")


def main():
    parser = argparse.ArgumentParser(description="Update robot mode flags and simulation time.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--real", "-r", action="store_const", const="true", default="false", help="Set real_mode")
    group.add_argument("--mock", "-m", action="store_const", const="true", default="false", help="Set mock_mode")
    group.add_argument("--sim", "-s", action="store_const", const="true", default="false", help="Set sim_mode (also affects use_sim_time)")

    parser.add_argument("--tuning", "-t", default="false", choices=["true", "false"], help="Set tuning_mode")
    parser.add_argument("--dxl-unavailable", "-w", default="false", choices=["true", "false"], help="Set dynamixel_unavailable")
    parser.add_argument("--imu-unavailable", "-i", default="false", choices=["true", "false"], help="Set imu_unavailable")
    parser.add_argument("--dynamixel-port", default="/dev/u2d2", help="Set Dynamixel port")
    parser.add_argument("--dynamixel-baudrate", default="1000000", help="Set Dynamixel baudrate")
    parser.add_argument("--stm32-port", default="/dev/stm32", help="Set STM32 port")
    parser.add_argument("--stm32-baudrate", default="115200", help="Set STM32 baudrate")
    parser.add_argument("--walk-package", "-p", default="quintic_walk", 
                       choices=["quintic_walk", "cubic_walk", "b_splines_walk", "zmp_walk"], 
                       help="Set walk package for Gazebo controller")
    parser.add_argument("--robot-name", default="alta", help="Set robot_name in execute_motion.launch.py")
    

    args = parser.parse_args()

    update_movegroup_file(args.real, args.mock, args.sim, args.tuning, args.dxl_unavailable, args.imu_unavailable, args.dynamixel_port, args.dynamixel_baudrate, args.stm32_port, args.stm32_baudrate)
    update_CONTROLLER_FILE(args.sim, args.mock)
    update_execute_motion_launch_file(args.robot_name)
    update_walk_package(args.walk_package)

if __name__ == "__main__":
    main()
