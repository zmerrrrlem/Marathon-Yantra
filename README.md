# altair_lab

## Description
The next-generation altair_ws.

## Prerequisites
- Install Jazzy Jalisco:  
  [ROS2 Jazzy Jalisco Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Dynamixel SDK:

  ```bash
  sudo apt install ros-jazzy-dynamixel-sdkgit config --global user.name "Your Name"
  ```

- Docker on Raspberry Pi 5

## Cloning the Repository

### Clone the Repository

```bash
git clone https://github.com/Altair-Humanoid-GMRT/altair_ws.git
```

Then pull the submodules with:

```bash
git submodule update --init --recursive --remote
```

### Pulling Commits from the Repository

```bash
git pull --rebase
```

The `--rebase` flag avoids creating merge commits, which result in non-linear history. Always use it for straight pulls. If you encounter conflicts (indicated by a big `CONFLICT` message), edit the mentioned files, run `git add` on the files, and then use `git rebase --continue` to proceed.

## Installation
### Installation on Linux PC

1. Install the dependencies for ROS2:

    ```bash
    sudo apt install ros-jazzy-xacro ros-jazzy-rqt ros-jazzy-rviz2 ros-jazzy-yaml-cpp-vendor ros-jazzy-vision-msgs ros-jazzy-nav-msgs ros-jazzy-orocos-kdl-vendor ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-joint-limits ros-jazzy-interactive-markers ros-jazzy-image-geometry ros-jazzy-imu-sensor-broadcaster ros-jazzy-kinematics-interface ros-jazzy-libyaml-vendor libserial-dev libserialport-dev libserial1 ros-jazzy-examples-rclcpp-minimal-subscriber ros-jazzy-examples-rclcpp-minimal-publisher ros-jazzy-yaml-cpp-vendor
    ```

2. Install the dependencies for ROS2 Control:

    ```bash
    sudo apt install ros-jazzy-ros2-control ros-jazzy-hardware-interface ros-jazzy-forward-command-controller ros-jazzy-filters ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs ros-jazzy-control-toolbox ros-jazzy-controller-interface ros-jazzy-ros2controlcli ros-jazzy-ros2component ros-jazzy-pid-controller  ros-jazzy-joint-state-broadcaster ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim | sudo apt update
    ```

3. Install ROS2 Jazzy Additional Packages (Simulation & Bridge)
    ```bash
    sudo apt install -y 
    ros-jazzy-ros-gz-bridge 
    ros-jazzy-rosbridge-server 
    ros-jazzy-rosidl-default-generators 
    ros-jazzy-rosidl-default-runtime
    ```

4. Install the dependencies for MoveIt2:

    ```bash
    sudo apt install ros-jazzy-moveit ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds ros-jazzy-moveit-ros-control-interface ros-jazzy-ompl
    ```

5. Change the ROS_DOMAIN_ID and RMW to CycloneDDS:

    ```bash
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc &&
    echo "export ROS_DOMAIN_ID=96" >> ~/.bashrc
    ```

6. Configure DDS and Network, to handle large messages:
    ```bash
    sudo cp cyclone_dds.xml $HOME/.ros/
    echo 'export CYCLONEDDS_URI=file://'"$HOME/.ros/cyclone_dds.xml" >> "$HOME/.bashrc"
    sudo cp 10-cyclone-max.conf /etc/sysctl.d/
    ```

7. Configure the USB port:

    ```bash
    sudo cp 99-dynamixel.rules /etc/udev/rules.d && sudo cp 99-usb-microcontroller.rules /etc/udev/rules.d &&
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

8. Build the project (only some packages, as the rest are still in development):

    ```bash
    colcon build --packages-up-to dynamixel_msgs dynamixel_sdk robotis_device robotis_framework_common robotis_controller op3_description rf_gz_bridge op3_gazebo_ros2 op3_base_module op3_action_module experiment_module altair_walk_module altair_manager --executor sequential
    ```

9. Ready to walk!

10. Optional instruction

    Creating alias for command `colcon build --packages-select` as `only`

    ```bash
    echo "alias only='colcon build --packages-select'" >> ~/.bashrc
    ```

### Installation on Raspberry Pi 5 with Docker

1. Build the image (this will take some time):

    ```bash
    docker build -t altair_ws .
    ```

2. Run the compose:

    ```bash
    docker compose up -d
    ```

3. Interact with the Docker container:

    ```bash
    docker exec -it altair_ws /bin/bash
    ```

4. Build the project as described in the previous method.

## Running on Gazebo Simulation

Read altair-book

## Contributing

If you would like to contribute, please fork the repository or create a new branch. Pull requests are warmly welcome.

## Acknowledgments

- Hat tip to anyone whose code was used.
- Thanks to alfarobi_ws for the inspiration.
- Thanks to Bitbots for the inspiration.

## TODO

- Add odometry fuser.
- Add motion sequence for standing up.
- Add a way to show errors and pass them to the microcontroller screen.
- Add BehaviorTree as robot strategy.
