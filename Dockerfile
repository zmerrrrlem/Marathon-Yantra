FROM ros:jazzy-ros-base
# Packages: [ament_cmake, ament_cmake_auto, ament_cmake_gmock, ament_cmake_gtest, ament_cmake_pytest, ament_cmake_ros, ament_index_cpp, ament_index_python, ament_lint_auto,
# ament_lint_common, class_loader, common_interfaces, launch, launch_ros, launch_testing, launch_testing_ament_cmake, launch_testing_ros, launch_xml, launch_yaml, 
# pluginlib, rcl_lifecycle, rclcpp, rclcpp_action, rclcpp_lifecycle, rclpy, ros2cli_common_extensions, ros2launch, ros_environment, rosidl_default_generators,
# rosidl_default_runtime, sros2, sros2_cmake, geometry2, kdl_parser, robot_state_publisher, rosbag2, urdf]
# TODO: might need to use ros:jazzy-ros-perception 

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y locales software-properties-common curl wget nano vim \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && export LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*

# ==== Setup Python 3 Packages Dependencies ====
#  Current Python 3 Version: 3.12.3
RUN apt-get update \
  && apt-get install -y --no-install-recommends python3-pip python3-serial \
  && rm -rf /var/lib/apt/lists/*

# ==== Install ROS 2 Packages & Dependencies ====
RUN apt-get update \
  && apt-get install -y --no-install-recommends ros-jazzy-xacro ros-jazzy-rqt ros-jazzy-rviz2 ros-jazzy-yaml-cpp-vendor ros-jazzy-vision-msgs ros-jazzy-nav-msgs ros-jazzy-orocos-kdl-vendor \
  ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui  ros-jazzy-joint-limits ros-jazzy-interactive-markers \
  ros-jazzy-image-geometry ros-jazzy-imu-sensor-broadcaster ros-jazzy-kinematics-interface ros-jazzy-libyaml-vendor libserial-dev libserialport-dev libserial1 \
  ros-jazzy-examples-rclcpp-minimal-subscriber ros-jazzy-examples-rclcpp-minimal-publisher ros-jazzy-dynamixel-sdk \
  && rm -rf /var/lib/apt/lists/*

# ==== Setup ros2_control Packages Dependencies ====
RUN apt-get update \
  && apt-get install -y --no-install-recommends ros-jazzy-ros2-control ros-jazzy-hardware-interface ros-jazzy-forward-command-controller ros-jazzy-filters ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs \
  ros-jazzy-control-toolbox ros-jazzy-controller-interface ros-jazzy-ros2controlcli ros-jazzy-ros2component ros-jazzy-pid-controller ros-jazzy-joint-state-broadcaster \
  ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim \
  && rm -rf /var/lib/apt/lists/*

# ==== Setup Moveit 2 ====
RUN apt-get update && \
  apt-get install -y --no-install-recommends ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds ros-jazzy-moveit ros-jazzy-moveit-ros-control-interface ros-jazzy-ompl \
  && rm -rf /var/lib/apt/lists/*

# ==== Setup Dynamixel SDK ====
# Use the fork of Dynamixel SDK to fix compilation error on linux_sbc of the original repository
RUN cd /opt && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK
RUN cd /opt/DynamixelSDK/c++/build/linux_sbc \
  && make && sudo make install

# ==== Setup OpenCV ====
RUN apt-get update && \
  apt-get install -y --no-install-recommends v4l-utils ros-jazzy-usb-cam \
  && rm -rf /var/lib/apt/lists/*

# === Install OpenCV ===

# Install base dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    wget \
    unzip \
    pkg-config \
    libgtk2.0-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    python3-dev \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Download OpenCV and opencv_contrib
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip && \
    unzip opencv.zip && \
    unzip opencv_contrib.zip

RUN apt-get update && apt-get install -y \
    libopenmpi-dev \
    libhdf5-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Create build directory and configure OpenCV
RUN mkdir -p opencv-4.x/build && cd opencv-4.x/build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.x/modules \
          -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_opencv_python3=ON \
          -DINSTALL_C_EXAMPLES=OFF \
          -DINSTALL_PYTHON_EXAMPLES=OFF \
          -DPYTHON3_EXECUTABLE=/usr/bin/python3 \
          -DPYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3 \
          -DBUILD_TESTS=OFF \
          -D WITH_GTK=ON \
          -D WITH_QT=ON \
          ..

# Build and install OpenCV
RUN cd opencv-4.x/build && make -j$(nproc) && make install && ldconfig

# Remove the default 'ubuntu' user
RUN deluser --remove-home ubuntu || true

# ==== Create a non-root user ====
ARG USERNAME=altair01
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

# ==== Setup sudo ====
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Install both Wayland and X11 GUI dependencies
RUN apt-get update && apt-get install -y \
    qtwayland5 \
    x11-apps \
    weston \
    libgl1 \
    libglu1-mesa \
    mesa-utils \
    libglvnd0 \
    libglx0 \
    qtbase5-dev \
    qt5-qmake \
    && apt-get clean

RUN apt-get update && apt-get install -y bash-completion libncurses-dev

# Set Wayland as the default display server, but will be overridden if X11 is detected
ENV WAYLAND_DISPLAY=wayland-1

# Copy udev rules files first but don't try to reload udev (not running in container)
RUN apt-get update && apt-get install -y udev && apt-get clean
COPY 99-dynamixel.rules 99-usb-microcontroller.rules /tmp/
RUN mkdir -p /etc/udev/rules.d && \
    cp /tmp/99-dynamixel.rules /etc/udev/rules.d && \
    cp /tmp/99-usb-microcontroller.rules /etc/udev/rules.d
    
# Note: udev rules will only apply when device is connected to host
# The rules are copied for reference and documentation purposes

RUN apt remove ros-jazzy-dynamixel-sdk -y

# Ensure the ROS environment is sourced for interactive shells
ARG ROS_DOMAIN_ID=0
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /root/.bashrc \
    && echo "alias only='colcon build --packages-select'" >> /root/.bashrc \
    && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-96}' >> /root/.bashrc \
    && echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /root/.bashrc \
    && echo "alias ls='ls --color=auto'" >> /root/.bashrc \
    && echo "alias grep='grep --color=auto'" >> /root/.bashrc \
    && echo "export TERM=xterm-256color" >> /root/.bashrc

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/$USERNAME/.bashrc \
    && echo "alias only='colcon build --packages-select'" >> /home/$USERNAME/.bashrc \
    && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-96}' >> /home/$USERNAME/.bashrc \
    && echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/$USERNAME/.bashrc \
    && echo "alias ls='ls --color=auto'" >> /home/$USERNAME/.bashrc \
    && echo "alias grep='grep --color=auto'" >> /home/$USERNAME/.bashrc \
    && echo "export TERM=xterm-256color" >> /home/$USERNAME/.bashrc

# ==== .bashrc Config ====
RUN echo "#!/usr/bin/env bash" > /rcl_entrypoint.sh
RUN echo "source /opt/ros/jazzy/setup.bash" >> /rcl_entrypoint.sh
# use rmw_cyclonedds_cpp as the default middleware
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /rcl_entrypoint.sh 
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /rcl_entrypoint.sh
RUN echo "alias only='colcon build --packages-select'" >> /rcl_entrypoint.sh
RUN echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-96}' >> /rcl_entrypoint.sh
RUN echo 'echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"' >> /rcl_entrypoint.sh
RUN echo 'export PS1="\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /rcl_entrypoint.sh
RUN echo "alias ls='ls --color=auto'" >> /rcl_entrypoint.sh
RUN echo "alias grep='grep --color=auto'" >> /rcl_entrypoint.sh
RUN echo "export TERM=xterm-256color" >> /rcl_entrypoint.sh

# Add display server detection
RUN echo '# Auto-detect display server and set appropriate variables' >> /rcl_entrypoint.sh
RUN echo 'if [ -n "$WAYLAND_DISPLAY" ] && [ -z "$DISABLE_WAYLAND" ]; then' >> /rcl_entrypoint.sh
RUN echo '  echo "Wayland detected. Using Wayland display server."' >> /rcl_entrypoint.sh
RUN echo '  export QT_QPA_PLATFORM=wayland' >> /rcl_entrypoint.sh
RUN echo '  export XDG_SESSION_TYPE=wayland' >> /rcl_entrypoint.sh
RUN echo '  export GDK_BACKEND=wayland' >> /rcl_entrypoint.sh
RUN echo 'else' >> /rcl_entrypoint.sh
RUN echo '  echo "X11 detected or Wayland disabled. Using X11 display server."' >> /rcl_entrypoint.sh
RUN echo '  unset WAYLAND_DISPLAY' >> /rcl_entrypoint.sh
RUN echo '  export QT_QPA_PLATFORM=xcb' >> /rcl_entrypoint.sh
RUN echo '  export XDG_SESSION_TYPE=x11' >> /rcl_entrypoint.sh
RUN echo '  export GDK_BACKEND=x11' >> /rcl_entrypoint.sh
RUN echo '  export QT_X11_NO_MITSHM=1' >> /rcl_entrypoint.sh
RUN echo 'fi' >> /rcl_entrypoint.sh

RUN echo 'exec "$@"' >> /rcl_entrypoint.sh
RUN chmod +x /rcl_entrypoint.sh

# Start bash on altair_ws folder
WORKDIR /home/$USERNAME/altair_ws

# only dynamixel_workbench_toolbox altair_description altair_moveit_config biped_interfaces altair_msgs ros2_python_extension altair_hardware bio_ik rot_conv altair_splines quintic_walk

# ------------ ENTRYPOINT AND LOOP ------------
ENTRYPOINT ["/rcl_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]