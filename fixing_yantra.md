# Fixing Yantra Simulation & Control Pipeline

This document records the issues observed in the ROS 2 Jazzy + Gazebo simulation logs for the Yantra humanoid robot, their root causes, and a concrete phased plan for fixing them.

---

## 1. Summary of Simulation Launch Test

### Test Command Executed
1. **MoveIt Group:**
   ```bash
   cd ~/Downloads/altair_lab
   export ROS_DOMAIN_ID=96
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch yantra_moveit_config move_group.launch.py
   ```
2. **Gazebo Sim + Controllers:**
   ```bash
   cd ~/Downloads/altair_lab
   export ROS_DOMAIN_ID=96
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch quintic_walk yantra_gazebo.launch.py
   ```

### Status of Controllers
All three controllers loaded, configured, and activated successfully:
- `joint_state_broadcaster` (Active)
- `imu_sensor_broadcaster` (Active)
- `quintic_walk` (Active)

Despite successful activation, several runtime errors, clock desynchronizations, and configuration warnings were detected in the log.

---

## 2. Issues Observed in the Logs & Root Causes

### A. Time Desynchronization & TF Buffer Time Jumps (High Priority)
* **Log Symptoms:**
  ```text
  [move_group]: [WARN] TF_OLD_DATA ignoring data from the past for frame base_link at time 4.509000 according to authority Authority undetectable
  [gazebo-1] [WARN] [tf2_buffer]: Detected jump back in time. Clearing TF buffer.
  ```
* **Root Causes:**
  1. `move_group.launch.py` launches `move_group` and `robot_state_publisher` without setting `use_sim_time:=true`. As a result, MoveIt stamps TFs using system wall-clock time (~1.79B seconds epoch), while Gazebo publishes sim-time starting at `0.0s`.
  2. Multiple orphaned `parameter_bridge` processes from earlier runs were actively publishing competing clocks to `/clock`.

### B. Duplicate `robot_state_publisher` & Resource Manager Reload Collisions (Medium Priority)
* **Log Symptoms:**
  ```text
  WARNING: Be aware that there are nodes in the graph that share an exact name, which can have unintended side effects.
  /robot_state_publisher (multiple instances active)
  [gazebo-1] [WARN] [controller_manager]: ResourceManager has already loaded a urdf and is initialized. Ignoring attempt to reload a robot description.
  ```
* **Root Causes:**
  Both `move_group.launch.py` and `yantra_gazebo.launch.py` independently launch a `robot_state_publisher` node publishing to `/robot_description` and `/tf`. This results in graph name collisions and controller_manager discarding duplicate URDF publications.

### C. KDL Root Link Inertia Warning (Medium Priority)
* **Log Symptoms:**
  ```text
  [robot_state_publisher] [WARN] [kdl_parser]: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia. As a workaround, you can add an extra dummy link to your URDF.
  ```
  *(Repeats 6 times across `robot_state_publisher` and kinematic solver groups: `LeftLeg`, `RightLeg`, `RightArm`, `LeftArm`, `Head`).*
* **Root Causes:**
  In `src/yantra_misc/yantra_description/urdf/yantra_compose.urdf.xacro`, `<link name="base_link">` includes an `<inertial>` block with mass `0.001`. In KDL and ROS URDF conventions, the root link must not have inertia (the physical torso mass is already specified on `body_link`).

### D. MoveIt Controller Manager Configuration Errors (Medium Priority)
* **Log Symptoms:**
  ```text
  [ERROR] [move_group.moveit.moveit.plugins.simple_controller_manager]: No controller_names specified.
  [ERROR] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
  ```
* **Root Causes:**
  In `src/yantra_misc/yantra_moveit_config/config/moveit_controllers.yaml`, MoveIt 2 in ROS 2 Jazzy expects controller parameter structures formatted for the current `moveit_simple_controller_manager` plugin schema. Also, octomap sensor monitoring is active without sensor input definitions.

### E. SDF Element Warnings for Gazebo Sensors (Low Priority)
* **Log Symptoms:**
  ```text
  Warning [Utils.cc:132] [/sdf/model[@name="yantra"]/link[@name="base_link"]/sensor[@name="imu_sensor"]/gz_frame_id]: XML Element[gz_frame_id], child of element[sensor], not defined in SDF. Copying[gz_frame_id] as children of [sensor].
  ```
  *(Same warning for camera sensor).*
* **Root Causes:**
  In `src/yantra_misc/yantra_description/gazebo/yantra.gazebo.xacro`, `<gz_frame_id>` is specified inside `<sensor>` without the appropriate SDFormat namespace/attribute formatting.

### F. ROS 2 Parameter Deprecations & Real-Time Scheduling Notice (Low Priority)
* **Log Symptoms:**
  ```text
  [quintic_walk]: Using the legacy anti-windup technique is deprecated. This option will be removed by the ROS 2 Kilted Kaiju release.
  [quintic_walk]: The parsed 'thread_priority' parameter will be deprecated and not be functional from ROS 2 Lyrical Luth release. Please use the 'async_parameters.thread_priority' parameter instead.
  [AsyncFunctionHandler]: Could not enable FIFO RT scheduling policy.
  [gz_ros_control]: Desired controller update period (0.0133333 s) is slower than the gazebo simulation period (0.001 s).
  ```
* **Root Causes:**
  1. `src/yantra_motion/quintic_walk/config/gazebo_sim/walking.yaml` defines `thread_priority: 50` rather than the updated key `async_parameters.thread_priority: 50`.
  2. The Linux user environment lacks real-time FIFO scheduling privileges in `/etc/security/limits.conf`.
  3. Controller update rate in `controller.yaml` is set to 75 Hz (0.0133s) while Gazebo physics runs at 1000 Hz (0.001s).

### G. Residual Legacy Name Reference
* `src/yantra_misc/yantra_moveit_config/launch/spawn_controllers.launch.py` still references `"alfarobi"` instead of `"yantra"`.

---

## 3. Concrete Plan for Implementing Fixes

### Phase 1: Environment & Process Cleanup
1. Kill any stale/orphaned processes lingering from past test runs:
   ```bash
   pkill -9 -f parameter_bridge || true
   pkill -9 -f robot_state_publisher || true
   pkill -9 -f move_group || true
   pkill -9 -f "gz sim" || true
   ```

### Phase 2: Time Synchronization & Node Deduplication
1. **Update `src/yantra_misc/yantra_moveit_config/launch/move_group.launch.py`:**
   - Add launch argument `use_sim_time` (default `true` for simulation runs).
   - Pass `{"use_sim_time": LaunchConfiguration("use_sim_time")}` to `move_group`.
   - Add launch argument `publish_robot_state` (default `false` when launching together with Gazebo) to prevent spawning duplicate `robot_state_publisher` nodes.
2. **Clock Authoritativeness:**
   - Ensure only a single `/clock` bridge is spawned by `yantra_gazebo.launch.py`.

### Phase 3: URDF & Gazebo Sensor Model Fixes
1. **Remove `base_link` Inertia in `yantra_compose.urdf.xacro`:**
   - Remove `<inertial>` from `base_link` in `src/yantra_misc/yantra_description/urdf/yantra_compose.urdf.xacro`.
   - Preserve `<inertial>` on `body_link` (torso) where the robot's physical mass belongs.
   - Eliminates KDL parser root inertia warnings across MoveIt and `robot_state_publisher`.
2. **Correct Sensor Tags in `yantra.gazebo.xacro`:**
   - Update sensor frame definitions to conform to SDFormat specifications to eliminate element copy warnings.

### Phase 4: MoveIt Config & Parameter Cleanup
1. **Fix `moveit_controllers.yaml`:**
   - Format controller descriptions under the structure expected by `moveit_simple_controller_manager` in ROS 2 Jazzy.
2. **Migrate Deprecated Parameters in `walking.yaml`:**
   - Change `thread_priority: 50` to `async_parameters.thread_priority: 50`.
3. **Fix Legacy Package Call in `spawn_controllers.launch.py`:**
   - Change `"alfarobi"` to `"yantra"`.

### Phase 5: Rebuild & End-to-End Verification
1. Rebuild the modified packages:
   ```bash
   cd ~/Downloads/altair_lab
   export ROS_DOMAIN_ID=96
   export PATH=/usr/bin:$PATH
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select yantra_description yantra_moveit_config quintic_walk --symlink-install
   source install/setup.bash
   ```
2. Run MoveIt in Terminal 1 and Gazebo simulation in Terminal 2.
3. Validate:
   - No `TF_OLD_DATA` or `jump back in time` warnings appear.
   - Exactly one `robot_state_publisher` exists (`ros2 node list`).
   - KDL root inertia warnings are resolved.
   - All three controllers configure and activate smoothly.
