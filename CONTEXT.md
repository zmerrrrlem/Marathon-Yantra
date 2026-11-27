# Workspace Context — Yantra humanoid (ROS 2 Jazzy + Gazebo)

Read this before editing or launching anything in this workspace. It records the
Altair → Yantra rename and the current build/launch state.

## 1. Identity and paths

- Workspace root: `/home/zmerrr/Downloads/altair_lab` (source, build, install, log all live here).
- Always use `export ROS_DOMAIN_ID=96` for every ROS 2 command, in every terminal.
- Declared robot name is `yantra` everywhere: URDF `<robot name="yantra">`, SRDF, Gazebo `-name yantra`.
- Canonical description package: `yantra_description` at `src/yantra_misc/yantra_description`.
- Canonical composed model: `urdf/yantra_compose.urdf.xacro`, which includes
  `urdf/yantravira/yantravira.urdf.xacro` (Yantra-derived geometry with normalized legacy
  joint names) and injects ros2_control, the gz_ros2_control plugin, sensors and compatibility
  frames (`body_link`, `imu_link`, `cam_link`, `l_sole`, `r_sole`).
- Meshes: `package://yantra_description/meshes/yantravira/*.STL` (23 files).

## 2. Rename status (Altair → Yantra)

A full internal rename was applied: 247 text files, 97 renamed paths. All `altair_*` ROS
packages became `yantra_*`: `yantra_description`, `yantra_msgs`, `yantra_data`,
`yantra_manager`, `yantra_bringup`, `yantra_dashboard`, `yantra_framework`,
`yantra_hardware`, `yantra_misc`, `yantra_motion`, `yantra_tools`, `yantra_vision`,
`yantra_vision_deeplearning`, `yantra_animation`, `yantra_calibration`, `yantra_teleop`,
`yantra_moveit_config`. Launch files are now `yantra*.launch.py`; identity/config files
are `yantra.srdf`, `yantra.rviz`, `yantra.robot`, `yantra.ros2_control.xacro`,
`yantra.gazebo.xacro`. Xacro macros are `yantra_ros2_control` and `yantra_gazebo`.

Deliberately NOT renamed (external identifiers): `altair_ws` clone URL, Docker
service/image/tag and WORKDIR, `altair01` container user, `altair_lab` on-disk path,
`Altair-Humanoid-GMRT` GitHub org in `.gitmodules`, maintainer email addresses,
`Altair GMRT` team name. Also left untouched: `Migration_plan.md` and `task/task1.md` —
archival records of the pre-rename migration.

Do NOT re-add `altair_*` compatibility aliases; the old names are intentionally gone.
`git status` shows the renames as mass deletions plus a few collapsed untracked
directories — that is normal for unstaged directory renames, not missing files.

## 3. Build state

Currently built and installed with the new names: `yantra_description`, `yantra_msgs`,
`quintic_walk`, `yantra_moveit_config`. Stale `build/altair_*` and `install/altair_*`
directories were removed. Other packages may still have stale symlink-install artifacts;
rebuild them if a launch fails on a missing file with an old path.

One-time build:

```
cd ~/Downloads/altair_lab
export ROS_DOMAIN_ID=96
source /opt/ros/jazzy/setup.bash
colcon build --packages-select yantra_description yantra_msgs quintic_walk yantra_moveit_config --symlink-install
source install/setup.bash
```

Build gotchas (all hit and solved here):
- This shell can have a Hermes Python 3.14 venv first on PATH. ROS Jazzy needs system
  Python 3.12. Use `export PATH=/usr/bin:$PATH` before building, and pass
  `-DPython3_EXECUTABLE=/usr/bin/python3 -DPython3_INCLUDE_DIR=/usr/include/python3.12
  -DPython3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.12.so
  -DPython3_NumPy_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include`
  via `--cmake-args` for message-generation packages.
- After renaming a package directory, delete its `build/<pkg>` directory: stale
  `CMakeCache.txt` records the old source path and CMake refuses to rebuild.
- With `--symlink-install`, renaming source files leaves dangling symlinks in `install/`.
  Remove them and rebuild the owning package.
- A full workspace build is blocked by a pre-existing dependency gap
  (`robotis_math` was never installed). Build only the packages you need.

## 4. How to run

Terminal 1 — MoveIt (MUST start first; it publishes the SRDF the walk controller needs):

```
cd ~/Downloads/altair_lab
export ROS_DOMAIN_ID=96
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch yantra_moveit_config move_group.launch.py
```

Wait for `You can start planning now!`.

Terminal 2 — Gazebo + controllers:

```
cd ~/Downloads/altair_lab
export ROS_DOMAIN_ID=96
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch quintic_walk yantra_gazebo.launch.py
```

Wait 10–15 s. Expected: `Entity creation successful`, `Robot model loaded.`, and
`Configured and activated` for `joint_state_broadcaster`, `imu_sensor_broadcaster`,
`quintic_walk`. On a headless machine the launch automatically uses `gz sim -s`
(server only, no GUI). Do not run terminal 2 alone — without `move_group` the walk
controller has no SRDF and configure fails (it now fails fast instead of hanging).

Terminal 3 — checks / command:

```
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Always publish walking commands with `-r 10`; a one-shot publish leaves the controller
repeating zero velocity. Shutdown: Ctrl+C terminal 2, then terminal 1. If a headless
`gz sim` survives, `pkill -9 gz`.

## 5. Launch-level fixes already applied

- `src/yantra_motion/quintic_walk/launch/yantra_gazebo.launch.py`:
  - `gnome-terminal` helper is skipped when no `DISPLAY`/`WAYLAND_DISPLAY` exists.
  - headless machines get `gz_args = "-v 4 -s -r empty.sdf"` instead of a crashing GUI.
  - the three controller spawners are wrapped in a 10 s `TimerAction`, because Gazebo
    hosts the controller manager via the `gz_ros2_control` URDF plugin. Never add a
    standalone `ros2_control_node` to a Gazebo launch — it loops on "Waiting for data
    on 'robot_description' topic".
- `src/yantra_motion/quintic_walk/src/quintic_walk.cpp` `on_configure`: the
  `SyncParametersClient` call to `move_group` is guarded with
  `wait_for_service(std::chrono::seconds(2))` and skips on timeout. Previously a missing
  `move_group` blocked configure forever and exhausted spawner retries. The same
  unguarded pattern still exists in `b_splines_walk/src/b_splines_walk.cpp` and
  `cubic_walk/src/cubic_walk.cpp` if those packages are used.

## 6. Verification commands (already proven)

```
source /opt/ros/jazzy/setup.bash && source install/setup.bash
xacro src/yantra_misc/yantra_description/urdf/yantra_compose.urdf.xacro sim_mode:=true > /tmp/yantra.urdf
xmllint --noout /tmp/yantra.urdf && check_urdf /tmp/yantra.urdf
ros2 launch yantra_moveit_config move_group.launch.py --show-args
ros2 launch quintic_walk yantra_gazebo.launch.py --show-args
```

Expanded URDF declares `yantra`; all mesh URIs use `package://yantra_description/...`.

## 7. Non-fatal noise you will see

- KDL warning: root link `base_link` has inertia.
- MoveIt: "No 3D sensor plugin(s) defined for octomap updates" and
  "No controller_names specified." (pre-existing config quirk; move_group still plans).
- Headless sim: `tf2_buffer: Detected jump back in time` warnings.
- `cd ~/Downloads/altair_lab` is mandatory; running colcon or sourcing elsewhere
  silently operates on stale files.
