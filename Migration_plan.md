# Migration Plan: Altair → Yantravira (`Yantravira-urdf1`)
## 1. Model Diff Table

| Area | Current robot (`altair`) | Yantra (`Yantravira-urdf1`) | Migration action |
|---|---|---|---|
| **Entry file** | `src/altair_misc/altair_description/urdf/altair_compose.urdf.xacro` (ROS 2 xacro, robot name `altair`) | `Yantravira-urdf1/urdf/Yantravira-urdf1.urdf` (plain ROS 1 URDF, robot name `Yantravira-urdf1`) | Import into `altair_description`; keep `altair_compose.urdf.xacro` as the stable entry. Do **not** keep the ROS 2-invalid package name `Yantravira-urdf1`. |
| **Base frame / TF** | `base_link` → `base_footprint` (fixed, `z=-0.343`), `body_link` (fixed), `imu_link`, `cam_link`, `l_sole`, `r_sole` | Only root `base_link`; no `base_footprint`, `body_link`, `imu_link`, `cam_link`, or sole frames | Add compatibility frames; map the Yantra torso link to `body_link`. |
| **Joint names** | 20 actuated revolute joints: `l_hip_yaw/roll/pitch`, `l_knee`, `l_ank_pitch/roll`; mirrored `r_*`; `l_sho_pitch/roll`, `l_el`; mirrored `r_*`; `head_pan`, `head_tilt` | 22 joints / 21 actuated: `left_leg_yaw_1`, `left_leg_roll_1`, `left_leg_pitch_1/3/5`, `left_leg_roll_5`; `right_leg_yaw_2`, `right_leg_roll_2`, `right_leg_pitch_2/4`, `right_leg_roll_6`, `right_leg_pitch_6` (**fixed**); `left_arm_pitch_1`, `left_arm_roll_1`, `left_arm_pitch_3`, `left_claw`; mirrored right; `head_yaw`, `head_pitch` | Normalize to legacy names; add `l_claw`/`r_claw`; change `right_leg_pitch_6` to revolute unless the hardware really is 5-DOF on that leg. |
| **Joint types / limits** | All revolute; `effort=1000`, `velocity=100`, `lower=-pi*0.9`, `upper=pi*0.9`; damping `0.1` | Mixed `revolute`/`continuous`/`fixed`; `effort=0`, `velocity=0`; varied position limits; no `<dynamics>`; `right_leg_yaw_2` is continuous but has `lower=0 upper=0` | Replace zero effort/velocity, remove the continuous-joint limit, add damping, keep the new position limits. |
| **Mesh URI style** | `file://$(find altair_description)/meshes/altair/*.stl` | `package://Yantravira-urdf1/meshes/*.STL` | Copy to `meshes/yantravira/`; rewrite to `package://altair_description/meshes/yantravira/*.STL`. |
| **Plugins / sensors** | `gz_ros2_control` plugin, `gz-sim-sensors`, 20 transmissions, IMU sensor on `imu_link`, camera block commented out | None: no `<gazebo>`, `<transmission>`, `<ros2_control>`, or `<sensor>` tags | Add plugin, transmissions, IMU and camera sensors on the new frames. |
| **Launch / config** | ROS 2 Python launch; `worlds/altair.sdf` is a baked SDF with absolute `/home/lexciese/...` paths | ROS 1 XML `display.launch` / `gazebo.launch`; `joint_names_*.yaml` only | Discard Yantra launch files; replace baked SDF usage with URDF spawning; regenerate joint config. |

## 2. File Change Manifest

### Create

| Path | Change |
|---|---|
| `src/altair_misc/altair_description/meshes/yantravira/` | Copy all 23 `.STL` files from `Yantravira-urdf1/meshes/`; preserve exact case. |
| `src/altair_misc/altair_description/urdf/yantravira/yantravira.urdf.xacro` | New model: 23 links / 22 joints, legacy names, base/IMU/camera/sole frames, corrected limits and dynamics. |

### Modify

| Path | Change |
|---|---|
| `src/altair_misc/altair_description/urdf/altair_compose.urdf.xacro` | Replace old includes with the Yantra model; define `base_link`, `base_footprint`, `body_link`, `imu_link`, `cam_link`, `l_sole`, `r_sole`. |
| `src/altair_misc/altair_description/ros2_control/altair.ros2_control.xacro` | Add `l_claw`/`r_claw` to real, mock, and sim branches; keep 22-joint interface. |
| `src/altair_misc/altair_description/ros2_control/hardware_snippet.xacro` | Add claw joints to real/mock snippets; assign IDs 21/22 and offsets. |
| `src/altair_misc/altair_description/gazebo/altair.gazebo.xacro` | Add claw transmissions; retarget IMU/camera sensors to new frames; keep `gz_ros2_control` plugin. |
| `src/altair_misc/altair_description/launch/gazebo.launch.py` | Stop loading `worlds/altair.sdf`; use `empty.sdf` and spawn `robot_description`; enable RSP/controller manager. |
| `src/altair_misc/altair_description/urdf/initial_position.yaml` | Add `l_claw`/`r_claw` initial positions. |
| `src/altair_misc/altair_description/urdf/offset.yaml` | Add claw offsets. |
| `src/altair_misc/altair_moveit_config/config/altair.srdf` | Add claw links/joints to `Arms`/`All`; add claw collision-pair entries. |
| `src/altair_misc/altair_moveit_config/config/joint_limits.yaml` | Add claw limits. |
| `src/altair_misc/altair_moveit_config/config/moveit_controllers.yaml` | Add claws to arm/all controller joint lists. |
| `src/altair_misc/altair_moveit_config/config/ros2_controllers.yaml` | Add claws to arm/all controller joint lists. |
| `src/altair_manager/config/altair.robot` | Add claw Dynamixel entries with IDs 21/22. |
| `src/altair_manager/config/dxl_init_altair.yaml` | Add claw initialization entries. |
| `src/altair_manager/config/offset.yaml` | Add claw offsets. |
| `src/altair_data/config/joint_config.yaml` | Set `dxl_num: 22`; append claw IDs, types, names. |
| `src/altair_data/config/robots/robot_{1,2}/robot_config.yaml` | Add claw offsets. |
| `src/altair_misc/experiment/config/{dxl_init.yaml,offset.yaml,robot.yaml}` | Add claw entries if the experiment package remains in use. |
| `src/altair_dashboard/param_manager/sample params/params.yaml` | Add claw parameters if the dashboard is used. |
| `src/altair_misc/altair_description/rviz/altair.rviz` | Keep `odom` fixed frame; verify TF/RobotModel; change only if switching to `base_footprint`. |

`CMakeLists.txt` and `package.xml` need no change: `install(DIRECTORY ... meshes urdf ...)` already installs the new subdirectories.

### Remove

| Path | Change |
|---|---|
| `src/altair_misc/altair_description/urdf/altair/` | Remove all old structure/visual/inertia/collision xacros and `altair_old.urdf`. |
| `src/altair_misc/altair_description/urdf/altair.urdf` | Remove stale generated URDF with absolute paths and `odom` base. |
| `src/altair_misc/altair_description/urdf/altair_compose.urdf` | Remove stale generated flat URDF. |
| `src/altair_misc/altair_description/meshes/altair/` | Remove old meshes after the new model is verified. |
| `src/altair_misc/altair_description/worlds/altair.sdf` | Remove baked robot SDF with absolute `/home/lexciese/...` paths. |
| `src/altair_misc/altair_description/urdf/offset copy.yaml` | Remove duplicate/stale offset file. |
| `src/altair_misc/altair_description/urdf/offset altair 2.yaml` | Remove duplicate/stale offset file. |
| `Yantravira-urdf1/` | Remove the raw import folder after copying: ROS 1 `package.xml`, catkin `CMakeLists.txt`, `export.log`, empty `textures/`, ROS 1 launch files, and `joint_names_*.yaml`. |

## 3. Risks & Mismatches

- **Package name is invalid for ROS 2.** `Yantravira-urdf1` contains uppercase letters and a hyphen, and its `package.xml` is catkin/format 2. If used as-is, `package://Yantravira-urdf1/...` will not resolve. Import into `altair_description` or rename to `yantravira_description` with an ament manifest.
- **Right leg is asymmetric.** `right_leg_pitch_6` is `fixed` while the mirrored left joint `left_leg_pitch_5` is `revolute`. Decide whether this is intentional (right leg = 5-DOF) or an export typo. If typo, change to `revolute`; if intentional, remove `r_ank_pitch` from walk/IK/MoveIt assumptions.
- **Missing TF frames break existing consumers.** The walking controllers use `l_sole`/`r_sole`, the IMU broadcaster uses `imu_link`, vision uses `cam_link`, and MoveIt uses `body_link`/`*_link` names. Add fixed frames or update every consumer.
- **Joint/link rename decision.** Recommended: normalize Yantra names to legacy names and links so current controllers/MoveIt configs keep working:
  - Legs: `left_leg_{yaw_1,roll_1,pitch_1,pitch_3,pitch_5,roll_5}` → `{l_hip_yaw,l_hip_roll,l_hip_pitch,l_knee,l_ank_pitch,l_ank_roll}`; right `{yaw_2,roll_2,pitch_2,pitch_4,pitch_6,roll_6}` → `{r_hip_yaw,r_hip_roll,r_hip_pitch,r_knee,r_ank_pitch,r_ank_roll}`.
  - Arms: `left_arm_{pitch_1,roll_1,pitch_3}` → `{l_sho_pitch,l_sho_roll,l_el}`; `left_claw` → `l_claw`; mirror for right.
  - Head: `head_yaw` → `head_pan`, `head_pitch` → `head_tilt`.
  - Links: same names with `_link` suffix; rename Yantra `base_link` to `body_link`; add new root `base_link`, plus `base_footprint`, `imu_link`, `cam_link`, `l_sole`, `r_sole`, `l_claw_link`, `r_claw_link`.
  - If you keep Yantra names instead, you must update all of: `ros2_control/*.xacro`, `gazebo/altair.gazebo.xacro`, `urdf/initial_position.yaml`, `urdf/offset*.yaml`, `altair_moveit_config/config/{altair.srdf,joint_limits.yaml,moveit_controllers.yaml,ros2_controllers.yaml}`, `altair_manager/config/*`, `altair_data/config/*`, `altair_motion/altair_animation/data/hello.yaml`, `altair_motion/head_controller/src/head_controller.cpp`, `altair_tools/offset_editor/src/offset_editor.cpp`, and the `quintic_walk`/`cubic_walk`/`b_splines_walk`/`zmp_walk`/`bezier_kick` sources that hardcode `l_sole`/`r_sole`/`base_link`.
- **Zero limits and missing dynamics.** All Yantra joints have `effort="0"` and `velocity="0"`, and no damping/friction. MoveIt and ros2_control may reject or behave erratically. Replace with hardware-safe values; current model uses `1000`/`100` and damping `0.1`.
- **Continuous joint with a limit.** `right_leg_yaw_2` is `continuous` but has `lower="0" upper="0"`. Remove the limit block or make it revolute with real limits.
- **No transmission / ros2_control / sensor tags.** Gazebo will load a passive model unless transmissions or ros2_control tags are added; the current `gz_ros2_control` plugin expects them. IMU and camera sensors also need explicit `<gazebo reference="...">` blocks.
- **Mesh alignment risk.** Yantra `base_link` visual/collision origin is approximately `(-0.765, 1.466, -1.053)`, and several joint axes are slightly tilted. Verify in RViz/Gazebo before accepting; if the mesh appears displaced or the legs are skewed, re-export with local coordinates or correct the origins.
- **Old baked SDF will silently load the old robot.** `worlds/altair.sdf` contains the old `altair` model with absolute mesh paths. If left in place and `gazebo.launch.py` still points to it, the new URDF is ignored.
- **Some motion launch files load `alfarobi_compose.urdf.xacro`, not `altair_compose.urdf.xacro`.** Decide whether those packages are in scope; replacing `altair_description` alone will not change them.
- **`config/joint_names_Yantravira-urdf1.yaml` has an empty first joint name.** Do not import it blindly; regenerate from the normalized joint list.

## 4. Execution Order

1. Decide the interface strategy: normalize Yantra names to legacy names (recommended) or keep Yantra names and update all downstream configs; confirm `right_leg_pitch_6` and claw actuation.
2. Copy the 23 Yantra meshes into `meshes/yantravira/` and create `urdf/yantravira/yantravira.urdf.xacro` with corrected URIs, limits, dynamics, joint types, and compatibility frames.
3. Rewrite `urdf/altair_compose.urdf.xacro` to include the new model and preserve the `base_link`/`base_footprint`/`body_link`/`imu_link`/`cam_link`/`l_sole`/`r_sole` TF tree; remove old model files.
4. Extend `ros2_control` and Gazebo xacros with the 22-joint interface, claw joints, transmissions, IMU, and camera.
5. Update `launch/gazebo.launch.py` to spawn the URDF in an empty world; remove `worlds/altair.sdf`.
6. Update MoveIt SRDF/controllers/limits and all offset/initial-position/Dynamixel-ID configs for the claws.
7. Build and run the XML/kinematics checks; launch RViz and Gazebo; verify TF, joint states, controllers, and MoveIt.
8. Retune walking/IK parameters for the new link lengths and axes; commit only after the model spawns cleanly.

## 5. Verification Commands

```bash
cd /home/zmerrr/Downloads/altair_lab
source /opt/ros/jazzy/setup.bash

DESC=src/altair_misc/altair_description
xacro "$DESC/urdf/altair_compose.urdf.xacro" \
  sim_mode:=false mock_mode:=true real_mode:=false \
  offset_file:="$PWD/$DESC/urdf/offset.yaml" \
  initial_position_file:="$PWD/$DESC/urdf/initial_position.yaml" \
  > /tmp/altair.urdf

xmllint --noout /tmp/altair.urdf
check_urdf /tmp/altair.urdf
gz sdf -p /tmp/altair.urdf > /tmp/altair.sdf
```

```bash
cd /home/zmerrr/Downloads/altair_lab
colcon build --packages-select altair_description altair_moveit_config --symlink-install
source install/setup.bash
```

```bash
# RViz / TF / joint states
ros2 launch altair_description altair_view_robot.launch.py gui:=true
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link base_footprint
ros2 run tf2_ros tf2_echo base_link body_link
ros2 run tf2_ros tf2_echo l_ank_roll_link l_sole
ros2 run tf2_ros tf2_echo r_ank_roll_link r_sole
ros2 run tf2_ros tf2_echo head_tilt_link cam_link
ros2 topic echo /joint_states --once
```

```bash
# Gazebo spawn and ros2_control
ros2 launch altair_description gazebo.launch.py
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
gz model --list
gz model -m altair --pose
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic echo /clock --once
```

```bash
# MoveIt kinematics/controller load
ros2 launch altair_moveit_config move_group.launch.py
ros2 action list | grep -E 'move_group|execute_trajectory'
```

```bash
# Mesh URI / count sanity check
find install/altair_description/share/altair_description/meshes/yantravira -name '*.STL' | wc -l   # expect 23
grep -o 'package://[^"]*' /tmp/altair.urdf | sort -u
```
