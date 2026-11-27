# Fixes for "robot falls apart in Gazebo" (2026-11-27)

Only 5 files changed. Visual meshes, joint positions and masses are unchanged.

## 1. Joint force limits were 1000 N·m (main cause)
`urdf/yantravira/yantravira.urdf.xacro`: every `<limit>`.

gz_ros2_control drives position-controlled joints with a motor whose maximum force is the URDF
`effort` limit. At 1000 N·m (and 100 rad/s) it could hit links weighing 13–240 g with enormous
forces, so the solver could not keep the joints together. That produced the gaps at the head, feet and claws.

| Joints | Before | After |
|---|---|---|
| hip, knee, ankle | effort 1000, velocity 100 | effort **12**, velocity **7** |
| shoulder, elbow, head | effort 1000, velocity 100 | effort **6**, velocity **7** |
| claws | effort 500, velocity 50 | effort **2**, velocity **7** |

These are roughly Dynamixel MX-64/MX-106-class values. If the robot can't hold its squat, raise the leg value (e.g. to 20).

## 2. Every link's centre of mass was at its joint, and the torso inertia was ~20× too small
`urdf/yantravira/yantravira.urdf.xacro` (all limb and head links) and `urdf/yantra_compose.urdf.xacro` (`body_link`).

- Every `<inertial><origin>` was `0 0 0`, the joint position, even though the parts extend up to
  9 cm from their joints (e.g. the hip-pitch part's centre is 93 mm below its joint).
- `body_link`: 2.8 kg with inertia ≈ 5e-4 kg·m². A part that size and mass is ≈ 1e-2 kg·m².

The masses are kept as they were. The centre of mass and full inertia tensor of each link were
computed from its STL mesh (assuming uniform density) and scaled to that mass.

## 3. Collision meshes replaced with boxes
Same two files. The 23 collision meshes (up to 47,924 triangles each) are replaced by one box per
link, fitted to the mesh. This speeds up the simulation and makes contact stable. The `*_ank_roll_link`
collisions are removed, so the existing `l_sole`/`r_sole` boxes are the only foot contact.
Self-collision is off, so the upper-body boxes only matter if the robot falls over.

## 4. Two controller managers were running
`launch/gazebo.launch.py`

The launch file started a standalone `ros2_control_node` called `controller_manager`, while the
gz_ros2_control plugin inside Gazebo already runs `/controller_manager`. Two nodes with the same
name caused the spawner timeouts ("Failed to acquire lock", `configure_controller` timeouts).
- Removed the standalone `controller_manager` node.
- Spawners now start in order: `joint_state_broadcaster` → `imu_sensor_broadcaster` + `controller_experiment_1` → `quintic_walk`.

## 5. Claw velocity NaN (cosmetic)
`ros2_control/hardware_snippet.xacro` (sim/mock macro). `l_claw`/`r_claw` had no `velocity`
state interface, so joint_state_broadcaster filled in NaN. Added `<state_interface name="velocity"/>`.
This was not a physics problem.

## 6. World file was invalid
`worlds/empty.sdf`
- `<physics ... default>`: an attribute with no value is not valid XML. Changed to `default='true'`.
- `<max_step>` and `<kPA>` are not SDF tags, so Gazebo ignored them. Replaced with `<max_step_size>0.001</max_step_size>` and `<real_time_factor>1.0</real_time_factor>`.

Your Gazebo screenshots show the world named **default**, not **empty**. The launch may not be
loading this file, or it failed to parse. `launch/gazebo.control.py` assumes the world is called `empty`.

## 7. Mesh placement was wrong (head floating, feet detached), fixed in round 2
`urdf/yantravira/yantravira.urdf.xacro`, `urdf/yantra_compose.urdf.xacro`, and a new mesh `meshes/yantravira/neck_servo.STL`.

The conversion that produced `yantravira.urdf.xacro` baked a rotation into every mesh, but it was
**missing a 90° yaw**, and every mesh had a different leftover offset. At zero pose, neighbouring parts sat
20–40 mm apart (hip roll 37 mm, ankle 23 mm, shoulder 19 mm). RViz and Gazebo drew exactly that.
The old `meshes/yantra` set, with the same joints, fits together with 0 mm gaps.

How it was fixed:
- Each yantravira part was matched to the identical old part (fit error ≈ 1 mm) to get its correct `<visual> <origin>`.
- Parts with no identical old part (ankle roll/foot, elbow, claw) take the correction of the neighbour they already fit.
  The torso was fitted to the old torso (90° yaw + 24 mm shift).
- **The new torso export is missing the head-pan servo and neck mount** (it ends 22 mm below the head).
  That section was cut from the old `body.stl` and added as a second visual on `body_link` (`neck_servo.STL`).
- Collision boxes and centres of mass were recomputed from the corrected placement.

Result at zero pose: every joint has a gap under 1 mm between its two parts (elbows 3–4 mm, because the elbow part differs from the old one).
The foot meshes now sit exactly on the sole contact boxes (bottom at −343 mm, matching `base_footprint`).

## Not changed: check these yourself
- **`controller_experiment_1`**: its config is in the `quintic_walk` package (not included). If it
  claims the same leg joints as `quintic_walk`, one of the two will fail to activate.
- **`urdf/yantra_compose.urdf`** is an old generated file. The launch uses the `.xacro`, so it is unused and out of date.
- The world shows as **default** in Gazebo, so `worlds/empty.sdf` may not be the world being loaded.
