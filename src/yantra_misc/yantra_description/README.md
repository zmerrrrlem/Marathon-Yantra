# Yantra robot: "disintegrated in Gazebo" diagnosis

Instructions for an AI coding agent (or a person) working on this machine.

## Problem

When the `yantra` robot is spawned in Gazebo Sim (gz-sim 8), it appears broken apart.
Its links are scattered on the ground, one link has been thrown far away from the rest,
and the real-time factor is low (~39%). Your job is to find the cause in the robot
description and fix it.

## Where things are

| What | Path |
|---|---|
| Workspace root | `~/Downloads/altair_lab` |
| Description package | `~/Downloads/altair_lab/src/yantra_misc/yantra_description` |
| Robot model (URDF/xacro) | `yantra_description/urdf/` |
| Meshes | `yantra_description/meshes/` |
| Gazebo extras / plugins | `yantra_description/gazebo/`, `yantra_description/ros2_control/` |
| Controller config | `yantra_description/config/` |
| Worlds | `yantra_description/worlds/` |
| Launch files | `yantra_description/launch/` |
| Diagnostic script | `check_robot.py` (usually `~/Downloads/check_robot.py`) |

## Step 1: Find the main robot file

```bash
cd ~/Downloads/altair_lab/src/yantra_misc/yantra_description
ls urdf
grep -rl "<robot" urdf          # files that define a robot
grep -rn "xacro:include" urdf   # included pieces, not the main file
```

The main file is the one that defines `<robot name=...>` and includes the others.
Also check `launch/` to see which file the launch actually loads:

```bash
grep -rn "urdf\|xacro\|sdf" launch
```

## Step 2: Run the diagnostic script

```bash
source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || source /opt/ros/*/setup.bash
cd ~/Downloads/altair_lab/src/yantra_misc/yantra_description

python3 ~/Downloads/check_robot.py urdf/<MAIN_FILE> \
        --pkg-path ~/Downloads/altair_lab/src \
        --world worlds/<WORLD_FILE>.sdf \
        --gz
```

- Leave out `--world` if there is no world file. Leave out `--gz` if the `gz` command is missing.
- If the script says `xacro` was not found: `sudo apt install ros-$ROS_DISTRO-xacro`,
  or expand the file manually with `xacro urdf/<MAIN_FILE> > /tmp/yantra.urdf` and check that file.
- Exit code 1 means at least one ERROR was found.

The script uses only the Python standard library and does not modify any files.

## Step 3: Interpret the output

Results are grouped into numbered sections. Fix every ERROR first, then the WARNs.

| Section | Finding | Why the robot falls apart | Fix |
|---|---|---|---|
| 0. gz sdf conversion | Fewer links/joints after conversion than in section 1 | Converter dropped links/joints, so those parts are free bodies | Fix the cause reported in sections 1 and 3 (usually missing mass) |
| 1. Joints | Parent/child link does not exist | Joint is ignored, so the child link falls freely | Fix the link name typo in the joint |
| 1. Joints | More than one root link | Each extra root is an unattached body | Add the missing joint connecting it to the tree |
| 1. Joints | Link has two parent joints | Invalid tree, so a joint is dropped | Remove or rename the duplicate joint |
| 1. Joints | Child of non-fixed joint has no mass | URDF→SDF converter removes the link and its joint | Add an `<inertial>` with realistic mass and inertia |
| 2. Meshes | Mesh is many metres across | Mesh is in millimetres, so it is 1000× too big and collides with everything | Add `scale="0.001 0.001 0.001"` to the `<mesh>` (visual and collision) |
| 2. Meshes | Visual origin far from link frame | Part is drawn away from where it actually is | Fix `<visual><origin xyz=...>` (CAD exporters often bake in assembly offsets) |
| 2. Meshes | Cannot find mesh | Part invisible, or a broken path | Fix the `package://` path, or set `GZ_SIM_RESOURCE_PATH` |
| 3. Inertia | Mass ≤ 0, tiny mass or tiny inertia | Solver becomes unstable and explodes | Use realistic values (take them from CAD, or compute them from primitive shapes) |
| 3. Inertia | Triangle inequality violated / not positive definite | Physically impossible body, so the solver explodes | Recompute the inertia tensor |
| 3. Inertia | Inertia implies a huge radius | Probably g/mm units in the inertia values | Convert to kg and m² |
| 4. Spawn | Model z ≤ 0 | Starts inside the ground and gets launched | Spawn higher (e.g. `-z 0.3` in `ros_gz_sim create`) |
| 5. Collisions | Collision mesh has many triangles | Unstable contacts and low real-time factor | Replace with boxes, cylinders or a simplified hull |
| 6. Controllers | Very high PID gains | Controller tears the joints apart | Lower the gains in `config/` / `ros2_control/`, or test with controllers disabled |
| 6. Physics | Large `max_step_size` | Integration is unstable | Set `<max_step_size>0.001</max_step_size>` |

Inertia for common shapes (mass `m`, SI units):
- Box `x,y,z`: `ixx = m(y²+z²)/12`, `iyy = m(x²+z²)/12`, `izz = m(x²+y²)/12`
- Cylinder radius `r`, length `h` (along z): `ixx = iyy = m(3r²+h²)/12`, `izz = m r²/2`
- Sphere radius `r`: `ixx = iyy = izz = 2 m r²/5`

## Step 4: Checks the script cannot do

These can only be done with Gazebo running:

1. **Paused vs. playing test.** Launch the simulation, press reset, and keep it **paused**.
   - Robot already in pieces while paused: it's a geometry problem (mesh origins or scale, or a missing joint).
   - Robot assembled while paused but breaks on play: it's a physics problem (inertia, collisions, spawn, controllers).
2. **Visual debugging.** In the Gazebo GUI, right-click `yantra` → View →
   Joints / Collisions / Inertia / Frames. Look for missing joints, oversized or
   overlapping collision shapes, and missing or odd inertia boxes.
3. **Verbose log.**
   ```bash
   gz sim -v 4 worlds/<WORLD_FILE>.sdf 2>&1 | grep -iE "error|warn|inertia|joint|link"
   ```
4. **Isolate controllers.** Temporarily comment out the `gz_ros2_control` / controller
   plugin and spawn again. If the robot stays together, the gains in `config/` are the cause.

## Step 5: Fix and verify

1. Make one category of fix at a time. Keep a backup first (e.g. `git commit` or copy `urdf/`).
2. Re-run `check_robot.py` until it reports 0 errors.
3. Rebuild if the launch uses the installed package:
   ```bash
   cd ~/Downloads/altair_lab && colcon build --packages-select yantra_description
   source install/setup.bash
   ```
4. Relaunch Gazebo and confirm that the robot is assembled while paused and stays assembled on play.

## What to report back

- Main file and world used
- Full `check_robot.py` output
- Result of the paused vs. playing test
- Each change made (file, line, old value → new value) and why
- Whether the robot is stable after the fixes
