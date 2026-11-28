# Fix plan: `ros2 launch command_center execute_motion.launch.py`

Status: **not runnable as committed**. The launch file raises `NameError` before any node
starts, and several downstream references are missing or inconsistent with the current
workspace. This document is an ordered, verifiable implementation plan.

Scope: the **real-robot** bringup path described in `README.md` §Usage ("Launch the System").
The Gazebo path (`quintic_walk yantra_gazebo.launch.py`) is out of scope; do not add a
standalone `ros2_control_node` to it (see `AI/CONTEXT.md` §5).

Primary files:

| File | Role |
|---|---|
| `src/a_command_center/launch/execute_motion.launch.py` | The launch under repair |
| `src/a_command_center/config/controller.yaml` | Controller definitions for `ros2_control_node` |
| `src/a_command_center/CMakeLists.txt` | Must install `config/` |
| `src/yantra_misc/yantra_moveit_config/launch/move_group.launch.py` | Hardcodes xacro mode mappings; defaults `use_sim_time:=true` |
| `src/yantra_motion/quintic_walk/launch/controller.launch.py` | Spawns `microcontroller_broadcaster` (undefined) |
| `configure.py` | Rewrites the above files; must be run before launch |

---

## Phase 0 — Baseline and backup

- [ ] 0.1 Confirm the failure mode (read-only):
  ```bash
  cd ~/Downloads/altair_lab
  export ROS_DOMAIN_ID=96
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  ros2 launch command_center execute_motion.launch.py --show-args
  ```
  Expected: `NameError: name 'os' is not defined`. This proves the launch description is
  never generated.

- [ ] 0.2 Back up the files that will be edited:
  ```bash
  cp src/a_command_center/launch/execute_motion.launch.py /tmp/execute_motion.launch.py.bak
  cp src/a_command_center/config/controller.yaml          /tmp/controller.yaml.bak
  cp src/a_command_center/CMakeLists.txt                  /tmp/CMakeLists.txt.bak
  cp src/yantra_misc/yantra_moveit_config/launch/move_group.launch.py /tmp/move_group.launch.py.bak
  ```

- [ ] 0.3 Record current git state so the diff is reviewable:
  ```bash
  git status --short
  git diff -- src/a_command_center src/yantra_misc/yantra_moveit_config > /tmp/pre_fix.diff
  ```

---

## Phase 1 — Make the launch file importable (blocker #1)

File: `src/a_command_center/launch/execute_motion.launch.py`

- [ ] 1.1 Add the missing `os` import. After line 10 (`from launch_ros.actions import ...`):
  ```python
  import os
  ```

- [ ] 1.2 (Optional, no behaviour change) Remove unused imports to keep the file clean:
  `DeclareLaunchArgument`, `RegisterEventHandler`, `OnProcessStart`, `LaunchConfiguration`.
  Keep `IncludeLaunchDescription`, `TimerAction`, `ExecuteProcess`, `GroupAction`,
  `PushRosNamespace`, `Node`, `PathJoinSubstitution`, `FindPackageShare`.

- [ ] 1.3 Re-run `--show-args`. It should now print the launch description instead of
  crashing. It will still not run correctly — continue.

---

## Phase 2 — Install the controller config (blocker #2)

File: `src/a_command_center/CMakeLists.txt` (install block at lines 57–67)

- [ ] 2.1 Add a `config` install rule next to the `launch` rule:
  ```cmake
  install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
  ```

- [ ] 2.2 Rebuild and confirm the file exists in the install tree:
  ```bash
  cd ~/Downloads/altair_lab
  export PATH=/usr/bin:$PATH          # avoid the Hermes Python venv (see AI/CONTEXT.md §3)
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select command_center --symlink-install
  source install/setup.bash
  ls install/command_center/share/command_center/config/controller.yaml
  ```

  Note: `install/command_center/` already exists but predates the `config/` rule, so the
  rebuild is mandatory. With `--symlink-install` the launch file will be symlinked to source.

---

## Phase 3 — Fix the rosbridge helper (blocker #3)

File: `execute_motion.launch.py`, lines 95–102. Two independent problems: the hardcoded
`~/altair_ws_fix` path does not exist, and `gnome-terminal` needs a GUI.

- [ ] 3.1 Remove the hardcoded source line. The launch already inherits the caller's
  environment (which has `install/setup.bash` sourced), so no `source` is needed:
  ```python
  auxiliary_node = ExecuteProcess(
      cmd=['gnome-terminal', '--', 'bash', '-c',
           'ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash'],
      output='screen'
  )
  ```

- [ ] 3.2 Guard the GUI dependency the same way `quintic_walk/launch/yantra_gazebo.launch.py`
  does: only create the terminal action when `DISPLAY` or `WAYLAND_DISPLAY` is set.
  On headless, either skip rosbridge or start it without a terminal:
  ```python
  has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
  ```
  Then include `auxiliary_node` only if `has_display`, or provide a headless fallback
  `ExecuteProcess(cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'])`.

- [ ] 3.3 Verify rosbridge is installed (it is, in `/opt/ros/jazzy`):
  ```bash
  ros2 pkg prefix rosbridge_server
  ```

---

## Phase 4 — Wire the robot description to the controller manager (blocker #4/#5)

This is the core functional fix. Today `control_node` receives only `controller.yaml`, so it
loads no `<ros2_control>` hardware and every spawner fails. The URDF is built inside
`move_group.launch.py`, not here.

### 4.0 Make `real_mode` actually true

`move_group.launch.py` lines ~56–63 hardcode the xacro mappings
(`real_mode/mock_mode/sim_mode = "false"`). They are only changed by `configure.py`.

- [ ] 4.0.1 Run the configurator before every launch (minimal-change path):
  ```bash
  cd ~/Downloads/altair_lab
  python3 configure.py --real
  ```
- [ ] 4.0.2 Confirm the mapping flipped:
  ```bash
  grep -n '"real_mode"\|"sim_mode"\|"mock_mode"' \
    src/yantra_misc/yantra_moveit_config/launch/move_group.launch.py
  ```
  Expected: `"real_mode": "true"`, the other two `"false"`.
- [ ] 4.0.3 Confirm the composed URDF contains the hardware plugins:
  ```bash
  xacro src/yantra_misc/yantra_description/urdf/yantra_compose.urdf.xacro \
    real_mode:=true sim_mode:=false mock_mode:=false > /tmp/yantra_real.urdf
  grep -n "yantra_hardware/DynamixelHardware\|yantra_hardware/MicrocontrollerHardware" /tmp/yantra_real.urdf
  ```

  Follow-up (not required for the first working run): refactor `move_group.launch.py` to
  declare `real_mode`, `mock_mode`, `sim_mode`, ports, etc. as launch arguments, and have
  `execute_motion.launch.py` pass them through. `configure.py` rewriting source files is
  fragile and leaves the tree dirty; treat it as debt.

### 4.1 Publish `robot_description` for the controller manager

Mirror the proven pattern in `head_controller/launch/yantra.launch.py` (lines 47–55): its
`ros2_control_node` subscribes via the remap
`("/controller_manager/robot_description", "/robot_description")`.

- [ ] 4.1.1 Add the same remapping to `control_node`:
  ```python
  control_node = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[ros2_controller_param],
      output="both",
      remappings=[
          ("/controller_manager/robot_description", "/robot_description"),
      ],
  )
  ```

- [ ] 4.1.2 Ensure something publishes `/robot_description`. Pass launch arguments to the
  MoveIt include so its `robot_state_publisher` runs (it is gated by
  `publish_robot_state`, default `false`):
  ```python
  move_group_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          PathJoinSubstitution([
              FindPackageShare('yantra_moveit_config'),
              'launch',
              'move_group.launch.py'
          ])
      ),
      launch_arguments={
          'use_sim_time': 'false',
          'publish_robot_state': 'true',
      }.items()
  )
  ```

- [ ] 4.1.3 Fix the namespace mismatch (blocker #9). `PushRosNamespace("alta")` makes the
  MoveIt RSP publish `/alta/robot_description`, while the remap above targets the absolute
  `/robot_description`. **Recommended: remove the namespace entirely** so topics match
  `AI/CONTEXT.md` (`/joint_states`, `/cmd_vel`, `/robot_description`):
  ```python
  robot_name = ""            # keep the variable for configure.py's regex
  ...
  return LaunchDescription([
      GroupAction([
          # PushRosNamespace(robot_name),   # removed: run in the global namespace
          auxiliary_node,
          ...
      ])
  ])
  ```
  If a namespace is genuinely required, use the same namespace on both sides instead
  (e.g. remap target `/alta/robot_description`, and adjust all verification commands).
  Also update `configure.py`'s `--robot-name` default from `"alta"` and the README example,
  otherwise the configurator will reintroduce the old name.

- [ ] 4.1.4 Fallback if the topic is not picked up on this Jazzy build: give `control_node`
  the description directly as a parameter, using the same xacro + mappings that
  `move_group.launch.py` uses:
  ```python
  robot_description = ParameterValue(
      Command([
          FindExecutable(name='xacro'), ' ',
          PathJoinSubstitution([FindPackageShare('yantra_description'), 'urdf',
                                'yantra_compose.urdf.xacro']),
          ' real_mode:=true sim_mode:=false mock_mode:=false',
      ]),
      value_type=str
  )
  ...
  parameters=[ros2_controller_param, {'robot_description': robot_description}],
  ```
  Do not use both approaches at once; prefer 4.1.1–4.1.2.

---

## Phase 5 — Fix controller definitions and spawners (blocker #6/#7)

- [ ] 5.1 `microcontroller_broadcaster` is spawned by
  `quintic_walk/launch/controller.launch.py` (lines 49–52) but is not declared in
  `src/a_command_center/config/controller.yaml`, and no hardware plugin in
  `yantra_hardware` provides its interfaces (it is commented out in every walk package's
  `config/controller.yaml`). Choose one:
  - **Option A (recommended):** remove the `microcontroller_broadcaster_spawner` from
    `quintic_walk/launch/controller.launch.py` (also affects sim, where it already fails
    harmlessly). Verify sim still reaches the expected controller set from
    `AI/CONTEXT.md` §4.
  - **Option B:** uncomment the `microcontroller_broadcaster` entry in
    `src/a_command_center/config/controller.yaml` and confirm a hardware component actually
    exports those interfaces. Do not do this speculatively.

- [ ] 5.2 Decide on `head_controller`:
  - It is declared in `controller.yaml` (lines 18–19, 27–29) but its launch is commented
    out in `execute_motion.launch.py` (line 113).
  - Do **not** simply uncomment it: `head_controller/launch/yantra.launch.py` starts its own
    `ros2_control_node` (line 47), which would create a second controller manager.
  - Either leave it disabled and remove/comment the entry in `controller.yaml`, or refactor
    `head_controller/launch/yantra.launch.py` to spawn only its controllers and include it
    after `control_node` is up.

- [ ] 5.3 Reconcile `update_rate`: `controller.yaml` uses 75 Hz while the hardware blocks
  declare `rw_rate="125"`. Pick one value or document the intentional difference; a
  mismatch is not fatal but makes timing behaviour harder to reason about.

---

## Phase 6 — Align the microcontroller serial port (blocker #8)

File: `execute_motion.launch.py`, lines 27–35. The node's parameters are `port` and
`baudrate` (`microcontroller_hardware.cpp` lines 42–44). Today they are hardcoded to
`/dev/ttyUSB0` / `115200`, while `configure.py` defaults the STM32 link to `/dev/stm32` /
`115200` and the xacro default is `/dev/ttyACM0`.

- [ ] 6.1 Make the port a launch argument with the udev-rule name as default:
  ```python
  stm32_port_arg = DeclareLaunchArgument('stm32_port', default_value='/dev/stm32')
  stm32_baud_arg = DeclareLaunchArgument('stm32_baudrate', default_value='115200')
  ...
  microcontroller_hardware_node = Node(
      package='microcontroller_hardware_node',
      executable='microcontroller_hardware_node',
      name='microcontroller_hardware_node',
      parameters=[{
          'port': LaunchConfiguration('stm32_port'),
          'baudrate': ParameterValue(LaunchConfiguration('stm32_baudrate'), value_type=int),
      }]
  )
  ```
  (Add `ParameterValue` to the `launch_ros.parameter_descriptions` imports.)

- [ ] 6.2 Confirm the udev rules from `README.md` step 5 are installed, and that
  `/dev/stm32` exists before launching:
  ```bash
  ls -l /dev/stm32 /dev/u2d2 2>/dev/null || echo "udev rules / device missing"
  ```

- [ ] 6.3 Make `configure.py --stm32-port` the single source of truth: either have the
  configurator rewrite these launch defaults (it already knows how to edit this file), or
  document that `--stm32-port` must be passed to the launch command too.

---

## Phase 7 — Startup ordering (blocker #10)

- [ ] 7.1 Keep the MoveIt-first ordering. `quintic_walk` needs the SRDF that `move_group`
  publishes (`AI/CONTEXT.md` §4). The current fixed `TimerAction(period=5.0)` is a race.
  Prefer spawners that wait for the service rather than a wall-clock delay:
  - Add `--controller-manager-timeout 60` to the spawner arguments in
    `quintic_walk/launch/controller.launch.py` (shared with sim; safe improvement), and/or
  - Raise the delay to 10–15 s and document why.

- [ ] 7.2 Ensure the two controller-manager-adjacent processes do not race:
  `control_node` must be up before the spawners. If keeping `TimerAction`, measure the
  actual time to `Configured and activated` and set the period above it.

---

## Phase 8 — Verification

Run from the workspace root, with `ROS_DOMAIN_ID=96` in every terminal.

- [ ] 8.1 Static checks:
  ```bash
  export ROS_DOMAIN_ID=96
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  ros2 launch command_center execute_motion.launch.py --show-args
  python3 -m py_compile src/a_command_center/launch/execute_motion.launch.py
  ```
- [ ] 8.2 Launch and confirm no process exits early:
  ```bash
  ros2 launch command_center execute_motion.launch.py
  ```
  Expected in the log: `move_group` reaches `You can start planning now!`, `ros2_control_node`
  starts without "parameter 'robot_description' not set", and each spawner reports
  `Configured and activated`.
- [ ] 8.3 Runtime checks (separate terminal):
  ```bash
  ros2 control list_controllers          # joint_state_broadcaster, imu_sensor_broadcaster, quintic_walk
  ros2 control list_hardware_components  # Dynamixel (+ Microcontroller if enabled), state=active
  ros2 topic echo /joint_states --once
  ros2 topic echo /robot_description --once | head -c 200
  ros2 param get /controller_manager use_sim_time   # must be false
  ```
- [ ] 8.4 Walking smoke test (slow, continuous publish):
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
- [ ] 8.5 Shutdown cleanly; if a process survives, `pkill -9 -f "gz sim"; pkill -9 -f ruby;
  ros2 daemon stop` (per `AI/context2.md`).

---

## Risks and rollback

- **Risk:** editing `quintic_walk/launch/controller.launch.py` also changes the Gazebo path.
  Mitigation: run the Gazebo smoke test from `AI/CONTEXT.md` §4 after the edit.
- **Risk:** `configure.py` rewrites source files and can silently flip modes. Mitigation:
  check `git diff` after every run; never commit its output blindly.
- **Risk:** enabling `publish_robot_state:=true` starts a second `robot_state_publisher` in
  sim. This plan is real-mode only; keep sim using its existing launch.
- **Rollback:** restore the `/tmp/*.bak` copies and rebuild:
  ```bash
  cp /tmp/execute_motion.launch.py.bak src/a_command_center/launch/execute_motion.launch.py
  cp /tmp/controller.yaml.bak          src/a_command_center/config/controller.yaml
  cp /tmp/CMakeLists.txt.bak           src/a_command_center/CMakeLists.txt
  cp /tmp/move_group.launch.py.bak     src/yantra_misc/yantra_moveit_config/launch/move_group.launch.py
  colcon build --packages-select command_center --symlink-install
  ```

---

## Suggested commit sequence

1. `command_center: add missing os import and install config/` (Phases 1–2)
2. `command_center: fix rosbridge helper path and headless guard` (Phase 3)
3. `command_center: wire robot_description and real-mode args` (Phase 4)
4. `walk: drop stale microcontroller_broadcaster spawner` (Phase 5)
5. `command_center: parametrize stm32 port, harden startup ordering` (Phases 6–7)

## Open decisions to confirm before editing

- Keep or remove the `/alta` namespace (recommended: remove).
- Disable `head_controller` cleanly, or refactor its launch to reuse `control_node`.
- Remove the `microcontroller_broadcaster` spawner, or implement/declare the controller.
- Whether to keep the `configure.py` rewrite workflow or convert mode flags to launch
  arguments (recommended follow-up, not required for the first working run).
