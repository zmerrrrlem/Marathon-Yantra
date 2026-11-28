# yantra_heading

IMU heading hold and closed-loop relative turns for the walk engines. Ported from the
`old` branch, where the same job was split across `altair_walk/heading_hold.py`,
`altair_turn` (turn supervisor + cmd mux) **and** a second turn state machine inside
`cubic_walk`. Those competed for `/cmd_vel` and `/target_yaw`; here it is one node and
one state machine, and it works with any engine that takes `geometry_msgs/Twist`.

```
teleop / mission --/cmd_vel_raw--> heading_controller --/cmd_vel--> quintic_walk
                                          ^        ^
                     /imu_sensor_broadcaster/imu   /walk_status (published by quintic_walk)
```

## Run

```bash
ros2 launch yantra_heading heading.launch.py            # add use_sim_time:=true in Gazebo
```

Point whatever publishes walk commands (e.g. `yantra_teleop`, mission code) at
`/cmd_vel_raw` instead of `/cmd_vel`, for example with a remap:
`--ros-args -r cmd_vel:=/cmd_vel_raw`. If the node is not running, nothing changes:
the walk controller still listens on `/cmd_vel`.

## Use

| What | How |
|---|---|
| Walk straight, auto-corrected | Send forward speed with `angular.z = 0`. After `hold_start_delay` s of WALKING the current heading is latched and held. |
| Steer manually | Send non-zero `angular.z`: passed through untouched. When you stop steering, the new heading is latched after `relatch_delay`. |
| Turn by an angle | `ros2 topic pub --once /heading/turn_deg std_msgs/Float64 "{data: 90.0}"` (or `/heading/turn` in radians). Positive = counter-clockwise. After settling it keeps holding the new heading. |
| Cancel a turn | `ros2 topic pub --once /heading/cancel std_msgs/Empty` (a stop command, `angular.x != 0`, also cancels). |
| Hold a specific heading | `/heading/set_target` (absolute IMU yaw, radians) |
| Watch it | `/heading/state` (0 PASSTHROUGH, 1 HOLD, 2 TURNING, 3 SETTLING, 4 DONE, 5 ABORTED), `/heading/yaw`, `/heading/target_yaw`, `/heading/error` |

Safety behaviour: nothing is published before the first operator command (a zero Twist
would start the engine), the robot's stop command is always forwarded, and a turn aborts
on stale IMU (> `imu_timeout`), timeout, or if the engine does not start stepping within
`walk_start_grace`.

## Tuning order

1. **Direction**: request a +30° turn. If the robot turns the wrong way, set `yaw_sign: -1.0`.
2. **Turn speed**: raise `turn_max_rate` until turns are quick without stumbling; the walk
   controller still clamps to `max_step_angular`.
3. **Hold**: walk straight and watch `/heading/error` in PlotJuggler. Raise `hold_kp` until it
   starts to wiggle, back off ~30 %, then raise `hold_ki` until the steady offset disappears.
   `hold_max_rate` caps how hard it can steer.

All parameters are in `config/heading.yaml` and can be changed live with `ros2 param set`.

## Tests

`python3 -m pytest test/` runs closed-loop tests against a simulated walker (drift, gain
error, start-up delay, wrap-around at ±180°, stale IMU, stop/cancel). They need no ROS install.
