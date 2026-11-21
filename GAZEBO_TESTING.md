# Gazebo Simulation Testing Guide — Yantravira IK Fixes

This document provides step-by-step instructions for testing the IK fixes and walking functionality of the Yantravira humanoid robot in Gazebo simulation.

## Prerequisites

- ROS2 Jazzy Jalisco installed and sourced
- Workspace built: `colcon build --packages-select cubic_walk yantravira_urdf1 --executor sequential`
- Workspace sourced: `source install/setup.bash`
- `ROS_DOMAIN_ID=96` set (as per project conventions)

---

## Step 1: Launch Gazebo Simulation

### 1.1 Start the Simulation

```bash
# Source the workspace
source install/setup.bash

# Launch Gazebo with cubic_walk walking engine
ros2 launch cubic_walk cubic_walk_integrated.launch.py world:=empty.sdf
```

### 1.2 Verify Launch Success

Expected log output (look for these messages):
```
[cubic_walk]: Robot model loaded.
[spawner_cubic_walk]: Configured and activated cubic_walk
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[parameter_bridge-5]: Creating GZ->ROS Bridge: [/imu ...]
```

**Success criteria:**
- `cubic_walk` controller activates without errors
- `joint_state_broadcaster` activates without errors
- IMU bridge creates `/imu` topic
- Zero `package://` errors
- Zero `collision geometry` warnings

**Common failures:**
- `Package does not exist` → URDF `package://` URI case mismatch
- `IK failed` → Walk engine commanding unreachable foot poses
- `Robot model not loaded` → URDF/SRDF mismatch

---

## Step 2: Reset Robot to Default Pose

### 2.1 Call the Reset Pose Service

```bash
ros2 service call /cubic_walk/reset_pose std_srvs/srv/Trigger
```

Expected response:
```
response:
std_srvs.srv.Trigger_Response(success=True, message='Robot reset to default standing pose')
```

### 2.2 Verify Reset

In Gazebo GUI, the robot should be standing upright with:
- Both feet on the ground
- Legs in default standing pose
- Arms at sides
- Trunk upright

In ROS2 topics:
```bash
timeout 3 ros2 topic echo /is_falling --once
```
Expected: `data: False`

---

## Step 3: Test Walking Forward

### 3.1 Publish Walk Command

```bash
# Walk forward at 0.05 m/s
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Keep this running for at least 10 seconds.

### 3.2 Monitor Odometry

In a separate terminal:
```bash
source install/setup.bash
timeout 5 ros2 topic echo /walk_engine_odometry --once
```

Expected: Position x should increase over time (robot moving forward).

### 3.3 Monitor Walk Status

```bash
timeout 3 ros2 topic echo /altair_walk/walk_status --once
```

Expected values:
- `0` = IDLE
- `2` = START_MOVEMENT  
- `4` = WALKING

During active walking, expect `data: 4`.

### 3.4 Monitor Fall Detection

```bash
timeout 3 ros2 topic echo /is_falling --once
```

Expected: `data: False`

---

## Step 4: Test Turning

### 4.1 Request a 90° Left Turn

```bash
ros2 topic pub --once /altair_turn/request std_msgs/msg/Float64 "{data: 1.5708}"
```

### 4.2 Monitor Turn State

```bash
timeout 3 ros2 topic echo /altair_turn/state --once
```

Expected state transitions:
- `0` = IDLE → `1` = TURNING → `2` = SETTLING → `3` = DONE → `0` = IDLE

If abort occurs: `4` = ABORTED

### 4.3 Monitor Target Yaw

```bash
timeout 3 ros2 topic echo /target_yaw --once
```

---

## Step 5: Test Walk/Stop Transitions

### 5.1 Start Walking

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Wait 5 seconds.

### 5.2 Stop Walking

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 5.3 Verify Stop

Robot should come to a stable stop without falling:
```bash
timeout 3 ros2 topic echo /is_falling --once
```
Expected: `data: False`

---

## Step 6: Check IK Failure Count

After running the simulation for 30+ seconds with walking:

```bash
# Check launch log for IK failures
grep -c "IK failed" /tmp/opencode/gazebo_*.log
```

**Success criteria:** Zero (0) IK failures during normal walking.

**Acceptable:** Less than 5 IK failures per 30 seconds.

**Failure:** More than 10 IK failures per 30 seconds indicates the walk engine is commanding unreachable foot poses.

---

## Step 7: Test Marathon Mode (Optional)

### 7.1 Switch to Marathon Mode

```bash
ros2 topic pub --once /robot_mode std_msgs/msg/Int32 "{data: 0}"
```

### 7.2 Walk for Extended Period

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Run for 60+ seconds and monitor:
- `/walk_distance_m` — cumulative distance
- `/walk_distance_completed` — True when 400m reached
- `/is_falling` — should remain False

---

## Step 8: Full Test Script

Save this as `test_walking.sh`:

```bash
#!/bin/bash
set -e

echo "========================================="
echo "Yantravira Walking Test Suite"
echo "========================================="

source install/setup.bash

# Launch simulation
echo "[1/5] Launching Gazebo..."
ros2 launch cubic_walk cubic_walk_integrated.launch.py world:=empty.sdf &
LAUNCH_PID=$!
sleep 25

# Reset pose
echo "[2/5] Resetting robot pose..."
timeout 5 ros2 service call /cubic_walk/reset_pose std_srvs/srv/Trigger
sleep 2

# Walk forward test
echo "[3/5] Testing forward walk..."
timeout 12 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 &
sleep 10

# Check IK failures
echo "[4/5] Checking IK failures..."
IK_COUNT=$(grep -c "IK failed" /tmp/opencode/gazebo_ik_fix.log 2>/dev/null || echo 0)
echo "IK failures: $IK_COUNT"

# Check odometry
echo "[5/5] Checking odometry..."
timeout 3 ros2 topic echo /walk_engine_odometry --once 2>&1 | head -15

# Cleanup
kill $LAUNCH_PID 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null

echo "========================================="
echo "Test complete!"
echo "========================================="
```

Run with:
```bash
chmod +x test_walking.sh
./test_walking.sh
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| `Package does not exist` errors | URDF `package://` URI case mismatch | Fix URI: `Yantravira-urdf1` → `yantravira_urdf1` |
| `IK failed: left=0 right=0` | Foot poses outside reachable workspace | Reduce `max_step_x`, `max_step_y` in walking.yaml |
| Robot falls immediately | IMU not publishing / stabilizer not active | Check `/imu` topic, enable stability_stop |
| Walk status stays at 0 (IDLE) | `/cmd_vel` not received or walkable_state false | Check topic publishing, verify robot_state |
| Odometry not publishing | cubic_walk not active or walk_engine IDLE | Verify controller activation, send walk command |
| Robot twists ankles / split pose | Frame convention mismatch in IK | Verify trunk-to-root frame transform in walk_ik.cpp |

---

## Files Modified for IK Fixes

| File | Change |
|------|--------|
| `src/yantravira_urdf1/urdf/yantravira_urdf1.urdf` | Switched to Yantravira-fix model with corrected masses/inertias |
| `src/altair_motion/cubic_walk/config/walking.yaml` | Retuned step limits, freq, foot_rise for Yantra kinematics |
| `src/altair_motion/cubic_walk/src/walk_ik.cpp` | Added frame transform (trunk→root), pose clamping, orientation validation, standing pose fallback |
| `src/altair_motion/cubic_walk/include/cubic_walk/walk_ik.hpp` | Added `resetLegToStanding()` declaration |

---

## Expected Test Results

| Test | Expected Result |
|------|----------------|
| Launch | Zero errors, controllers activate |
| Reset pose | success=True, robot stands upright |
| Walk forward | Odometry x increases, walk_status=4 |
| Turn | /altair_turn/state transitions 0→1→2→3→0 |
| Walk/stop | Robot stops without falling |
| IK failures | 0 during normal walking |
| Fall detection | /is_falling = False throughout |

---

*Last updated: Sat Nov 21 2026*
