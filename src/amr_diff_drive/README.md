# amr_diff_drive

Differential drive controller for the AMR (ROS 2 Humble, Python / rclpy).

It does two jobs in one node (`diff_drive_controller`):

1. **Command path:** converts `/cmd_vel` (body velocity) into per-wheel speed commands on `/left_vel` and `/right_vel`.
2. **Odometry path:** turns wheel encoder feedback from `/encoder_telemetry` into `/odom` and the `odom → base_link` TF.

All outputs are published at **50 Hz** from a single timer.

---

## 1. Interfaces

### Subscribed topics

| Topic | Type | Units | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` m/s, `angular.z` rad/s | Other fields are ignored (a diff drive can't move sideways). QoS: reliable, depth 10. |
| `/encoder_telemetry` | `sensor_msgs/msg/JointState` | `position` rad, `velocity` rad/s | Wheels are looked up **by name** (`left_wheel`, `right_wheel`), not by array index. QoS: best-effort (sensor data), which works with both reliable and best-effort publishers. |

Expected encoder message format:

```yaml
header:
  stamp: {sec: 1790359259, nanosec: 594341871}
  frame_id: ''
name: [left_wheel, right_wheel]
position: [6698.438452980677, 7147.911794138329]   # rad, cumulative
velocity: [10.016096115112305, 9.962715148925781]   # rad/s
effort: []
```

### Published topics (all at `publish_rate`, default 50 Hz)

| Topic | Type | Units / content |
|---|---|---|
| `/left_vel` | `std_msgs/msg/Float64` | Left wheel command, **rad/s** |
| `/right_vel` | `std_msgs/msg/Float64` | Right wheel command, **rad/s** |
| `/odom` | `nav_msgs/msg/Odometry` | `frame_id: odom`, `child_frame_id: base_link`. The pose is in the odom frame; the twist (m/s, rad/s) is in base_link. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform `odom → base_link` |

The names of the four output topics are fixed in code. The input topic names and the frame names are parameters.

---

## 2. Robot configuration (hard-coded defaults)

| Value | Setting | Parameter |
|---|---|---|
| Wheel radius | **0.056 m** | `wheel_radius` |
| Wheel separation (wheel centre to wheel centre) | **0.39 m** | `wheel_separation` |
| Maximum wheel speed | **18.0 rad/s** (≈ 1.008 m/s at the rim) | `max_wheel_speed` |
| cmd_vel timeout | **0.5 s** | `cmd_vel_timeout` |
| Publish rate | **50 Hz** | `publish_rate` |
| Frames | `odom` → `base_link` | `odom_frame`, `base_frame` |
| Wheel direction | both wheels positive = forward | `invert_*` flags |

These defaults live in the node **and** in `config/diff_drive.yaml`. At launch, the YAML file overrides the node's defaults.

---

## 3. Package layout

```
final_hoja_pls/                     # colcon workspace root
└── src/amr_diff_drive/
    ├── amr_diff_drive/
    │   ├── diff_drive_controller.py   # ROS node
    │   └── kinematics.py              # pure math (no ROS), unit tested
    ├── config/diff_drive.yaml         # all parameters
    ├── launch/diff_drive.launch.py    # launch file
    ├── test/test_kinematics.py        # pytest unit tests
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

---

## 4. Installation and build

Prerequisites: Ubuntu 22.04, ROS 2 Humble at `/opt/ros/humble`, and `colcon`. The dependencies (`rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_ros`, `tf2_ros_py`, `launch`, `launch_ros`) are all part of a standard ROS 2 Humble desktop install. If anything is missing, run:

```bash
cd ~/Desktop/final_hoja_pls
rosdep install --from-paths src --ignore-src -y
```

Build:

```bash
cd ~/Desktop/final_hoja_pls
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`--symlink-install` means edits to the Python files and `diff_drive.yaml` take effect on the next launch without rebuilding. Without it, rebuild after every change.

---

## 5. Running

In every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/final_hoja_pls/install/setup.bash
```

### Start the controller (recommended way)

```bash
ros2 launch amr_diff_drive diff_drive.launch.py
```

With a custom parameter file:

```bash
ros2 launch amr_diff_drive diff_drive.launch.py params_file:=/absolute/path/to/my_params.yaml
```

### Start without the launch file

```bash
ros2 run amr_diff_drive diff_drive_controller --ros-args \
  --params-file ~/Desktop/final_hoja_pls/src/amr_diff_drive/config/diff_drive.yaml
```

Or override individual parameters:

```bash
ros2 run amr_diff_drive diff_drive_controller --ros-args \
  -p max_wheel_speed:=12.0 -p wheel_separation:=0.392
```

On startup the node logs a line like this:

```
diff_drive_controller up: r=0.056 m, L=0.39 m, max_wheel_speed=18.0 rad/s, rate=50.0 Hz, odom->base_link, cmd="/cmd_vel", encoders="/encoder_telemetry"
```

It logs another line when the first encoder message arrives:

```
First encoder sample: left=6698.4385 rad, right=7147.9118 rad
```

### Send commands

```bash
# Straight at 5 m/s (gets limited to 18 rad/s per wheel, see section 7)
ros2 topic pub -r 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Gentle arc: 0.2 m/s forward, 0.5 rad/s left
ros2 topic pub -r 50 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"

# Spin in place counter-clockwise
ros2 topic pub -r 50 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
```

Keyboard teleop also works: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

### Check the outputs

```bash
ros2 topic hz /left_vel          # ~50 Hz
ros2 topic hz /right_vel         # ~50 Hz
ros2 topic hz /odom              # ~50 Hz
ros2 topic hz /tf                # ~50 Hz
ros2 topic echo /left_vel
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_link
```

---

## 6. Parameters reference (`config/diff_drive.yaml`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `wheel_radius` | double | `0.056` | Wheel radius in m. Must be > 0. |
| `wheel_separation` | double | `0.39` | Distance between the wheel contact points in m. Must be > 0. |
| `max_wheel_speed` | double | `18.0` | Wheel speed limit in rad/s. If either wheel exceeds it, **both** are scaled by the same factor. `<= 0` turns the limit off. |
| `cmd_vel_timeout` | double | `0.5` | Seconds without `/cmd_vel` before both wheels are commanded to `0.0`. |
| `invert_left_cmd` | bool | `false` | Negate the left wheel command before publishing. |
| `invert_right_cmd` | bool | `false` | Negate the right wheel command before publishing. |
| `encoder_topic` | string | `/encoder_telemetry` | JointState input topic. |
| `left_joint_name` | string | `left_wheel` | Name of the left wheel in `JointState.name`. |
| `right_joint_name` | string | `right_wheel` | Name of the right wheel in `JointState.name`. |
| `invert_left_encoder` | bool | `false` | Negate the left encoder position and velocity. |
| `invert_right_encoder` | bool | `false` | Negate the right encoder position and velocity. |
| `encoder_timeout` | double | `0.5` | Seconds without encoder messages before the odom twist is reported as 0 and a warning is logged. The pose is kept. |
| `max_wheel_delta` | double | `10.0` | Largest wheel angle change (rad) accepted between two encoder messages. A larger jump is treated as an encoder reset or glitch: it is ignored and the reference point is reset (see section 8). |
| `cmd_vel_topic` | string | `/cmd_vel` | Twist input topic. |
| `publish_rate` | double | `50.0` | Hz for all outputs. Must be > 0. |
| `odom_frame` | string | `odom` | `header.frame_id` of odom and TF. |
| `base_frame` | string | `base_link` | `child_frame_id` of odom and TF. |
| `publish_tf` | bool | `true` | Whether to broadcast `odom → base_link`. Set it to `false` if something else (for example an EKF from `robot_localization`) publishes that transform. |
| `pose_covariance_diagonal` | double[6] | `[0.001, 0.001, 1e6, 1e6, 1e6, 0.01]` | Diagonal of `odom.pose.covariance` for (x, y, z, roll, pitch, yaw). The large values mark axes the robot can't measure. |
| `twist_covariance_diagonal` | double[6] | `[0.001, 0.001, 1e6, 1e6, 1e6, 0.01]` | Diagonal of `odom.twist.covariance`. |

You can inspect parameters at runtime with `ros2 param list /diff_drive_controller` and `ros2 param get /diff_drive_controller wheel_radius`. Parameters are **read once at startup**, so to change one, edit the YAML and restart the node.

---

## 7. How it works

Conventions follow REP-103: +x is forward, +y is left, and positive yaw is counter-clockwise. A positive wheel speed drives the robot forward.

Symbols: `r` = wheel_radius, `L` = wheel_separation.

### 7.1 cmd_vel → wheel speeds (inverse kinematics)

```
ω_left  = (v − ω · L/2) / r
ω_right = (v + ω · L/2) / r
```

**Speed limit.** When `max(|ω_left|, |ω_right|) > max_wheel_speed`, both wheels are multiplied by `max_wheel_speed / max(|ω_left|, |ω_right|)`. The robot then drives the **same curve**, just slower. Clamping each wheel separately would change the direction of travel.

Worked examples (r = 0.056, L = 0.39, limit 18 rad/s):

| cmd_vel (v, ω) | Raw wheels (rad/s) | Published (rad/s) | Actual robot motion |
|---|---|---|---|
| (5.0, 0.0) | 89.29 / 89.29 | **18.0 / 18.0** | 1.008 m/s straight |
| (0.5, 0.0) | 8.93 / 8.93 | 8.93 / 8.93 | 0.5 m/s straight |
| (0.0, 1.0) | −3.482 / +3.482 | −3.482 / +3.482 | spin CCW at 1 rad/s |
| (0.2, 0.5) | 1.830 / 5.312 | 1.830 / 5.312 | arc with 0.4 m radius |

**Limits of the robot:** top straight speed = `18 × 0.056` = **1.008 m/s**. Top spin rate = `2 × 18 × 0.056 / 0.39` ≈ **5.17 rad/s**.

**Timeout.** If no `/cmd_vel` arrives within `cmd_vel_timeout` (0.5 s), or none has arrived since startup, `0.0` is published to both wheels. `0.0` is also sent once when the node shuts down (Ctrl-C). Non-finite (NaN/inf) commands are ignored.

### 7.2 Encoders → odometry

- **First message:** the node records the wheel positions as the starting reference. The odom pose starts at `(0, 0, 0)` no matter how large the absolute encoder values are.
- **Every following message:** the position changes `ΔL`, `ΔR` (rad) are computed, and then:
  ```
  Δs = r (ΔR + ΔL) / 2          # distance travelled (m)
  Δθ = r (ΔR − ΔL) / L          # heading change (rad)
  ```
  The pose is advanced by **exact arc integration**: the robot is assumed to move along a circle during the interval. If `|Δθ| < 1e-6`, a straight-line midpoint formula is used instead to avoid dividing by zero. θ is kept in the range [−π, π].
- **Why positions and not velocities:** integrating position changes doesn't drift when messages arrive late, early or get dropped. The distance travelled is always the true wheel rotation.
- **Twist** in `/odom` comes from the encoder `velocity` field:
  `v = r (ω_R + ω_L) / 2`, `ω = r (ω_R − ω_L) / L`. If the velocity field is missing, the twist is 0.

### 7.3 Publishing (50 Hz timer)

Each tick publishes `/left_vel`, `/right_vel`, `/odom` and the TF, stamped with the **node's clock (`now`)**. The encoder header stamp isn't used, because the telemetry repeats the same stamp across messages with different data. The pose in `/odom` is the latest integrated pose, which is at most one encoder period old.

---

## 8. Safety and error handling

| Situation | Behaviour |
|---|---|
| `/cmd_vel` stops for > 0.5 s | Both wheel commands → `0.0` |
| Node shut down | `0.0` sent to both wheels once |
| Encoder message missing `left_wheel` or `right_wheel` | Message ignored; warning printed (at most every 2 s) listing the names received |
| Encoder position is NaN or inf | Message ignored; warning printed |
| Encoder jumps by more than `max_wheel_delta` (10 rad) in one message (for example a microcontroller restart that resets the position to 0) | Pose is **not** moved. The new position becomes the reference and a warning is logged. |
| No encoder messages for > 0.5 s | Odom twist reported as 0 and the last pose is kept. Warning: `Encoder telemetry stale`. `/odom` and TF keep publishing at 50 Hz. |
| No encoder messages since startup | `/odom` and TF publish pose `(0, 0, 0)` with zero twist |
| Invalid geometry (radius/separation ≤ 0), rate ≤ 0, or wrong covariance length | Node refuses to start (ValueError) |

At the 18 rad/s limit and 50 Hz, a normal change per encoder message is about 0.36 rad, so 10 rad leaves a lot of margin. Raise `max_wheel_delta` only if encoder messages can go missing for longer than about 0.5 s while the robot is moving.

---

## 9. First run on the real robot (checklist)

1. **Lift the wheels off the ground.**
2. Start the controller and check that the encoders are connected:
   `ros2 topic echo /encoder_telemetry --once` and look for the `First encoder sample` log line.
3. Send `{linear: {x: 0.1}}`. **Both wheels should spin forward.**
   - If one wheel spins backwards, set that side's `invert_<side>_cmd: true`.
4. While driving forward, **both encoder positions should increase**.
   - If one decreases, set `invert_<side>_encoder: true`.
5. Put the robot on the floor, drive forward slowly, and check that `/odom` `pose.position.x` **increases** and `y` stays about 0.
6. Send `{angular: {z: 0.5}}`. The robot should turn **left (counter-clockwise, seen from above)**, and the yaw in `/odom` should increase.
7. Stop publishing and confirm the wheels stop within 0.5 s.

---

## 10. Calibration

Nominal dimensions are never exact. Calibrate in this order:

**Wheel radius (straight line):**
1. Mark the start point and reset odom by restarting the node.
2. Drive straight a known distance `D_actual` (for example 3 m, measured with tape).
3. Read `D_odom` = `pose.position.x` from `/odom`.
4. Set `wheel_radius_new = wheel_radius × D_actual / D_odom`.

**Wheel separation (rotation):**
1. After calibrating the radius, restart the node.
2. Spin in place **exactly N full turns** (for example 5 turns = `θ_actual = 10π`), using a floor mark to count them.
3. Add up the total yaw change `θ_odom` from `/odom`, counting every wrap-around at ±π.
4. Set `wheel_separation_new = wheel_separation × θ_odom / θ_actual`.

Edit `config/diff_drive.yaml`, restart, and repeat until the error is under about 1%.

---

## 11. Tests

Unit tests cover the kinematics: inverse kinematics, the speed limit preserving the curve, forward/inverse round trips, straight-line integration, a closed circle returning to the origin, and an exact quarter arc.

```bash
cd ~/Desktop/final_hoja_pls/src/amr_diff_drive
python3 -m pytest -q test
# or through colcon:
cd ~/Desktop/final_hoja_pls && colcon test --packages-select amr_diff_drive && colcon test-result --verbose
```

Results from the live test against a simulated robot:

| Check | Result |
|---|---|
| `/left_vel`, `/right_vel`, `/odom`, `/tf` rates | 50.0 / 50.0 / 50.0 / 50.0 Hz |
| cmd_vel 5.0 m/s | wheels 18.0 rad/s, odom v = 1.008 m/s |
| cmd_vel (0.2, 0.5) | odom v = 0.2000, ω = 0.5000; yaw rate measured from the pose = 0.5000 rad/s |
| cmd_vel stopped | wheels 0.0 within 0.5 s |

---

## 12. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Wheels always 0 | `/cmd_vel` isn't reaching the node: check `ros2 topic info /cmd_vel -v`. Or it's published slower than 2 Hz, so the timeout triggers. |
| `/odom` stays at 0 while the robot moves | No encoder data: check `ros2 topic hz /encoder_telemetry`. Or the joint names don't match: look for the warning showing the names received, then set `left_joint_name` / `right_joint_name`. |
| Odom moves backwards or turns the wrong way | Encoder direction: set the `invert_*_encoder` flags (see section 9). |
| Robot goes slower than commanded | Expected above 1.008 m/s because of the 18 rad/s limit. Raise `max_wheel_speed` only if the motors can handle it. |
| `Encoder jump ignored` warnings during normal driving | Encoder messages are too far apart or the values are noisy. Raise `max_wheel_delta`. |
| `Encoder telemetry stale` warnings | Encoder publisher is slower than 2 Hz or stopped. |
| Two nodes publishing `odom → base_link` (TF flickers) | Set `publish_tf: false` if an EKF or another node owns that transform. |
| Odom drifts over distance or rotation | Calibrate (section 10). |
