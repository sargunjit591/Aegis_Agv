# Project Aegis — AGV · Context & Handoff Brief

> **To the next agent:** read this whole file before touching anything. It records
> what is *verified on hardware*, what is *written but untested*, and traps that
> already cost hours. Where this file and a code default disagree, **this file
> wins** — several code defaults are stale (§7). `PROGRESS.txt` has the full story.

---

## 0. Rules for you, the agent

1. **Never move the vehicle on your own.** Do not run anything that publishes to
   `/cmd_vel`, `/agv/gripper_cmd`, `/agv/brake`, or starts a mission without the user's
   explicit go-ahead in the current turn. Syntax checks, mock runs, `--dry-run`,
   reading topics, and `--help` are fine. A wrong sign on this machine drives into
   something; the steering has no end stops.
2. **Safety code is never YAGNI.** Watchdogs, slew limits, direction interlock, brake
   pulse limit, steering limits, look-ahead, max-run backstop, runaway guard, stale-data
   checks and E-stop paths are load-bearing. No skill, refactor or "simplification" may
   remove or weaken them. If one looks redundant, ask.
3. **Rewrite, don't stack patches.** Repeated patching already produced duplicate
   methods and duplicate `declare_parameter` calls (ROS2 raises on the second one). For
   structural changes, rewrite the file clean, then confirm every symbol and parameter
   is defined exactly once.
4. **Verify before claiming done.** `python3 -m py_compile`, a mock-mode run where the
   code supports it, and for logic changes a small simulated test. Say plainly what
   still needs a hardware test — never imply hardware behaviour you couldn't observe.
5. **Keep the bridge and console pin maps identical.** Check both after any change.
6. **Restate pin numbers back** before coding against a wiring change.

---

## 1. Who you are working with

- **Sargunjit** — AI/ML + embedded engineer: ROS2, ArduPilot/MAVLink, Hailo, PCB design.
  Don't explain basics.
- Terse and technical. Wants direct answers **with explicit reasoning**. Pushes back on
  redundancy and hedging.
- Wants **complete runnable files**, not diffs.
- Wants **exact commands per terminal, from a fresh boot** for any test.
- Hardware: cost is not a constraint; prefers reliable parts.
- Works from a Windows laptop over SSH into the Pi.
- Writes "break" for **brake**; "turn 45°" means **vehicle heading**, not wheel angle.

---

## 2. Core objective

**Project Aegis** — GPS-denied autonomous navigation, two phases:

1. **Phase 1 (now) — warehouse AGV.** Navigate among oil drums, pick one up with the
   gripper, carry it to a set location, put it down. Localisation by 2D lidar SLAM.
2. **Phase 2 (later) — drone interceptor.** Port the validated stack to a drone
   (LightWare GRF250 altimeter, OAK-D VIO, terrain matching).

Immediate goal: **real odometry + closed-loop motion**, so Nav2 can drive the AGV.

---

## 3. Repository layout

```
aegis_agv/
├── CLAUDE.md                 this file — auto-loaded by Claude Code
├── PROGRESS.txt              plain-English history, current state, next steps
├── .gitignore
├── ubuntu_ros2/              Ubuntu 24.04 + ROS2 Jazzy — the robot
│   ├── agv_motor_bridge.py     /cmd_vel → motors, brake, gripper; /agv/status
│   ├── agv_imu_node.py         BNO085 → /agv/imu
│   ├── agv_drive_turn.py       mission: drive N m, turn N° of heading
│   ├── agv_obstacle_guard.py   demo: drive, stop for obstacle front or rear
│   └── hall_odometry.py        BLDC hall wheel-odometry tool (untested)
├── rpi_os_console/           Raspberry Pi OS — the web console
│   ├── agv_console_angle.py    Flask operator console, port 8000
│   └── agv-console.service     systemd unit (starts the console on boot)
├── calibration/
│   ├── as5600_calibrate.py     encoder:wheel ratio by hand-turning (result: 1:1)
│   └── as5600_test.py          live angle readout (use only its raw: field)
├── config/
│   └── box_slam_params.yaml    slam_toolbox params (on the Pi: ~/aegis_ws/config/)
├── docs/
│   └── aegis_slam_boxtest_guide.md   original SLAM bring-up guide (partly superseded)
└── archive/                  superseded — read for reference, don't run
    ├── agv_straight_stop.py    single-cone stop demo → replaced by obstacle_guard
    └── bno085_node.py          old IMU node: publishes Float32 /agv/heading and a
                                proper sensor_msgs/Imu on /imu/data. Superseded by
                                agv_imu_node.py, BUT its /imu/data publisher is what
                                the EKF step needs — port it. Never run both: they
                                fight over /dev/ttyAMA0.
```

On the Pi, mirror this tree at `~/aegis_agv/`. The systemd unit expects that path.

---

## 4. Vehicle & hardware — verified

Front-steer **tricycle** (bicycle model). Wheelbase ≈ 0.9 m, track ≈ 0.8 m, 13" wheels.
**Cannot rotate in place.** **Steered wheel has NO end stops** — spins freely.

| Component | Detail |
|---|---|
| Compute | Raspberry Pi 5 8 GB, NVMe; AI HAT+ (Hailo, unused so far) |
| Drive | BLDC hub motor + CY Gold 48 V/60 A controller (5-wire hall harness — never break it) |
| Throttle | Waveshare AD/DA HAT, DAC8532 ch A, SPI |
| Direction / brake | 4-ch active-LOW relay module, IN1 / IN2 |
| Brake | 24 V **self-opening** actuator — relay shorts its wires to apply |
| Steering | 24 V wiper motor via BTS7960 #2 |
| Gripper | linear actuator via BTS7960 #1 |
| Steering angle | AS5600, **1:1 on the pivot** (ratio 1.002 measured) |
| Heading | **BNO085**, UART-RVC |
| Lidar | RPLiDAR C1, **mounted rotated 180°** |
| Camera | OAK-D Lite — works, not used in current missions |
| On hand | MPU6050, LightWare GRF250 |

### Pin map — both code files must match

| Function | Pin | Notes |
|---|---|---|
| Steering RPWM (right) | GPIO19 | active **HIGH** |
| Steering LPWM (left) | GPIO16 | active **HIGH** |
| Gripper RPWM (**open**) | GPIO13 | active **LOW** |
| Gripper LPWM (**close**) | GPIO12 | active **LOW** |
| Direction relay IN1 | GPIO5 | **LOW = forward** |
| Brake relay IN2 | GPIO6 | **LOW = applied** — polarity unverified |
| DAC chip select | GPIO23 | SPI0 dev1, mode 1, 20 MHz |
| AS5600 | I2C1 0x36 | GPIO2/3 |
| BNO085 | UART0 RX GPIO15 | `/dev/ttyAMA0`, 115200 |
| Halls A/B/C (planned) | GPIO17/27/22 | via BSS138 level shifter |
| Free | GPIO4, 18, 20, 21, 24, 25, 26 | |

BTS7960 R_EN/L_EN/VCC → 3.3 V. Star ground (logic and power rails meet at one point).

### Measured constants

| Quantity | Value |
|---|---|
| Throttle idle / max | 0.80 V / 4.20 V |
| Forward drive (start threshold) | **1.50 V** |
| Reverse drive (start threshold) | **1.28 V** |
| Steering rate | ~100 °/s |
| Coast before look-ahead | left 15°, right 6.3° |
| AS5600 centre | **raw 3791** — re-home if startup angle ≠ ~0 |
| AS5600 sign | **`as5600_ccw_positive = false`** (left must read positive) |
| Lidar forward bearing | **−180°** in the raw scan |
| BNO085 resting forward accel | +2.95 m/s² → mounted ~17° off level |

---

## 5. Two operating systems — critical

| | Ubuntu 24.04 (`sargunjit-desktop`) | Raspberry Pi OS (`raspberrypi`) |
|---|---|---|
| Runs | ROS2, bridge, SLAM, IMU, missions | web console only |
| Header gpiochip | **gpiochip4** (`pinctrl-rp1`) | **gpiochip0** |

**Opening the wrong gpiochip succeeds silently and no pin moves.** Auto-detect by label in
any GPIO code. The bridge does; the console still hardcodes chip 0.

Ubuntu setup already done: user in `dialout` + `i2c`; udev rules for gpiochip/spidev/i2c;
UART0 enabled in `/boot/firmware/config.txt`, serial console removed (`/dev/ttyAMA10` is
the debug header, not ours); `~/aegis_ws` holds `sllidar_ros2` built from source.

---

## 6. Current state

**Verified ✅** — lidar SLAM mapping; all motors from ROS2; pulsed brake; AS5600 steering
with limits and look-ahead; BNO085 heading; obstacle demos; drive-then-turn with correct
distance and correct turn angle.

**Written, untested ⚠️** — the straighten fix in `agv_drive_turn.py` (stop on zero-crossing
or inside ±6°; bridge status raised 5→20 Hz). The previous run swung the wheel through
centre to the far limit. **Test this first.** Also `hall_odometry.py` (halls not wired).

**Unusable ❌** — IMU distance (drifts as t²; never use it). Lidar yaw during rotation.
Lidar distance in large spaces (pose drift) — wheel odometry is the fix.

### Topics
| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | Twist | `linear.x` sign = direction (fixed-speed mode); `+angular.z` = left |
| `/agv/status` | String JSON | from bridge, 20 Hz |
| `/agv/estop` | Bool | |
| `/agv/gripper_cmd` | String | `open` / `close` / `stop` |
| `/agv/brake` | String | `apply` / `release` / `auto` |
| `/agv/steer_home` | String | `here` — current wheel position becomes 0° |
| `/agv/imu` | String JSON | from IMU node, ~45 Hz |
| `/agv/imu_zero` | String | `zero` |
| `/scan` | LaserScan | |

---

## 7. Decisions & stale defaults

**Sensor split:** distance = lidar SLAM (→ wheel encoders) · heading = BNO085 ·
wheel angle = AS5600 · obstacles = raw `/scan` cones.

- No magnetometer, ever (steel drums, onboard motors).
- BNO085 on UART-RVC, never I2C (clock stretching breaks on the Pi).
- Lidar driver: `sllidar_ros2` from source; apt `rplidar_ros` can't run the C1.
- TF `base_link → laser` needs `--yaw 3.14159`. Raw-scan code uses `--forward-bearing -180`.
- Drive is fixed-speed; `fixed_speed:=false` restores proportional (Nav2 will want it).
- Brake is pulsed, never held, ≤2 s per engagement.
- Halls: BSS138 module or 74LVC245. Buck converters **cannot** level-shift a signal.
  Tap in parallel; never cut the harness; never power anything from the hall 5 V.
- If hall edge rate > ~2 kHz or misses > 1 %, move counting to a Pico/ESP32 over serial.

**Stale defaults.** `agv_motor_bridge.py` defaults don't match hardware. Until fixed,
always launch it with:
```
-p as5600_unwrap:=false -p steer_gear_ratio:=1.0 -p as5600_center_raw:=3791
-p steer_max_deg:=45.0 -p as5600_ccw_positive:=false
```
`agv_console_angle.py` still has `AS5600_CENTER_RAW = 3746`, `AS5600_CCW_POSITIVE = True`,
no rate look-ahead, and `gpiochip_open(0)` hardcoded.

**Safety:** wheels off the ground for first tests of changed motor code; `--dry-run` first;
never run console and bridge together; if the angle jumps ±170 → ∓170 the wheel crossed the
single-turn seam — stop and re-home.

---

## 8. Next steps — in order

1. **Verify the straighten fix** (launch sequence §10). Pass = wheel ends within a few
   degrees of 0. If not: bridge `closed_loop_steering:=true steer_deadband_deg:=4.0`.
2. **Bake verified values into defaults** — bridge and console: `as5600_unwrap` False,
   centre 3791, `ccw_positive` False. Port gpiochip auto-detect and the look-ahead into
   the console. Clean rewrite, then the long launch flags go away.
3. **Hall odometry** once the BSS138 module arrives: halls → GPIO17/27/22,
   `hall_odometry.py --check` (all 6 states), then `--calibrate --distance 5.0`.
4. **Wheel odometry node:** hall distance + AS5600 angle → bicycle model
   (`Δθ = d·tan(δ)/L`, L = 0.9 m) → `/odom` + `odom → base_link` TF, retiring the static
   identity. Restore slam_toolbox `minimum_travel_distance` ≈ 0.2 / heading ≈ 0.17.
5. **EKF:** `robot_localization` fusing `/odom` + IMU. Port `/imu/data` from
   `archive/bno085_node.py` into `agv_imu_node.py`.
6. **Nav2 for Ackermann:** Smac Hybrid-A* + Regulated Pure Pursuit, min turning radius
   ~1.3 m (35° lock). Diff-drive defaults will plan impossible motions.
7. **Drums:** YOLO on Hailo, ArUco for identity, OAK-D depth for approach, pick/place
   state machine.

Housekeeping: shim the IMU level; udev rules for stable serial names; keep this under git.

---

## 9. Skills — how to use them in this repo

| Skill | Use it for | Rules here |
|---|---|---|
| **graphify** | `/graphify .` once at session start to map code + docs; query the graph instead of re-reading files. `--update` after edits. | Treat INFERRED/AMBIGUOUS edges as hints — confirm with grep before acting. |
| **caveman** | Terse replies (matches the user's style). | Never compress away safety warnings, wiring instructions, or per-terminal test steps. |
| **ponytail** | Keep new code minimal. | Use **`/ponytail lite`** for anything touching motors, brake, steering or E-stop. Safety code is exempt from YAGNI (rule 0.2). |
| **arena** | Hard design problems with a measurable winner: odometry-node math, EKF config, Nav2 Ackermann params. | Only for things verifiable in software (mock tests). Not for hardware debugging — sub-agents can't touch the AGV. Keep the agent count modest; it's expensive. |
| **superpowers** (recommended) | `systematic-debugging` for any hardware fault; `verification-before-completion` before any "fixed". | TDD fits pure-logic modules (unwrap, hall decode, bicycle model) with mocks. Hardware behaviour is still verified by the user on stands. |

---

## 10. Launch sequence (Ubuntu, fresh boot)

Front wheel straight before starting the bridge. AGV still for the first 3 s of the IMU.

```bash
cd ~/aegis_agv/ubuntu_ros2

# 1  odom -> base_link (stopgap until wheel odometry)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id odom --child-frame-id base_link

# 2  base_link -> laser (180° yaw REQUIRED; set z to real lidar height)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.075 --roll 0 --pitch 0 --yaw 3.14159 --frame-id base_link --child-frame-id laser

# 3  lidar
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser

# 4  SLAM
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$HOME/aegis_ws/config/box_slam_params.yaml

# 5  motor bridge
python3 agv_motor_bridge.py --ros-args -p as5600_unwrap:=false -p steer_gear_ratio:=1.0 -p as5600_center_raw:=3791 -p steer_max_deg:=45.0 -p as5600_ccw_positive:=false

# 6  IMU (no --test flag — --test publishes nothing)
python3 agv_imu_node.py

# 7  mission
python3 agv_drive_turn.py --dry-run
python3 agv_drive_turn.py --distance 2 --turn-deg 45 --turn-dir right
```

Checks:
```bash
ros2 topic echo /agv/status --field data
ros2 topic hz /agv/imu
ros2 run tf2_ros tf2_echo map base_link
ros2 topic pub --once /agv/steer_home std_msgs/msg/String "{data: here}"
ros2 topic pub --once /agv/estop std_msgs/msg/Bool "{data: true}"
```

Known glitches: lidar error `80008004` at launch → replug its USB. OAK-D brownouts →
`usb_max_current_enable=1` and a real 5 V/5 A supply.
