# Project Aegis — SLAM Bring-Up & Handheld Box Test Guide

RPLiDAR C1 + OAK-D Lite (FF) + Raspberry Pi 5 (8GB, AI HAT+) → slam_toolbox + Nav2

---

## 0. One decision before anything: the Pi's OS

Your motor test scripts (`lgpio` + `spidev`) run on Raspberry Pi OS Bookworm, but ROS2 Jazzy binary packages require **Ubuntu 24.04** — the same OS already on your Zephyrus laptop, which is exactly what you want (identical ROS2 versions on both machines, zero version friction).

**Recommended setup:** flash **Ubuntu 24.04 Server (64-bit)** for the Pi 5 onto your NVMe (or a second SD card) and keep your current Bookworm card untouched as the "motor test" fallback. Install ROS2 Jazzy from the standard apt repos exactly as you did on the laptop.

Your existing GPIO/DAC code carries over with one caveat: on some Ubuntu kernel builds for Pi 5, the 40-pin header lives on `gpiochip4`, not `gpiochip0`. If `lgpio.gpiochip_open(0)` throws or pins don't respond, change it to `gpiochip_open(4)`. Everything else (`spidev`, pin numbers, your whole `agv_control.py`) is unchanged.

Run the Pi **headless** (no desktop). RViz runs on your laptop over the network — the Pi never renders anything.

---

## 1. Hardware Interfacing

### RPLiDAR C1
- Connect via its included USB adapter into one of the **black USB 2.0 ports**. The C1 is a 460800-baud serial device — USB3 bandwidth is wasted on it, and keeping it on USB2 leaves the USB3 controller free for the camera.
- Power draw ~300 mA at 5V — fine from the port.
- It enumerates as `/dev/ttyUSB0` (CP210x). Give yourself permission once:
  ```bash
  sudo usermod -a -G dialout $USER   # then log out/in
  ```
- Mount it as the **highest thing on the box**, dead level. A tilted 2D lidar is the #1 cause of garbage maps — the scan plane must stay parallel to the floor. Nothing may block its 360° view (camera and cables mount *below* its scan plane).

### OAK-D Lite
- Connect to one of the **blue USB 3.0 ports** with the short shipped cable (or any shielded USB3 cable under 1 m — long unshielded cables cause the classic "device found, then disappears" DepthAI failure).
- Install the udev rule so it enumerates without root:
  ```bash
  echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

### Power — the Pi 5 USB current gotcha
The Pi 5 limits total USB output to **600 mA** unless it detects a 5V/5A supply. The OAK-D Lite alone can pull ~900 mA under load. Two options:
- On the bench: use the official 27W (5V/5A) PSU — limit rises to 1.6 A, enough for camera + lidar.
- On battery/UBEC (your 24V→5V BEC): add this to `/boot/firmware/config.txt`:
  ```
  usb_max_current_enable=1
  ```
  and make sure the BEC can genuinely deliver 5A. If you ever see the OAK-D disconnecting under load, that's a power symptom, not a software bug — a powered USB3 hub is the definitive fix.

### Networking during development
USB3 is a notorious 2.4 GHz WiFi jammer (it radiates right in that band). With the OAK-D active, use **5 GHz WiFi or Ethernet** between Pi and laptop, or your RViz stream and SSH will stutter and you'll chase phantom bugs.

---

## 2. Software & SLAM Stack

### The architecture (and why)

| Job | Package | Why this one |
|---|---|---|
| 2D SLAM (map + localization) | `slam_toolbox` | The modern default for warehouse-scale 2D lidar SLAM. Actively maintained, integrates natively with Nav2, has both mapping and localization modes. Cartographer is effectively unmaintained for ROS2; RTAB-Map is a full RGB-D SLAM that will eat the Pi alive and adds nothing for a flat warehouse. |
| Odometry (box test) | `rf2o_laser_odometry` | Estimates odometry purely from consecutive lidar scans. Necessary because the **OAK-D Lite has no IMU** — it cannot do VIO. On the final AGV this gets replaced/fused with your AS5600 wheel + steering encoders. |
| Camera driver | `depthai-ros` | Official Luxonis ROS2 driver — RGB, stereo depth, and on-device neural nets. |
| Navigation (next phase) | Nav2 | Path planning + following. Note for later: your AGV is a front-steer tricycle (Ackermann), so you'll use the **Smac Hybrid-A*** planner and **Regulated Pure Pursuit** controller — the default diff-drive assumptions do not fit your chassis. |
| Barrel detection (next phase) | YOLO on the AI HAT+ (Hailo) | You already run YOLO on Hailo professionally — same HailoRT toolchain, new class. Keeps the OAK-D free for depth. |

### TF tree you're building

```
map ──(slam_toolbox)──> odom ──(rf2o)──> base_link ──(static)──> laser
                                                   └─(static)──> oak
```

### Install (on the Pi, after ROS2 Jazzy itself)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rplidar-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-depthai-ros \
  ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-tools \
  python3-colcon-common-extensions

# rf2o — build from source (not reliably in Jazzy apt)
mkdir -p ~/aegis_ws/src && cd ~/aegis_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/aegis_ws && colcon build --symlink-install
echo "source ~/aegis_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Network setup (both Pi and laptop)

```bash
# add to ~/.bashrc on BOTH machines — must match
export ROS_DOMAIN_ID=42
```
Same LAN, and topics flow automatically. Verify from the laptop with `ros2 topic list` while the lidar node runs on the Pi.

### slam_toolbox config

Save as `~/aegis_ws/box_slam_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    resolution: 0.05          # 5 cm/cell — right for a warehouse
    max_laser_range: 12.0     # C1 spec
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.17   # ~10° — new scan every 10° of rotation
    do_loop_closing: true
    loop_search_maximum_distance: 5.0
    transform_publish_period: 0.05
```

---

## 3. The Box Test Plan

### Physical prep
1. Rigidly mount lidar (top, level) and OAK-D (front face, below the scan plane) to the box. **Rigid matters** — velcro flex between sensors corrupts the calibration between them.
2. Measure, with a ruler, the offset from a chosen reference point on the box (this is `base_link` — pick the box's center, at floor-projection) to the lidar center and camera lens. Note x (forward), y (left), z (up) in meters.

### Terminal session (use tmux on the Pi: `sudo apt install tmux`)

**Pane 1 — static transforms** (replace numbers with your measured offsets):
```bash
ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.25 \
  --frame-id base_link --child-frame-id laser &
ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0.0 --z 0.15 \
  --frame-id base_link --child-frame-id oak-d-base-frame &
```

**Pane 2 — lidar:**
```bash
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser
```
Sanity check from the laptop: `ros2 topic hz /scan` → expect ~10 Hz. If you get nothing, it's the serial port name or dialout group.

**Pane 3 — laser odometry:**
```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```
(Confirm it publishes `odom → base_link`; check with `ros2 run tf2_tools view_frames`.)

**Pane 4 — SLAM:**
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/aegis_ws/box_slam_params.yaml
```

**Laptop — visualization:**
```bash
rviz2
```
Set Fixed Frame to `map`. Add displays: **Map** (topic `/map`), **LaserScan** (`/scan`, size 0.03), **TF**. You should immediately see the first map patch appear around the box's starting pose.

**Camera smoke test (separate, don't run during SLAM walks — save the CPU):**
```bash
ros2 launch depthai_ros_driver camera.launch.py
# laptop: ros2 run rqt_image_view rqt_image_view  → /oak/rgb/image_raw
```

### Movement protocol — exact sequence

Put tape marks on the floor first: an **X at start**, and marks at 2 m and at each corner of a rectangle around the room. Hold the box steady at chest height, sensor level.

1. **Stand still on the X for 10 seconds.** The first map seed forms. Verify: walls near you appear straight, scan points sit on top of them.
2. **Walk straight 2 m** to your mark at slow pace (~0.3 m/s — about half normal walking speed). Stop. Pause 3 s. Verify: the pose arrow in RViz moved ~2 m (eyeball against the map grid — cells are 5 cm), walls didn't smear.
3. **Rotate 90° in place, slowly — take a full 4–5 seconds.** This is the harshest test for lidar-only odometry. Pause 3 s. Verify: the map didn't fracture into two overlaid copies. If it did, you rotated too fast — this is the single most common handheld failure.
4. **Walk the rectangle** around the room, pausing 2 s at each corner, rotating slowly at each. Verify at each corner: newly revealed walls connect cleanly to the existing map.
5. **Return to the X and stop.** This is the loop-closure moment — watch the map visibly "snap" tighter as slam_toolbox closes the loop. Verify: the pose arrow sits within ~10–15 cm of where it started, and every wall is a single crisp line, not a double image.
6. **Second lap, opposite direction**, passing behind any large obstacles, to fill occlusion shadows in the map.

**Acceptance criteria:** return-to-start error ≤ 15 cm; straight walls render straight; no doubled walls; loop closure visibly fires at step 5.

### Save the map, then test localization

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/room_v1
```

Then restart slam_toolbox in **localization mode** (in the yaml: `mode: localization`, add `map_file_name: /home/<user>/maps/room_v1`), start from the X, and walk the rectangle again — the pose should track smoothly on the *existing* map without modifying it. This is exactly how the final AGV will run day-to-day.

### Record everything

```bash
ros2 bag record /scan /tf /tf_static -o ~/bags/box_walk_1
```
This is the highest-leverage habit in this whole document: replay the bag on your laptop (`ros2 bag play`) and re-tune SLAM parameters as many times as you want **without re-walking the room**.

---

## 4. User Command Architecture — "Pick up barrel X, put it at Y"

Four layers, bottom to top:

**Semantic map layer.** After mapping, annotate named locations in a YAML the mission layer reads — pickup zones, drop zones, home/charge:
```yaml
locations:
  bay_a:   {x: 3.20, y: 1.10, yaw: 0.0}
  bay_b:   {x: 5.75, y: 1.10, yaw: 0.0}
  dropoff: {x: 0.80, y: 4.30, yaw: 1.57}
  home:    {x: 0.00, y: 0.00, yaw: 0.0}
```
Get the coordinates by clicking **Publish Point** in RViz on the finished map and reading `/clicked_point`.

**Barrel identity layer.** A warehouse of visually identical oil drums cannot be told apart by a neural net. Two-part solution: YOLO (running on your AI HAT+ — literally your day-job toolchain pointed at a new class) answers *"there is a drum at (x,y,z)"* for detection and safe approach; an **ArUco marker taped to each drum** answers *"this is drum #7"* — deterministic, costs nothing, works in bad lighting, and OpenCV's `aruco` module gives you full 6-DOF pose of the tag for the final gripper alignment. Maintain a small barrel registry node: `{id: 7, last_seen_pose: (x, y), status: stored|in_transit}`.

**Mission executor layer.** A state machine node (plain rclpy is fine; Nav2's behavior trees when you want retries/recovery for free):
```
IDLE → NAV_TO_APPROACH (Nav2 goal: 1 m in front of barrel, facing it)
     → VISUAL_ALIGN    (camera servo: center ArUco tag, creep to grip distance via depth)
     → GRIP            (your AGVControl.grip_close(), verify via AS5600 stall or camera)
     → NAV_TO_DROPOFF  (Nav2 goal: named location)
     → RELEASE → NAV_HOME → IDLE
```
The visual-align state exists because Nav2 gets you to ±10 cm at best — final barrel engagement needs camera guidance, not map coordinates.

**Operator layer.** Extend the Flask operator console you already have with a "Mission" tab: render the saved map as an image, overlay barrel pins from the registry, operator taps a barrel + picks a destination from a dropdown → POST to the mission node → live status feed ("Navigating to bay_a… Aligning… Gripping…"). Your existing E-stop must sit *above* Nav2 in authority: run a cmd_vel mux where the console's E-stop cuts the DAC and relay directly, regardless of what Nav2 is commanding. Autonomy never gets to overrule the red button.

---

## 5. Expert Insights — Bottlenecks & Pre-Build Improvements

**Compute budget on the Pi 5.** slam_toolbox (~1 core) + Nav2 (~1 core) + ROS2 overhead fits comfortably in 4 cores/8 GB — *if* you keep vision off the CPU. Rules: run YOLO on the Hailo, keep stereo depth on the OAK's own chip, never pipe full-resolution point clouds into a costmap on the Pi. Use 400P mono pairs for depth at ~15 fps; that's plenty for obstacle avoidance at AGV speeds. If you see SLAM stutter, check `htop` for a rogue image pipeline before touching SLAM params.

**Your camera cannot do odometry — plan around it.** No IMU in the OAK-D Lite means the real AGV's odometry rests entirely on your AS5600s: one on a rear wheel (distance) + one on the steering shaft (angle) → bicycle-model odometry. This is the correct next hardware task after the box test passes. Get closed-loop steering working first (AS5600 angle + BTS7960 bang-bang with a ±2° deadband is enough — the wiper motor's gearing self-holds), because Nav2's Ackermann controller assumes the wheel actually goes where it's told.

**Ackermann is not optional in software.** A front-steer tricycle cannot rotate in place and has a minimum turning radius. Configure Smac Hybrid-A* with your real measured turning radius, and Regulated Pure Pursuit for path following. If you leave Nav2 on its diff-drive defaults, it will plan pirouettes your chassis physically cannot execute, and "SLAM" will get blamed for what is actually a kinematics mismatch.

**Drums move — design localization for it.** slam_toolbox localizes against *everything* in the map, but the drums are exactly the objects your AGV exists to move. The walls, columns, and racking must carry localization. In a small test room this just works; in the real warehouse, map with drums in typical positions, and expect (test for) robustness when a few have moved. If localization gets flaky in drum-dense zones, that's your cue to re-map or lean harder on wall features (raise the lidar so it sees over the drums to the walls — a genuinely effective trick).

**FOV geometry for the grab.** The OAK-D Lite's ~73° horizontal FOV is narrow at close range: at 30 cm from a drum you see a ~44 cm slice — barely one drum face. Mount the camera tilted slightly down toward the gripper's working zone, and design the visual-align state to approach from ~1 m where the tag is comfortably in frame. Also accept that the AGV is blind behind: forbid autonomous reversing for now, or add a cheap rear ultrasonic later.

**Interference triple-check before blaming software.** The three phantom-bug generators for this exact hardware combo: USB3 jamming 2.4 GHz WiFi (use 5 GHz/Ethernet), the OAK-D browning out on the 600 mA default USB budget (fix in §1), and motor PWM noise coupling into USB when everything shares the chassis later (keep USB cables away from the 24V motor runs, and your star ground discipline from the motor phase already covers the rest).

**Highest-leverage habit: rosbags.** Every walk, every future drive — record it. Tuning SLAM from bags on the laptop is 10× faster than re-running hardware, and bags of failures are the only way to debug intermittent localization jumps after the fact.

**You already have a simulator — use it for Nav2, not for SLAM.** Your Gazebo `aegis_agv_ws` with the matching tricycle URDF is the right place to develop the Nav2 Ackermann config and the mission state machine *in parallel* with this sensor work. Sim is poor for validating SLAM (simulated lidar is too clean) but excellent for validating planners, controllers, and mission logic before the real chassis ever moves under its own authority.

---

## Quick-reference: what the box test does and doesn't prove

Proves: sensor wiring/power, driver stack, TF tree, SLAM quality in your actual space, map saving, localization mode — the entire perception half of the robot.

Does **not** prove: wheel odometry, steering closed-loop, Ackermann path following, cmd_vel → motor translation, E-stop authority over autonomy. Those are the next phase, on stands, with the map you're about to build.
