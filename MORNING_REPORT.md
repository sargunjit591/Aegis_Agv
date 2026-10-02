# MORNING REPORT — overnight run, 2026-10-02

## Onboarding confirmation (CLAUDE.md + PROGRESS.txt read in full)

1. Steering BTS7960 #2: RPWM (right) GPIO19, LPWM (left) GPIO16, active HIGH.
2. Gripper BTS7960 #1: RPWM (open) GPIO13, LPWM (close) GPIO12, active LOW.
3. Direction relay IN1 GPIO5, LOW = forward. Brake relay IN2 GPIO6, LOW = applied — polarity still unverified.
4. Throttle: DAC8532 ch A, CS GPIO23, SPI0 dev1 mode 1; 0.80 V idle, 1.50 V fwd, 1.28 V rev, 4.20 V max.
5. AS5600 on I2C1 0x36 (GPIO2/3), 1:1 on the pivot; BNO085 on UART0 RX GPIO15 (`/dev/ttyAMA0`); halls planned on GPIO17/27/22.
6. Two OSes: the 40-pin header is gpiochip4 (`pinctrl-rp1`) on Ubuntu and gpiochip0 on Raspberry Pi OS.
7. Opening the wrong chip succeeds silently and no pin moves, so GPIO code must detect the chip by label.
8. Stale bridge defaults (§7): `as5600_unwrap` True, centre 3746, `as5600_ccw_positive` True — hence the long launch flags.
9. Stale console values (§7): `AS5600_CENTER_RAW = 3746`, `AS5600_CCW_POSITIVE = True`, no look-ahead, `gpiochip_open(0)` hardcoded.
10. Rules I worked under: nothing published to the vehicle, safety code never simplified, rewrite rather than patch, pin maps identical.

---

## Summary

| Step | Result | Commit |
|---|---|---|
| 1 Onboarding + graphify | done | (graph is in `graphify-out/`, gitignored) |
| 2 Bridge + console rewrite | done, verified in software only | `6a86cfc` |
| 3 Wheel odometry design (arena, 4 agents) | done, core verified, ROS node never run | `71d9683` |
| This report + one test fix | | the commit after `71d9683` (`git log -1`) |

Baseline to diff against: `9de48c5`. Nothing was pushed. Nothing touched the Pi (no ssh/scp/ros2).

**Nothing in this report has run on hardware.** Every "passes" below means a
software check on this laptop. The bridge was not run under ROS2: `rclpy` is
not installed here (`ModuleNotFoundError: No module named 'rclpy'`).

**Three things in step 2 change how the vehicle behaves and need your eyes
before anything else** — see "Behaviour changes" below: the E-stop brake pulse
now releases after 2 s, the bridge's steering max-run backstop now actually
holds, and the console now enforces ±45° on manual steering.

---

## Step 1 — onboarding

- Read `CLAUDE.md` and `PROGRESS.txt` in full.
- Ran the graphify skill on the folder: 10 code files (AST) + 5 docs (one
  extraction agent). Result: 343 nodes, 645 edges, 14 communities.
  Outputs: `graphify-out/graph.html`, `GRAPH_REPORT.md`, `graph.json`.
  Refreshed with `python -m graphify update .` after the code changes.
- Health check flagged 78 dangling-endpoint edges and 5 self-loops (doc edges
  pointing at ids the AST pass names differently). The graph is usable but
  doc→code edges are incomplete; I used grep, not the graph, for every
  decision that mattered.
- The `graphify` command is not on PATH on this laptop; `python -m graphify …` works.

---

## Step 2 — bridge and console rewrite (`6a86cfc`)

Both files rewritten from scratch (ponytail lite). The console's HTML/CSS/JS
page was carried over verbatim from `9de48c5` apart from four small edits
(new button, its handler, the hint text, REV number).

### Old vs new defaults

**`ubuntu_ros2/agv_motor_bridge.py`**

| Parameter | Old default | New default | Why |
|---|---|---|---|
| `as5600_center_raw` | 3746 | **3791** | measured centre |
| `as5600_ccw_positive` | True | **False** | left must read positive |
| `as5600_unwrap` | True | **removed** (always single-turn) | encoder is 1:1 on the pivot |
| `as5600_sample_hz` | 100.0 | **removed** | only used by unwrap |
| `steer_gear_ratio` | 1.0 | **removed** | only used by unwrap |
| `steer_max_deg` | 45.0 | 45.0 | unchanged |
| all other 36 parameters | | unchanged | |
| status rate | 20 Hz (docstring said 5) | 20 Hz | docstring fixed |

Launch is now just `python3 agv_motor_bridge.py`. The old long command still
works as far as I know — rclpy ignores `-p` overrides for parameters that are
not declared — but I could not confirm that without ROS2.

**`rpi_os_console/agv_console_angle.py`**

| Constant | Old | New | Why |
|---|---|---|---|
| `AS5600_CENTER_RAW` | 3746 | **3791** | measured centre |
| `AS5600_CCW_POSITIVE` | True | **False** | left must read positive |
| gpiochip | `gpiochip_open(0)` | **`GPIOCHIP = -1`** (detect by label) | §5 |
| `CONTROL_TICK_S` | 0.05 (20 Hz) | **0.02 (50 Hz)** | look-ahead needs it (PROGRESS §10g) |
| throttle slew | `SLEW_STEP_V = 0.15`/tick = 3 V/s | **`SLEW_V_PER_S = 3.0`** (0.06 V/tick) | same 3 V/s — not weakened by the faster loop |
| runaway guard | `STEER_RUNAWAY_TICKS = 10` = 0.5 s | **`STEER_RUNAWAY_S = 0.5`** (25 ticks) | same 0.5 s |
| `STEER_STOP_LEAD_S` | — | **0.12** | ported from bridge |
| `STEER_RATE_ALPHA` | — | **0.35** | ported from bridge |
| `STEER_MAX_RUN_S` | — | **4.0** | ported from bridge |
| mock wheel | clamped ±60°, no coast | unclamped, coasts 0.12 s | so the mock can show an overshoot |

### Behaviour changes — read these

These are deliberate, and each one tightens safety, but they are changes to
motor behaviour that only you can confirm on stands.

1. **Bridge: the E-stop brake pulse now auto-releases after 2 s.** In REV 4
   `control_tick` returned on E-stop *before* the brake limit check, so the
   actuator stayed energised until E-stop was reset. I confirmed that by
   simulating the old file: `brake still engaged 10 s into E-stop: True`. That
   contradicts the "never held, ≤2 s" rule and the file's own docstring. The
   console already behaved correctly. If you *wanted* the brake held through
   an E-stop, say so — but the actuator is self-opening.
2. **Bridge: the steering max-run backstop now latches.** REV 4 cut the motor
   at 4 s and restarted it on the next tick (simulated old file: motor driven
   on 448 of 450 ticks with the command held for 9 s). It now stays off until
   the command is released or reversed, which is what its log message always
   claimed. This only matters when the encoder is missing or wrong.
3. **Bridge: `/cmd_vel` received during an E-stop is discarded**, so a command
   sent just before reset cannot act for the 0.5 s watchdog window after it.
   A node that is *still publishing* when you reset will move the AGV
   immediately — that was true before and is still true.
4. **Console: ±45° limit on every steering move.** Previously only go-to-angle
   was limited; hold-to-steer and timed runs could spin the wheel
   indefinitely. Now the same look-ahead limit as the bridge applies to all
   three, plus the bridge's >90° backstop and the 4 s max run. Timed steering
   runs longer than 4 s are cut at 4 s.
5. **Console: go-to-angle uses the look-ahead** and ends when the predicted
   angle reaches the target or crosses it, instead of reversing to chase a
   1.5° window at 5° per tick.
6. **Console: new "Set 0° here" button** (`POST /api/steer_home`), with a
   confirm dialog. Needed because the limit in (4) would otherwise trap an
   operator behind a stale centre (the PROGRESS §10e problem). It lasts until
   restart; edit `AS5600_CENTER_RAW` to keep it.
7. **Bridge: multi-turn unwrap mode removed.** It could not have worked: with
   the old default `as5600_unwrap=True` the startup probe did
   `math.degrees(None)`, raised, and the node disabled the encoder. Status
   fields `enc_accum_deg`, `enc_max_step`, `enc_lost`, `homed`, `gear_ratio`
   went with it; nothing in the repo read them. Old code is in `9de48c5`.

Lazier alternative I did not take (ponytail lite): limit the console port to
the two constants plus chip detection, about 30 changed lines, and leave manual
steering unlimited. I judged that "port the look-ahead" means nothing without
a limit to look ahead to. If you disagree, items 4–6 are the part to revert.

### Known limitation (both files, pre-existing in the bridge)

A steer command that starts with the wheel resting just *inside* the limit is
allowed, gets cut on the next tick, and coasts. In the mock (instant 100°/s,
0.12 s coast) that is one overshoot to about 57°; after that every further
command is refused. It cannot accumulate. Real overshoot depends on how fast
the wiper motor spins up in 20 ms — hardware only.

### Verification

Run from the repo root. The console test needs Flask, which is not installed
system-wide here; I used a throwaway venv outside the repo.

```
$ python -m py_compile ubuntu_ros2/agv_motor_bridge.py rpi_os_console/agv_console_angle.py ubuntu_ros2/agv_wheel_odom.py
(exit 0)

$ python tests/check_bridge_console.py          # 40 checks, exit 0
  ok   agv_motor_bridge.py: no duplicate top-level def/class (3 defined) []
  ok   agv_motor_bridge.py: class AGVMotorBridge has no duplicate method (24 methods) []
  ok   agv_console_angle.py: no duplicate top-level def/class (21 defined) []
  ok   agv_console_angle.py: no duplicate top-level assignment (44 names) []
  ok   agv_console_angle.py: class Hardware has no duplicate method (33 methods) []
  ok   agv_console_angle.py: no duplicate Flask route (17 routes) []
  ok   declare_parameter is called from exactly one place (found 1): the loop over PARAMS
  ok   PARAMS table has 40 parameters, each key unique
  ok   parameters read == parameters declared (undeclared: [], unused: [])
  ok   STEER_RPWM             19      | steer_rpwm_pin         19      | verified 19
  ok   STEER_LPWM             16      | steer_lpwm_pin         16      | verified 16
  ok   GRIP_RPWM              13      | grip_rpwm_pin          13      | verified 13
  ok   GRIP_LPWM              12      | grip_lpwm_pin          12      | verified 12
  ok   RELAY_DIR_PIN          5       | relay_dir_pin          5       | verified 5
  ok   BRAKE_PIN              6       | brake_pin              6       | verified 6
  ok   DAC_CS_PIN             23      | dac_cs_pin             23      | verified 23
  ok   AS5600_CENTER_RAW      3791    | as5600_center_raw      3791    | verified 3791
  ok   AS5600_CCW_POSITIVE    False   | as5600_ccw_positive    False   | verified False
  ... (24 shared constants in all, plus direction-relay polarity)
  ok   find_header_chip() is identical code in both files
  ok   agv_console_angle.py: no hardcoded gpiochip_open(0)
ALL CHECKS PASSED
```

"Verified" in that table means "equals the value in CLAUDE.md §4", checked by
AST. It does not mean the pin was observed to move.

```
$ <venv>/python tests/test_console_mock.py      # 48 checks, exit 0, ran 3 times
  ok   seek +30 -> landed +28.6 deg (tol 4), motor stopped, fault=None
  ok   seek -20 -> landed -18.3 deg (tol 4), motor stopped, fault=None
  ok   hold left: motor cut by the limit, wheel came to rest at +44.8 deg (want +40..+47)
  ok     right pressed 5 more times: wheel rests at -57.3 deg (bounded: limit + one coast), next press refused
  ok   motor cut after the max-run time: fault='Steering ran 0.5 s continuously — stopped.'
  ok   slew limited: biggest DAC step 0.06 V (limit 0.06), 0.80->2.00 V took 0.40 s (3 V/s => 0.40 s)
  ok   direction change while driving refused: 'Bring speed to 0 before changing direction'
  ok   drive stopped -> brake pulse (GPIO6=0)
  ok     held 2.01 s (limit 2.0 s), pin back to released level
  ok   manual APPLY also auto-releases within 2 s
  ok   steer / grip / throttle / seek / brake-release / timed all refused while E-stopped
  ok   E-stop brake pulse auto-released within 2 s while still E-stopped
  ok   no heartbeat -> ALL STOP after 2.0 s (timeout 2.0 s)
  ok   GET / serves the page
ALL CHECKS PASSED
```

This imports the real console in MOCK mode and drives `Hardware` against a
simulated wheel in real time (about 20 s), plus the routes through Flask's
test client. The first full run of this test had one failure, which is how I
found the limitation above; the check now asserts the bounded behaviour
instead of "goes nowhere".

```
$ python tests/test_bridge_logic.py             # 42 checks, exit 0
  ok   40 parameters declared, none twice (the stand-in raises on a duplicate)
  ok   defaults with NO launch flags: centre 3791, ccw_positive False, limit 45 deg
  ok   forward: throttle slewed to 1.50 V, biggest step 0.15 V (limit 0.15)
  ok   watchdog: throttle back at idle 0.60 s after the last cmd_vel
  ok   brake auto-released after 2.02 s (limit 2.0 s)
  ok   reverse: relay flipped only at 0.80 V (idle), then drove at 1.28 V
  ok   E-stop brake pulse auto-released after 2.02 s while STILL E-stopped
  ok   steer left held 2 s: stopped at the limit, wheel at +46.0 deg (left reads positive)
  ok   command still held 5 s more: motor stays OFF
  ok   sign inverted: >90 deg backstop stopped the wheel at +102.0 deg physical
  ok   sign inverted: runaway guard tripped, wheel stopped at +26.0 deg
  ok   gripper auto-stop at 8 s
  ok   shutdown leaves the brake RELEASED (de-energised) and throttle idle
ALL CHECKS PASSED
```

You told me not to try to run the bridge, and I did not run it under ROS2.
This script is a different thing: it imports the bridge with `rclpy` and the
message packages replaced by ~60 lines of stand-ins, a fake clock, and a
simulated wheel behind a fake AS5600. I added it because I had changed
safety logic and CLAUDE.md rule 4 asks for a simulated test of logic changes.
It proves the control logic does what I describe. It proves nothing about
real ROS2 behaviour (QoS, parameter types, timer scheduling) or hardware.

---

## Step 3 — wheel odometry node, design only (`71d9683`)

Files: `ubuntu_ros2/agv_wheel_odom.py` (200 lines), `tests/test_wheel_odom.py`.
Not referenced by any launch line, script, or doc.

### How the arena ran

The arena skill is installed but marked user-invocable only, so I followed
its six phases by hand. Its "portable runtime contract" file
(`pstack-pi/references/runtime.md`) does not exist on this machine; I used
plain subagents. Four candidates got one identical brief
(`.arena/odom/BRIEF.md`) on four different models, then one read-only
cross-judge. Five agents in total.

I also wrote an independent referee (`.arena/odom/referee.py`) so the ranking
did not rest on reading alone: 14 reference trajectories against closed-form
answers, then each candidate's own harness re-run against six deliberately
broken cores.

| Candidate | Referee trajectories | Own harness catches broken core | Core+node lines | Integration |
|---|---|---|---|---|
| 1 | 14/14 | 6/6 | 200 | mean curvature + exact chord |
| 2 | 14/14 | 6/6 | 222 | mean curvature + exact chord |
| 3 | 14/14 | 6/6 | 198 | mean curvature + exact chord |
| 4 | 11/14 | 6/6 | 240, no real node | Euler; 0.86 m off at 25 counts/update |

Candidates 1, 2 and 3 independently converged on the same integrator, to the
same error digits. That agreement is the strongest evidence the shape is right.

### Pick

**Base: candidate 3.** Smallest correct core, and the only node that fails
loud: when the steering angle is missing, stale, or out of range it publishes
no `/odom` and no TF and discards the distance, instead of integrating at a
guessed angle.

**The cross-judge disagreed on the base.** It scored candidates 1 and 3 equal
(22/25 each) and recommended candidate 1 for its harness, with candidate 3's
fault policy grafted in. I kept candidate 3 and grafted candidate 1's harness
ideas instead. The two routes land on nearly the same file; I chose the one
that starts from less code, since minimal code was your second criterion.

Grafted:
- from 1: constant-steer paths are checked at **every sample**, not just the
  end point, so an error that cancels on a symmetric path is still caught.
- from 1: a check that the node source has one publisher and names no
  actuator topic.
- from 2 (and flagged by the judge as a bug in 3): NaN steering now counts as
  invalid. Before, `abs(nan) > limit` was False and NaN would have poisoned
  the pose permanently.
- from the judge: `rclpy.shutdown()` on exit.

Rejected:
- 1 and 2's "hold the last steering angle and keep publishing with inflated
  covariance". slam_toolbox and TF consumers do not read covariance, so that
  is silently wrong heading.
- 1's mutants made by text-replacing the core's source: they break the moment
  the core is reformatted. Kept 3's independent wrong-model class.
- 2's 0.2 s velocity window. Real issue, wrong time: see below.
- 4: nothing.

### Verification

```
$ python tests/test_wheel_odom.py               # 35 checks, exit 0
PASS  straight forward 11.4 m  worst pos err 3.20e-14 m  worst yaw err 0.00e+00 rad (tol 1e-09)
PASS  arc left 30 deg, forward  worst pos err 1.13e-15 m  worst yaw err 8.88e-16 rad (tol 1e-09)
PASS  arc left 45 deg past 180 deg (yaw wrap)  worst pos err 1.40e-15 m ...
PASS  straight reverse 5.7 m  worst pos err 4.44e-15 m ...
PASS  arc reverse, wheel left 30 deg  worst pos err 1.13e-15 m ...
PASS  S-curve: arc left then arc right  pos err 1.35e-15 m (tol 1.0e-09)
PASS  sweep left->right while moving  pos err 3.41e-04 m (tol 3.5e-04)  yaw err 6.94e-18 rad
PASS  sweep left->right in reverse  pos err 3.41e-04 m (tol 3.5e-04)
PASS  v, omega forward / reverse
PASS  count jump rejected, then resumes from new baseline
PASS  node source: one publisher (/odom), no actuator topics
PASS  wrong model caught: steer sign flipped  (14/16 checks trip)
PASS  wrong model caught: sin(steer) instead of tan  (14/16 checks trip)
PASS  wrong model caught: count taken as front-wheel distance  (14/16 checks trip)
PASS  wrong model caught: Euler: step along the old heading  (9/16 checks trip)
... 13 wrong models, all caught
ALL PASSED
```

The synthesized file also passes the referee: 14/14 trajectories, 6/6 broken
cores caught. It imports with no `rclpy` and no `lgpio` loaded.

The sweep tolerance (0.35 mm) is the derived discretisation bound and the
observed error is 0.34 mm, so that margin is thin by construction, not by luck.

---

## What I could NOT verify

- **Anything on hardware.** No pin, relay, DAC, encoder or motor was touched.
- **The bridge under real ROS2.** Not run. Parameter declaration, QoS, timers
  and message types are exercised only against stand-ins.
- **That old launch flags for removed parameters are ignored** rather than
  rejected by rclpy. I believe so; untested.
- **gpiochip detection on either OS.** `find_header_chip()` in the bridge is
  the old logic reshaped into a module function; in the console it is new.
  It has never executed here (no `lgpio`). On Raspberry Pi OS it should pick
  gpiochip0 by the `rp1` label.
- **The wheel odometry ROS node.** Compiled, never executed. Its message
  filling, TF broadcast, parameter handling and its use of `HallCounter` are
  unrun. Only the `WheelOdometry` class is tested.
- **The console page in a browser.** The route returns the page and the new
  button's markup is in it; I did not load it in a browser.

---

## Hardware tests for you, in order

Wheels off the ground for all of 2–4. Never run console and bridge together.

1. **Still first: the straighten fix** (CLAUDE.md §8.1) — now with the new
   bridge and no flags. Front wheel straight, then:
   ```
   # terminal 5
   cd ~/aegis_agv/ubuntu_ros2 && python3 agv_motor_bridge.py
   ```
   Startup log must show `centre raw 3791, now ~0.0 deg` and the gpiochip4
   line. If it warns that the startup angle is not ~0, stop and re-home.
2. **Bridge signs.** `ros2 topic echo /agv/status --field data` while nudging
   steering: left must read positive, limit must hold near ±45°.
3. **Bridge E-stop brake.** Publish E-stop true, leave it: the brake must
   apply once and release by itself at ~2 s, and stay released. This is the
   changed behaviour.
4. **Console on Raspberry Pi OS.** Journal must show the header detected as
   gpiochip0 by label. Then: hold-to-steer stops near ±45° each way; go-to
   +30 / −30 / 0 lands within a few degrees without reversing; "Set 0° here"
   zeroes the readout; throttle ramps as before (3 V/s); brake pulses and
   releases; E-stop and the 2 s heartbeat watchdog still stop everything.
5. **Look-ahead tuning on the console.** `STEER_STOP_LEAD_S = 0.12` is the
   bridge's value. Left coasts 15°, right 6.3°, so go-to will not land equally
   on both sides.
6. **Brake polarity** is still unverified in both files.

Do not do yet: anything with `agv_wheel_odom.py`. It needs the halls wired,
`hall_odometry.py --check`, then `--calibrate`.

---

## Decisions that are yours

1. **Which wheel carries the hub motor?** PROGRESS.txt says "two rear wheels
   on a BLDC-driven axle"; CLAUDE.md says "BLDC hub motor". The odometry
   treats the hall count as distance at the rear-axle centre, per
   `Δθ = d·tan(δ)/L`. If the motor is in the steered front wheel, distance
   needs `·cos δ`; if it drives one rear wheel only, it needs a track-offset
   term. Three of the four candidates raised this unprompted. The harness
   already contains the front-wheel variant as a "wrong model", so switching
   is a small, testable change.
2. **Brake held or released during E-stop** (behaviour change 1).
3. **Console scope** (behaviour changes 4–6) — keep, or revert to the minimal
   port.
4. **CLAUDE.md and PROGRESS.txt are now out of date and I did not edit them.**
   CLAUDE.md "wins" over code, and the new defaults are untested on hardware,
   so I left it conservative. Once step 1–4 above pass, these need changing:
   §5 "the console still hardcodes chip 0"; §7 "Stale defaults" block; §8
   step 2 (done); §10 launch line 5; §3 tree (add `tests/`,
   `agv_wheel_odom.py`). Say the word and I will do it.
5. `calibration/as5600_calibrate.py` still prints `-p as5600_unwrap` /
   `-p steer_gear_ratio` advice for parameters that no longer exist. Untouched.

## Notes for the next steps

- **EKF step:** odometry `v` is one count per tick = 0.23 m/s granularity at
  20 Hz, and the covariances are deliberately left at zero.
  `robot_localization` will need real covariances and probably a smoothed
  velocity. Candidate 2's 0.2 s window is the ready-made answer.
- **The seam.** Neither file detects a ±170° → ∓170° jump. The odometry node
  refuses any steering angle beyond ±60°, which covers its own input.
- `verification-before-completion` (superpowers) is not installed on this
  machine. I applied the same discipline by hand: every figure above is from a
  fresh run made after the last code change.
- New in the repo: `tests/` (4 scripts). Untracked and left alone: `.claude/`.
  Scratch files (`.arena/`, `graphify-out/`) are gitignored.
