#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — AGV OPERATOR CONSOLE  [REV 4]
============================================================
Run on the Pi, open http://<pi-ip>:8000 from any phone/laptop.

HARDWARE MAP
  Steering : BTS7960 #2  GPIO19=RPWM(right) GPIO16=LPWM(left)  active HIGH
  Gripper  : BTS7960 #1  GPIO13=RPWM(open)  GPIO12=LPWM(close) active LOW
  Dir relay: relay IN1   GPIO5   LOW = FORWARD, HIGH = REVERSE
  BRAKE    : relay IN2   GPIO6   24 V self-opening actuator;
                                 the relay shorts its two wires
  Throttle : Waveshare HAT DAC0, SPI(0,1) mode 1, CS = GPIO23
             0.80 V idle ... 4.20 V max
             measured start thresholds: forward 1.50 V, reverse 1.28 V
  Steering feedback : AS5600, I2C bus 1 @ 0x36, centre raw 3746

BRAKE — PULSED, NEVER HELD
  The actuator is self-opening, so it must not sit energised:
    * RELEASED at startup and at shutdown
    * one pulse when the drive stops, one pulse on E-stop
    * EVERY engagement auto-releases after BRAKE_MAX_ENGAGE_S
      (2.0 s), including a manual APPLY
    * a drive command releases it immediately
    * will not re-fire until the AGV has moved again
  So this is a stopping aid, not a parking brake.

  !! VERIFY POLARITY ON STANDS BEFORE DRIVING !!
  BRAKE_ENGAGE_LEVEL below decides which GPIO level applies the
  brake. It depends on the relay module polarity AND on how the
  actuator behaves when its wires are shorted:
      0 = LOW applies the brake   (default: active-LOW relay)
      1 = HIGH applies the brake
  Wheels off the ground, no throttle: press APPLY and RELEASE and
  see which one actually locks the wheel. If they are backwards,
  change this one number.

THE OTHER THING THAT MAY NEED FLIPPING
  FORWARD_RELAY_LEVEL — 0 means GPIO5 LOW is forward. If the AGV
  drives backwards when told forward, change it to 1.

DEPENDENCIES
  pip install flask lgpio spidev smbus2 --break-system-packages

Without lgpio/spidev the server runs in MOCK MODE with a simulated
steering angle, so the whole UI is testable off-Pi.
============================================================
"""

import threading
import time
import atexit

from flask import Flask, request, jsonify, Response

# ==================================================================
# CONFIG — every tunable lives here, exactly once
# ==================================================================

# ---- steering: BTS7960 #2 ----
STEER_RPWM = 19          # right
STEER_LPWM = 16          # left
STEER_ACTIVE_LOW = False

# ---- gripper: BTS7960 #1 ----
GRIP_RPWM = 13           # open
GRIP_LPWM = 12           # close
GRIP_ACTIVE_LOW = True

# ---- drive direction: relay IN1 ----
RELAY_DIR_PIN = 5        # IN1 — FWD/REV
FORWARD_RELAY_LEVEL = 0  # 0 = LOW is forward; change to 1 if reversed

# ---- mechanical brake: relay IN2 ----
BRAKE_PIN = 6            # IN2 — brake actuator
BRAKE_ENGAGE_LEVEL = 0   # 0 = LOW applies the brake (active-LOW relay)
BRAKE_ENABLE = True
BRAKE_RELEASE_LEAD_S = 0.35   # brake lets go this long before throttle rises
BRAKE_MAX_ENGAGE_S = 2.0      # HARD LIMIT — auto-release after this long.
                              # The actuator is self-opening and must never
                              # sit energised.

# ---- throttle DAC ----
DAC_CS_PIN = 23
DAC_CHANNEL_A = 0x30
HAT_VREF = 5.0
THROTTLE_IDLE_V = 0.80
THROTTLE_MAX_V = 4.20
SPEED_CAP_PCT = 100      # lower (e.g. 50) to cap top speed for new operators

# ---- AS5600 steering encoder ----
AS5600_BUS = 1
AS5600_ADDR = 0x36
AS5600_CENTER_RAW = 3746
AS5600_CCW_POSITIVE = True

# ---- steering closed loop ----
STEER_MAX_DEG = 45.0
STEER_DEADBAND_DEG = 1.5
STEER_SEEK_TIMEOUT_S = 12.0
STEER_RUNAWAY_TICKS = 10

# ---- timing ----
SLEW_STEP_V = 0.15
CONTROL_TICK_S = 0.05    # 20 Hz
WATCHDOG_TIMEOUT_S = 2.0
TIMED_MAX_S = 15.0

# ---- mock simulation ----
MOCK_STEER_RATE_DPS = 100.0

HTTP_PORT = 8000

# ==================================================================
try:
    import lgpio
    import spidev
    MOCK = False
except ImportError:
    MOCK = True
    print("[MOCK MODE] lgpio/spidev not found — hardware calls printed only.")

try:
    import smbus2
    HAVE_SMBUS = True
except ImportError:
    HAVE_SMBUS = False


class Hardware:
    """Owns all GPIO/SPI/I2C access. Thread-safe via one lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.current_v = THROTTLE_IDLE_V
        self.target_v = THROTTLE_IDLE_V
        self.direction = "forward"
        self.steer_state = "stop"
        self.grip_state = "stop"
        self.estopped = False
        self.action_token = {"steer": 0, "grip": 0}

        # brake
        self.brake_engaged = False       # starts RELEASED
        self.brake_released_at = time.monotonic()
        self.brake_engaged_at = None     # when the current pulse started
        self.brake_pulse_spent = False   # pulse used; re-arms on motion
        self.brake_was_moving = False
        self.brake_manual = None         # None = auto, True/False = override

        # steering closed loop
        self.angle_deg = None
        self.seek_target = None
        self.seek_started = 0.0
        self.steer_fault = None
        self._prev_abs_err = None
        self._worsening = 0
        self._mock_angle = 0.0
        self._mock_last = time.monotonic()

        self.as5600 = None

        if not MOCK:
            self.h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self.h, STEER_RPWM, 1 if STEER_ACTIVE_LOW else 0)
            lgpio.gpio_claim_output(self.h, STEER_LPWM, 1 if STEER_ACTIVE_LOW else 0)
            lgpio.gpio_claim_output(self.h, GRIP_RPWM, 1 if GRIP_ACTIVE_LOW else 0)
            lgpio.gpio_claim_output(self.h, GRIP_LPWM, 1 if GRIP_ACTIVE_LOW else 0)
            lgpio.gpio_claim_output(self.h, RELAY_DIR_PIN, FORWARD_RELAY_LEVEL)
            lgpio.gpio_claim_output(self.h, DAC_CS_PIN, 1)
            if BRAKE_ENABLE:
                # boot RELEASED — the actuator is self-opening and must not
                # sit energised
                lgpio.gpio_claim_output(self.h, BRAKE_PIN, 1 - BRAKE_ENGAGE_LEVEL)

            self.spi = spidev.SpiDev()
            self.spi.open(0, 1)
            self.spi.max_speed_hz = 20000000
            self.spi.mode = 0b01
            self._write_dac(THROTTLE_IDLE_V)

        if HAVE_SMBUS and not MOCK:
            try:
                self.as5600 = smbus2.SMBus(AS5600_BUS)
                print(f"[AS5600] OK — centre raw {AS5600_CENTER_RAW}, "
                      f"now {self.read_angle():+.1f} deg")
            except Exception as e:
                self.as5600 = None
                print(f"[AS5600] UNAVAILABLE: {e} — angle features disabled")

    # ---------- low-level ----------
    def _pin(self, pin, active, active_low):
        level = (0 if active else 1) if active_low else (1 if active else 0)
        if MOCK:
            print(f"[MOCK] GPIO{pin} -> {level}")
        else:
            lgpio.gpio_write(self.h, pin, level)

    def _write_dac(self, voltage):
        voltage = max(0.0, min(HAT_VREF, voltage))
        value = int((voltage / HAT_VREF) * 65535) & 0xFFFF
        if MOCK:
            print(f"[MOCK] DAC0 -> {voltage:.2f} V")
            return
        lgpio.gpio_write(self.h, DAC_CS_PIN, 0)
        self.spi.xfer2([DAC_CHANNEL_A, (value >> 8) & 0xFF, value & 0xFF])
        lgpio.gpio_write(self.h, DAC_CS_PIN, 1)

    # ---------- brake (the ONLY place the brake pin is written) ----------
    def _brake(self, engage):
        if not BRAKE_ENABLE or engage == self.brake_engaged:
            return
        level = BRAKE_ENGAGE_LEVEL if engage else (1 - BRAKE_ENGAGE_LEVEL)
        if MOCK:
            print(f"[MOCK] GPIO{BRAKE_PIN} (brake) -> {level} "
                  f"({'APPLY' if engage else 'RELEASE'})")
        else:
            lgpio.gpio_write(self.h, BRAKE_PIN, level)
        self.brake_engaged = engage
        if engage:
            self.brake_engaged_at = time.monotonic()
        else:
            self.brake_released_at = time.monotonic()
            self.brake_engaged_at = None

    def set_brake_mode(self, mode):
        """mode: 'auto' | 'apply' | 'release'"""
        with self.lock:
            if mode == "auto":
                self.brake_manual = None
                return True, "brake: AUTO"
            if mode == "release":
                if self.estopped:
                    return False, "E-stop active"
                self.brake_manual = False
            else:
                self.brake_manual = True
            self._brake(self.brake_manual)
            return True, f"brake: {'APPLIED' if self.brake_manual else 'RELEASED'} (manual)"

    # ---------- AS5600 ----------
    def read_angle(self):
        """Signed degrees from centre. Wrap-safe for a centre near 0/4095."""
        if MOCK:
            return self._mock_angle
        if self.as5600 is None:
            return None
        hi = self.as5600.read_byte_data(AS5600_ADDR, 0x0E)
        lo = self.as5600.read_byte_data(AS5600_ADDR, 0x0F)
        raw = ((hi << 8) | lo) & 0x0FFF
        diff = (raw - AS5600_CENTER_RAW + 2048) % 4096 - 2048
        deg = diff * (360.0 / 4096.0)
        return deg if AS5600_CCW_POSITIVE else -deg

    def _mock_step(self):
        now = time.monotonic()
        dt = now - self._mock_last
        self._mock_last = now
        if self.steer_state == "left":
            self._mock_angle += MOCK_STEER_RATE_DPS * dt
        elif self.steer_state == "right":
            self._mock_angle -= MOCK_STEER_RATE_DPS * dt
        self._mock_angle = max(-60.0, min(60.0, self._mock_angle))

    # ---------- steering ----------
    def _steer_hw(self, cmd):
        self.action_token["steer"] += 1
        self._pin(STEER_RPWM, cmd == "right", STEER_ACTIVE_LOW)
        self._pin(STEER_LPWM, cmd == "left", STEER_ACTIVE_LOW)
        self.steer_state = cmd
        return self.action_token["steer"]

    def steer(self, cmd):
        with self.lock:
            if self.estopped and cmd != "stop":
                return self.action_token["steer"]
            self.seek_target = None          # manual overrides a seek
            return self._steer_hw(cmd)

    def at_limit(self, cmd, angle):
        if angle is None:
            return False
        if cmd == "left" and angle >= STEER_MAX_DEG:
            return True
        if cmd == "right" and angle <= -STEER_MAX_DEG:
            return True
        return False

    def seek_angle(self, deg):
        with self.lock:
            if self.estopped:
                return False, "E-stop active"
            if self.as5600 is None and not MOCK:
                return False, "AS5600 unavailable"
            try:
                deg = float(deg)
            except (TypeError, ValueError):
                return False, "Invalid angle"
            if abs(deg) > STEER_MAX_DEG:
                return False, f"Limit is +/-{STEER_MAX_DEG:.0f} deg"
            self.steer_fault = None
            self._prev_abs_err = None
            self._worsening = 0
            self.seek_target = deg
            self.seek_started = time.monotonic()
            return True, f"seeking {deg:+.1f} deg"

    def cancel_seek(self):
        with self.lock:
            self.seek_target = None
            self._steer_hw("stop")

    def _seek_tick(self, angle):
        if self.seek_target is None or angle is None:
            return
        if time.monotonic() - self.seek_started > STEER_SEEK_TIMEOUT_S:
            self.steer_fault = "Seek timed out"
            self.seek_target = None
            self._steer_hw("stop")
            return

        err = self.seek_target - angle
        abs_err = abs(err)

        if abs_err <= STEER_DEADBAND_DEG:
            self.seek_target = None
            self._steer_hw("stop")
            self._prev_abs_err = None
            self._worsening = 0
            return

        cmd = "left" if err > 0 else "right"
        if self.at_limit(cmd, angle):
            self.steer_fault = f"Mechanical limit ({cmd})"
            self.seek_target = None
            self._steer_hw("stop")
            return

        if self.steer_state in ("left", "right") and self._prev_abs_err is not None:
            if abs_err > self._prev_abs_err + 1e-4:
                self._worsening += 1
            else:
                self._worsening = 0
            if self._worsening >= STEER_RUNAWAY_TICKS:
                self.steer_fault = ("RUNAWAY — angle moved away from target. "
                                    "Flip AS5600_CCW_POSITIVE or swap "
                                    "STEER_RPWM/STEER_LPWM.")
                self.seek_target = None
                self._steer_hw("stop")
                return
        self._prev_abs_err = abs_err

        if self.steer_state != cmd:
            self._steer_hw(cmd)

    # ---------- gripper ----------
    def grip(self, cmd):
        with self.lock:
            if self.estopped and cmd != "stop":
                return self.action_token["grip"]
            self.action_token["grip"] += 1
            self._pin(GRIP_RPWM, cmd == "open", GRIP_ACTIVE_LOW)
            self._pin(GRIP_LPWM, cmd == "close", GRIP_ACTIVE_LOW)
            self.grip_state = cmd
            return self.action_token["grip"]

    # ---------- timed runs ----------
    def timed(self, group, cmd, secs):
        with self.lock:
            if self.estopped:
                return False, "E-stop active"
        try:
            secs = float(secs)
        except (TypeError, ValueError):
            return False, "Invalid duration"
        secs = max(0.05, min(TIMED_MAX_S, secs))
        token = self.steer(cmd) if group == "steer" else self.grip(cmd)
        t = threading.Timer(secs, self._timed_stop, args=(group, token))
        t.daemon = True
        t.start()
        return True, secs

    def _timed_stop(self, group, token):
        with self.lock:
            if self.action_token[group] != token:
                return                      # superseded
        if group == "steer":
            self.steer("stop")
        else:
            self.grip("stop")

    # ---------- drive ----------
    def _relay(self, direction):
        level = (FORWARD_RELAY_LEVEL if direction == "forward"
                 else 1 - FORWARD_RELAY_LEVEL)
        if MOCK:
            print(f"[MOCK] GPIO{RELAY_DIR_PIN} (dir) -> {level} ({direction})")
        else:
            lgpio.gpio_write(self.h, RELAY_DIR_PIN, level)
        self.direction = direction

    def set_direction(self, direction):
        with self.lock:
            if self.estopped:
                return False, "E-stop active"
            if (self.current_v > THROTTLE_IDLE_V + 0.05
                    or self.target_v > THROTTLE_IDLE_V + 0.05):
                return False, "Bring speed to 0 before changing direction"
            self._relay(direction)
            return True, direction

    def set_speed_percent(self, pct):
        with self.lock:
            if self.estopped:
                return False, "E-stop active"
            pct = max(0.0, min(float(SPEED_CAP_PCT), float(pct)))
            self.target_v = THROTTLE_IDLE_V + (pct / 100.0) * (THROTTLE_MAX_V - THROTTLE_IDLE_V)
            return True, pct

    def set_voltage(self, volts):
        with self.lock:
            if self.estopped:
                return False, "E-stop active"
            try:
                v = float(volts)
            except (TypeError, ValueError):
                return False, "Invalid voltage"
            clamped = False
            if v < THROTTLE_IDLE_V:
                v, clamped = THROTTLE_IDLE_V, True
            if v > THROTTLE_MAX_V:
                v, clamped = THROTTLE_MAX_V, True
            cap_v = THROTTLE_IDLE_V + (SPEED_CAP_PCT / 100.0) * (THROTTLE_MAX_V - THROTTLE_IDLE_V)
            capped = False
            if v > cap_v:
                v, capped = cap_v, True
            self.target_v = v
            msg = f"{v:.2f} V"
            if capped:
                msg += f" (cap {SPEED_CAP_PCT:.0f}%)"
            elif clamped:
                msg += " (clamped)"
            return True, msg

    # ---------- safety ----------
    def all_stop(self):
        with self.lock:
            self.action_token["steer"] += 1
            self.action_token["grip"] += 1
            self.seek_target = None
            self.target_v = THROTTLE_IDLE_V
            self.brake_manual = None
            self.brake_pulse_spent = False
            self._brake(True)              # pulse; auto-releases on the limit
            self._pin(STEER_RPWM, False, STEER_ACTIVE_LOW)
            self._pin(STEER_LPWM, False, STEER_ACTIVE_LOW)
            self._pin(GRIP_RPWM, False, GRIP_ACTIVE_LOW)
            self._pin(GRIP_LPWM, False, GRIP_ACTIVE_LOW)
            self.steer_state = "stop"
            self.grip_state = "stop"

    def estop(self):
        with self.lock:
            self.estopped = True
            self.action_token["steer"] += 1
            self.action_token["grip"] += 1
            self.seek_target = None
            self.target_v = self.current_v = THROTTLE_IDLE_V
            self._write_dac(THROTTLE_IDLE_V)
            self.brake_manual = None
            self.brake_pulse_spent = False
            self._brake(True)              # pulse; auto-releases on the limit
            self._pin(STEER_RPWM, False, STEER_ACTIVE_LOW)
            self._pin(STEER_LPWM, False, STEER_ACTIVE_LOW)
            self._pin(GRIP_RPWM, False, GRIP_ACTIVE_LOW)
            self._pin(GRIP_LPWM, False, GRIP_ACTIVE_LOW)
            self.steer_state = "stop"
            self.grip_state = "stop"

    def estop_reset(self):
        with self.lock:
            self.estopped = False
            self.steer_fault = None

    def clear_fault(self):
        with self.lock:
            self.steer_fault = None
            self._prev_abs_err = None
            self._worsening = 0

    # ---------- control loop tick ----------
    def tick(self):
        if MOCK:
            self._mock_step()
        try:
            a = self.read_angle()
        except Exception:
            a = None

        with self.lock:
            self.angle_deg = a
            if not self.estopped:
                self._seek_tick(a)

            wants_drive = self.target_v > THROTTLE_IDLE_V + 0.05
            now_t = time.monotonic()

            # ---- brake: PULSE, never held ----
            if BRAKE_ENABLE:
                # 1. hard auto-release — auto, manual and E-stop alike
                if (self.brake_engaged and self.brake_engaged_at is not None
                        and (now_t - self.brake_engaged_at) >= BRAKE_MAX_ENGAGE_S):
                    self._brake(False)
                    self.brake_pulse_spent = True
                    self.brake_manual = None       # drop any manual hold
                    print(f"[BRAKE] auto-released after "
                          f"{BRAKE_MAX_ENGAGE_S:.1f}s limit")

                # 2. a drive command always wins
                if wants_drive and self.brake_engaged:
                    self._brake(False)

                # 3. re-arm once actually driving
                if wants_drive:
                    self.brake_pulse_spent = False

                # 4. one pulse on the driving -> stopped transition
                if (self.brake_manual is None and not wants_drive
                        and self.brake_was_moving
                        and not self.brake_pulse_spent):
                    self._brake(True)

                self.brake_was_moving = wants_drive

            # hold throttle at idle until the brake has actually let go
            eff_target = self.target_v
            if wants_drive and BRAKE_ENABLE:
                if self.brake_engaged:
                    eff_target = THROTTLE_IDLE_V
                elif (self.brake_released_at is not None and
                      (time.monotonic() - self.brake_released_at) < BRAKE_RELEASE_LEAD_S):
                    eff_target = THROTTLE_IDLE_V

            if abs(eff_target - self.current_v) >= 0.001:
                delta = eff_target - self.current_v
                self.current_v += max(-SLEW_STEP_V, min(SLEW_STEP_V, delta))
                self._write_dac(self.current_v)

    def speed_percent_now(self):
        span = THROTTLE_MAX_V - THROTTLE_IDLE_V
        return max(0.0, (self.current_v - THROTTLE_IDLE_V) / span * 100.0)

    def shutdown(self):
        self.estop()
        self._brake(False)   # never leave the actuator energised
        if not MOCK:
            self.spi.close()
            lgpio.gpiochip_close(self.h)
        if self.as5600 is not None:
            try:
                self.as5600.close()
            except Exception:
                pass


hw = Hardware()
atexit.register(hw.shutdown)

# ------------------------------------------------------------------
last_heartbeat = time.monotonic()
watchdog_tripped = False


def control_loop():
    global watchdog_tripped
    while True:
        hw.tick()
        silent = time.monotonic() - last_heartbeat
        moving = (hw.target_v > THROTTLE_IDLE_V + 0.05
                  or hw.steer_state != "stop" or hw.grip_state != "stop")
        if silent > WATCHDOG_TIMEOUT_S and moving:
            print(f"[WATCHDOG] no heartbeat {silent:.1f}s — ALL STOP")
            hw.all_stop()
            watchdog_tripped = True
        time.sleep(CONTROL_TICK_S)


threading.Thread(target=control_loop, daemon=True).start()

# ------------------------------------------------------------------
app = Flask(__name__)


@app.route("/api/heartbeat", methods=["POST"])
def heartbeat():
    global last_heartbeat, watchdog_tripped
    last_heartbeat = time.monotonic()
    tripped = watchdog_tripped
    watchdog_tripped = False
    return jsonify(ok=True, watchdog_tripped=tripped)


@app.route("/api/status")
def status():
    return jsonify(
        estopped=hw.estopped,
        direction=hw.direction,
        speed_pct=round(hw.speed_percent_now(), 1),
        throttle_v=round(hw.current_v, 2),
        steer=hw.steer_state,
        grip=hw.grip_state,
        brake=("applied" if hw.brake_engaged else "released"),
        brake_mode=("auto" if hw.brake_manual is None else "manual"),
        brake_max_s=BRAKE_MAX_ENGAGE_S,
        angle_deg=(round(hw.angle_deg, 1) if hw.angle_deg is not None else None),
        seek_target=(round(hw.seek_target, 1) if hw.seek_target is not None else None),
        steer_fault=hw.steer_fault,
        encoder_ok=(hw.as5600 is not None or MOCK),
        mock=MOCK,
    )


@app.route("/api/steer", methods=["POST"])
def api_steer():
    cmd = (request.json or {}).get("cmd", "stop")
    if cmd not in ("left", "right", "stop"):
        return jsonify(ok=False, msg="bad cmd"), 400
    hw.steer(cmd)
    return jsonify(ok=True)


@app.route("/api/steer_to", methods=["POST"])
def api_steer_to():
    ok, msg = hw.seek_angle((request.json or {}).get("deg"))
    return jsonify(ok=ok, msg=msg)


@app.route("/api/steer_cancel", methods=["POST"])
def api_steer_cancel():
    hw.cancel_seek()
    return jsonify(ok=True)


@app.route("/api/steer_timed", methods=["POST"])
def api_steer_timed():
    d = (request.json or {}).get("dir")
    if d not in ("left", "right"):
        return jsonify(ok=False, msg="bad dir"), 400
    ok, res = hw.timed("steer", d, (request.json or {}).get("secs"))
    return jsonify(ok=ok, msg=(f"{d} for {res:.2f}s" if ok else res))


@app.route("/api/grip", methods=["POST"])
def api_grip():
    cmd = (request.json or {}).get("cmd", "stop")
    if cmd not in ("open", "close", "stop"):
        return jsonify(ok=False, msg="bad cmd"), 400
    hw.grip(cmd)
    return jsonify(ok=True)


@app.route("/api/grip_timed", methods=["POST"])
def api_grip_timed():
    cmd = (request.json or {}).get("cmd")
    if cmd not in ("open", "close"):
        return jsonify(ok=False, msg="bad cmd"), 400
    ok, res = hw.timed("grip", cmd, (request.json or {}).get("secs"))
    return jsonify(ok=ok, msg=(f"{cmd} for {res:.2f}s" if ok else res))


@app.route("/api/brake", methods=["POST"])
def api_brake():
    mode = (request.json or {}).get("mode", "auto")
    if mode not in ("auto", "apply", "release"):
        return jsonify(ok=False, msg="bad mode"), 400
    ok, msg = hw.set_brake_mode(mode)
    return jsonify(ok=ok, msg=msg)


@app.route("/api/direction", methods=["POST"])
def api_direction():
    d = (request.json or {}).get("dir")
    if d not in ("forward", "reverse"):
        return jsonify(ok=False, msg="bad dir"), 400
    ok, msg = hw.set_direction(d)
    return jsonify(ok=ok, msg=msg)


@app.route("/api/speed", methods=["POST"])
def api_speed():
    ok, msg = hw.set_speed_percent((request.json or {}).get("pct", 0))
    return jsonify(ok=ok, msg=msg)


@app.route("/api/voltage", methods=["POST"])
def api_voltage():
    ok, msg = hw.set_voltage((request.json or {}).get("volts"))
    return jsonify(ok=ok, msg=msg)


@app.route("/api/clear_fault", methods=["POST"])
def api_clear_fault():
    hw.clear_fault()
    return jsonify(ok=True)


@app.route("/api/estop", methods=["POST"])
def api_estop():
    hw.estop()
    return jsonify(ok=True)


@app.route("/api/estop_reset", methods=["POST"])
def api_estop_reset():
    hw.estop_reset()
    return jsonify(ok=True)


# ------------------------------------------------------------------
PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>AEGIS AGV — Operator Console</title>
<style>
:root{--panel:#14181d;--panel-2:#1b2129;--line:#2c3540;--ink:#d7dde3;
--ink-dim:#7d8894;--amber:#ffb020;--green:#3ddc84;--red:#ff3b30;
--blue:#4aa3ff;--hazard:#f5c518;}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
html,body{height:100%;}
body{background:repeating-linear-gradient(45deg,transparent 0 22px,
rgba(255,255,255,.012) 22px 44px),#0b0e11;color:var(--ink);
font-family:"DIN Alternate","Bahnschrift","Segoe UI",system-ui,sans-serif;
user-select:none;-webkit-user-select:none;display:flex;flex-direction:column;min-height:100%;}
header{display:flex;align-items:center;gap:12px;padding:10px 16px;
background:var(--panel);border-bottom:2px solid var(--line);}
.badge{font-family:"Courier New",monospace;font-weight:bold;letter-spacing:.18em;
font-size:14px;padding:4px 10px;border:1px solid var(--line);background:#0d1013;color:var(--hazard);}
header h1{font-size:13px;letter-spacing:.3em;font-weight:600;color:var(--ink-dim);text-transform:uppercase;}
.spacer{flex:1;}
.lamp{display:flex;align-items:center;gap:6px;font-size:11px;letter-spacing:.1em;color:var(--ink-dim);}
.dot{width:10px;height:10px;border-radius:50%;background:#333;box-shadow:0 0 0 2px #0008 inset;}
.dot.on-green{background:var(--green);box-shadow:0 0 8px var(--green);}
.dot.on-amber{background:var(--amber);box-shadow:0 0 8px var(--amber);}
.dot.on-red{background:var(--red);box-shadow:0 0 8px var(--red);animation:blink .6s steps(1) infinite;}
@keyframes blink{50%{opacity:.25;}}
main{flex:1;display:grid;gap:14px;padding:14px;grid-template-columns:1fr;
grid-template-areas:"drive" "steer" "grip" "estop";max-width:1000px;width:100%;margin:0 auto;}
@media(min-width:820px){main{grid-template-columns:1fr 1fr 1fr;
grid-template-areas:"drive steer grip" "estop estop estop";}}
.module{background:var(--panel-2);border:1px solid var(--line);border-radius:6px;
padding:14px;display:flex;flex-direction:column;gap:11px;
box-shadow:0 2px 0 #000 inset,0 1px 0 rgba(255,255,255,.03);}
.module h2{font-size:11px;letter-spacing:.25em;color:var(--ink-dim);text-transform:uppercase;
font-weight:600;border-bottom:1px solid var(--line);padding-bottom:8px;
display:flex;justify-content:space-between;align-items:center;}
.module h2 .tag{font-family:"Courier New",monospace;font-size:10px;color:#556;letter-spacing:0;}
#m-drive{grid-area:drive;}#m-steer{grid-area:steer;}#m-grip{grid-area:grip;}
#m-estop{grid-area:estop;background:none;border:none;box-shadow:none;padding:0;}
.readout{font-family:"Courier New",monospace;text-align:center;background:#0a0d10;
border:1px solid var(--line);border-radius:4px;padding:10px 6px;}
.readout .big{font-size:40px;font-weight:bold;color:var(--green);line-height:1;}
.readout .unit{font-size:14px;color:var(--ink-dim);}
.readout .sub{font-size:11px;color:var(--ink-dim);margin-top:4px;letter-spacing:.1em;}
.readout.warn .big{color:var(--amber);}
.dir-row{display:flex;gap:8px;}
.dir-btn{flex:1;padding:14px 0;font-size:13px;letter-spacing:.2em;font-weight:700;
background:#10151a;color:var(--ink-dim);border:1px solid var(--line);border-radius:4px;
cursor:pointer;text-transform:uppercase;}
.dir-btn.active{background:#12314f;color:var(--blue);border-color:var(--blue);
box-shadow:0 0 10px rgba(74,163,255,.25);}
.slider-wrap{display:flex;flex-direction:column;gap:6px;}
.slider-labels{display:flex;justify-content:space-between;font-size:10px;
font-family:"Courier New",monospace;color:var(--ink-dim);}
input[type=range]{width:100%;height:44px;appearance:none;background:transparent;cursor:pointer;}
input[type=range]::-webkit-slider-runnable-track{height:14px;border-radius:7px;
background:linear-gradient(90deg,var(--green),var(--amber) 60%,var(--red));border:1px solid #000;}
input[type=range]::-webkit-slider-thumb{appearance:none;width:34px;height:34px;margin-top:-11px;
border-radius:6px;background:linear-gradient(#e8edf2,#9aa5af);border:2px solid #000;
box-shadow:0 2px 4px #000a;}
.row{display:flex;gap:8px;align-items:stretch;}
.num-in{flex:1;min-width:0;font-family:"Courier New",monospace;font-size:18px;font-weight:bold;
text-align:center;color:var(--green);background:#0a0d10;border:1px solid var(--line);
border-radius:4px;padding:9px 6px;outline:none;}
.num-in:focus{border-color:var(--blue);box-shadow:0 0 8px rgba(74,163,255,.25);}
.num-in::placeholder{color:#3a444e;}
.num-in::-webkit-outer-spin-button,.num-in::-webkit-inner-spin-button{-webkit-appearance:none;margin:0;}
.num-in[type=number]{-moz-appearance:textfield;}
.unit{display:flex;align-items:center;font-family:"Courier New",monospace;
color:var(--ink-dim);font-size:14px;font-weight:bold;}
.mini-btn{min-width:70px;font-size:11px;letter-spacing:.12em;font-weight:700;
background:#10151a;color:var(--ink);border:1px solid var(--line);border-radius:4px;
cursor:pointer;text-transform:uppercase;padding:9px 8px;}
.mini-btn:active{background:#3d3010;color:var(--amber);border-color:var(--amber);}
.mini-btn.wide{flex:1;}
.mini-btn.on{background:#12314f;color:var(--blue);border-color:var(--blue);}
.stop-btn{padding:11px 0;font-size:13px;font-weight:700;letter-spacing:.25em;
background:#3a2020;color:#ff9a94;border:1px solid #5c2b2b;border-radius:4px;
cursor:pointer;text-transform:uppercase;}
.brake-bar{display:flex;align-items:center;gap:8px;background:#0a0d10;
border:1px solid var(--line);border-radius:4px;padding:8px 10px;}
.brake-bar .label{font-size:10px;letter-spacing:.2em;color:var(--ink-dim);text-transform:uppercase;}
.brake-bar .val{font-family:"Courier New",monospace;font-weight:bold;font-size:13px;}
.brake-bar .val.applied{color:var(--red);}
.brake-bar .val.released{color:var(--green);}
.hold-col{display:flex;gap:10px;}
.hold-btn{flex:1;min-height:60px;font-size:13px;font-weight:700;letter-spacing:.15em;
background:#10151a;color:var(--ink);border:1px solid var(--line);border-radius:4px;
cursor:pointer;text-transform:uppercase;display:flex;align-items:center;justify-content:center;
gap:6px;touch-action:none;}
.hold-btn.engaged{background:#3d3010;color:var(--amber);border-color:var(--amber);
box-shadow:0 0 12px rgba(255,176,32,.3);}
.hint{font-size:10px;color:var(--ink-dim);text-align:center;letter-spacing:.08em;line-height:1.4;}
.fault{font-size:11px;color:var(--red);text-align:center;font-weight:700;line-height:1.4;}
.estop-row{display:flex;gap:14px;align-items:stretch;}
#estopBtn{flex:1;min-height:88px;border-radius:8px;cursor:pointer;font-size:21px;
font-weight:900;letter-spacing:.3em;color:#fff;
background:radial-gradient(circle at 50% 30%,#ff5147,#b3140c 70%);border:3px solid #7a0c06;
box-shadow:0 6px 0 #6b0a05,0 8px 16px #000c;text-transform:uppercase;}
#estopBtn:active{transform:translateY(4px);box-shadow:0 2px 0 #6b0a05;}
#resetBtn{width:120px;border-radius:8px;cursor:pointer;font-size:12px;font-weight:700;
letter-spacing:.15em;color:var(--ink-dim);background:#10151a;border:1px solid var(--line);
text-transform:uppercase;}
#resetBtn.armed{color:var(--green);border-color:var(--green);}
.hazard-strip{height:8px;margin:0 0 10px;border-radius:2px;
background:repeating-linear-gradient(45deg,var(--hazard) 0 14px,#111 14px 28px);opacity:.85;}
#banner{position:fixed;left:0;right:0;top:0;padding:10px;text-align:center;font-weight:700;
letter-spacing:.2em;font-size:13px;display:none;z-index:50;text-transform:uppercase;}
#banner.estop{display:block;background:var(--red);color:#fff;}
#banner.watchdog{display:block;background:var(--amber);color:#000;}
#banner.offline{display:block;background:#444;color:#fff;}
footer{padding:8px 16px;font-size:10px;color:#4a545e;text-align:center;
font-family:"Courier New",monospace;letter-spacing:.08em;}
</style>
</head>
<body>
<div id="banner"></div>
<header>
  <div class="badge">AEGIS</div>
  <h1>AGV Operator Console</h1>
  <div class="spacer"></div>
  <div class="lamp"><div class="dot" id="brkDot"></div><span>BRK</span></div>
  <div class="lamp"><div class="dot" id="encDot"></div><span>ENC</span></div>
  <div class="lamp"><div class="dot" id="linkDot"></div><span>LINK</span></div>
  <div class="lamp"><div class="dot" id="sysDot"></div><span id="sysTxt">SYS</span></div>
</header>

<main>
  <section class="module" id="m-drive">
    <h2>Drive <span class="tag">DAC0 · IN1 GPIO5</span></h2>
    <div class="readout">
      <span class="big" id="spd">0</span><span class="unit"> %</span>
      <div class="sub"><span id="volts">0.80</span> V · <span id="dirTxt">FORWARD</span></div>
    </div>
    <div class="dir-row">
      <button class="dir-btn active" id="btnFwd">▲ Forward</button>
      <button class="dir-btn" id="btnRev">▼ Reverse</button>
    </div>
    <div class="slider-wrap">
      <div class="slider-labels"><span>IDLE 0.8V</span><span>SPEED</span><span>MAX 4.2V</span></div>
      <input type="range" id="speed" min="0" max="100" value="0" step="1">
    </div>
    <div class="row">
      <input type="number" class="num-in" id="voltIn" min="0.8" max="4.2" step="0.01"
             placeholder="0.80" inputmode="decimal">
      <span class="unit">V</span>
      <button class="mini-btn" id="btnSetVolt">Set V</button>
    </div>
    <div class="hint" id="voltHint">Throttle 0.80 – 4.20 V · starts moving ~1.50 fwd / 1.28 rev</div>
    <button class="stop-btn" id="btnDriveStop">Stop drive</button>

    <div class="brake-bar">
      <span class="label">Brake</span>
      <span class="val" id="brkVal">--</span>
      <span class="spacer"></span>
      <span class="label" id="brkMode">auto</span>
    </div>
    <div class="row">
      <button class="mini-btn wide" id="brkAuto">Auto</button>
      <button class="mini-btn wide" id="brkApply">Apply</button>
      <button class="mini-btn wide" id="brkRelease">Release</button>
    </div>
    <div class="hint" id="brkHint">IN2 GPIO6 · pulses on stop, auto-releases after 2 s</div>
  </section>

  <section class="module" id="m-steer">
    <h2>Steering <span class="tag">BTS#2 · GPIO19/16 · AS5600</span></h2>
    <div class="readout" id="angleReadout">
      <span class="big" id="angle">--</span><span class="unit"> °</span>
      <div class="sub" id="angleSub">encoder</div>
    </div>
    <div class="row">
      <input type="number" class="num-in" id="targetDeg" min="-45" max="45" step="0.5"
             placeholder="0.0" inputmode="decimal">
      <span class="unit">°</span>
      <button class="mini-btn" id="btnGoTo">Go to</button>
    </div>
    <div class="row">
      <button class="mini-btn wide" id="btnCenter">Center 0°</button>
      <button class="mini-btn wide" id="btnCancel">Cancel</button>
    </div>
    <div class="hold-col">
      <button class="hold-btn" data-group="steer" data-cmd="left">◀ Left</button>
      <button class="hold-btn" data-group="steer" data-cmd="right">Right ▶</button>
    </div>
    <div class="row">
      <input type="number" class="num-in" id="steerSecs" min="0.05" max="15" step="0.1"
             placeholder="1.0" inputmode="decimal">
      <span class="unit">s</span>
      <button class="mini-btn" id="secLeft">◀ Run</button>
      <button class="mini-btn" id="secRight">Run ▶</button>
    </div>
    <div class="hint" id="steerHint">Closed loop on encoder · limit ±45°</div>
    <div class="fault" id="steerFault"></div>
  </section>

  <section class="module" id="m-grip">
    <h2>Gripper <span class="tag">BTS#1 · GPIO13/12</span></h2>
    <div class="hold-col">
      <button class="hold-btn" data-group="grip" data-cmd="open">⊃⊂ Open</button>
      <button class="hold-btn" data-group="grip" data-cmd="close">⊂⊃ Close</button>
    </div>
    <div class="row">
      <input type="number" class="num-in" id="gripSecs" min="0.05" max="15" step="0.1"
             placeholder="1.0" inputmode="decimal">
      <span class="unit">s</span>
      <button class="mini-btn" id="gripOpenT">Open</button>
      <button class="mini-btn" id="gripCloseT">Close</button>
    </div>
    <div class="hint" id="gripHint">Hold to move jaws · or timed run</div>
  </section>

  <section class="module" id="m-estop">
    <div class="hazard-strip"></div>
    <div class="estop-row">
      <button id="estopBtn">Emergency stop</button>
      <button id="resetBtn">Reset<br>E-stop</button>
    </div>
  </section>
</main>

<footer>PROJECT AEGIS · REV 4 · AS5600 CLOSED-LOOP STEERING · BRAKE ON IN2 · WATCHDOG 2.0 S</footer>

<script>
const $=id=>document.getElementById(id);
let estopped=false, online=false;
const V_IDLE=0.8, V_MAX=4.2, DEG_MAX=45;

async function post(u,b){try{
  const r=await fetch(u,{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify(b||{})});return await r.json();
}catch(e){return{ok:false,msg:"offline"};}}

async function heartbeat(){
  const r=await post("/api/heartbeat");
  const was=online; online=!!r.ok;
  if(r.watchdog_tripped) banner("watchdog","Watchdog tripped — AGV auto-stopped");
  if(!online&&was) banner("offline","Link lost — AGV will auto-stop");
  lamps();
}
async function poll(){
  try{
    const s=await (await fetch("/api/status")).json();
    $("spd").textContent=Math.round(s.speed_pct);
    $("volts").textContent=s.throttle_v.toFixed(2);
    $("dirTxt").textContent=s.direction.toUpperCase();
    estopped=s.estopped;
    $("btnFwd").classList.toggle("active",s.direction==="forward");
    $("btnRev").classList.toggle("active",s.direction==="reverse");
    $("resetBtn").classList.toggle("armed",estopped);

    $("brkVal").textContent=s.brake.toUpperCase();
    $("brkVal").className="val "+s.brake;
    $("brkMode").textContent=s.brake_mode;
    $("brkDot").className="dot "+(s.brake==="applied"?"on-red":"on-green");
    $("brkAuto").classList.toggle("on",s.brake_mode==="auto");

    if(s.angle_deg===null||s.angle_deg===undefined){
      $("angle").textContent="--"; $("angleSub").textContent="no encoder";
    }else{
      $("angle").textContent=(s.angle_deg>0?"+":"")+s.angle_deg.toFixed(1);
      $("angleSub").textContent = (s.seek_target!==null&&s.seek_target!==undefined)
        ? "seeking "+(s.seek_target>0?"+":"")+s.seek_target.toFixed(1)+"°"
        : (s.steer==="stop"?"holding":"moving "+s.steer);
    }
    $("angleReadout").classList.toggle("warn",
      s.seek_target!==null&&s.seek_target!==undefined);
    $("steerFault").textContent=s.steer_fault||"";
    $("encDot").className="dot "+(s.encoder_ok?"on-green":"on-red");

    document.querySelectorAll(".hold-btn").forEach(b=>{
      const st=b.dataset.group==="steer"?s.steer:s.grip;
      b.classList.toggle("engaged",st===b.dataset.cmd);
    });
    if(estopped) banner("estop","E-STOP ACTIVE — reset to resume");
    else if($("banner").className==="estop") clearBanner();
    lamps();
  }catch(e){}
}
function lamps(){
  $("linkDot").className="dot "+(online?"on-green":"on-red");
  $("sysDot").className="dot "+(estopped?"on-red":"on-amber");
  $("sysTxt").textContent=estopped?"ESTOP":"SYS";
}
function banner(c,t){const b=$("banner");b.className=c;b.textContent=t;}
function clearBanner(){const b=$("banner");b.className="";b.textContent="";}
setInterval(heartbeat,500); setInterval(poll,300); heartbeat(); poll();

function flash(id,msg,ok,orig){
  const h=$(id); h.textContent=msg;
  h.style.color=ok?"var(--green)":"var(--amber)";
  setTimeout(()=>{h.textContent=orig;h.style.color="";},2600);
}
const VOLT_ORIG="Throttle 0.80 – 4.20 V · starts moving ~1.50 fwd / 1.28 rev";
const STEER_ORIG="Closed loop on encoder · limit ±45°";
const GRIP_ORIG="Hold to move jaws · or timed run";
const BRK_ORIG="IN2 GPIO6 · pulses on stop, auto-releases after 2 s";

/* drive */
$("btnFwd").onclick=async()=>{const r=await post("/api/direction",{dir:"forward"});
  if(!r.ok) flash("voltHint",r.msg,false,VOLT_ORIG);};
$("btnRev").onclick=async()=>{const r=await post("/api/direction",{dir:"reverse"});
  if(!r.ok) flash("voltHint",r.msg,false,VOLT_ORIG);};
let st=null;
$("speed").addEventListener("input",()=>{clearTimeout(st);
  st=setTimeout(()=>post("/api/speed",{pct:+$("speed").value}),80);});
$("btnDriveStop").onclick=()=>{$("speed").value=0;$("voltIn").value="";
  post("/api/speed",{pct:0});};
async function applyVoltage(){
  const v=parseFloat($("voltIn").value);
  if(isNaN(v)){flash("voltHint","Enter 0.80 – 4.20",false,VOLT_ORIG);return;}
  if(v<V_IDLE||v>V_MAX){flash("voltHint","Out of range 0.80 – 4.20",false,VOLT_ORIG);return;}
  const r=await post("/api/voltage",{volts:v});
  if(r.ok){flash("voltHint","Throttle → "+r.msg,true,VOLT_ORIG);
    $("speed").value=Math.round((v-V_IDLE)/(V_MAX-V_IDLE)*100);}
  else flash("voltHint",r.msg||"Rejected",false,VOLT_ORIG);
}
$("btnSetVolt").onclick=applyVoltage;
$("voltIn").addEventListener("keydown",e=>{
  if(e.key==="Enter"){e.preventDefault();applyVoltage();}});

/* brake */
async function brake(mode){
  const r=await post("/api/brake",{mode});
  flash("brkHint", r.ok?r.msg:(r.msg||"Rejected"), r.ok, BRK_ORIG);
}
$("brkAuto").onclick=()=>brake("auto");
$("brkApply").onclick=()=>brake("apply");
$("brkRelease").onclick=()=>brake("release");

/* steering */
async function goTo(deg){
  if(isNaN(deg)){flash("steerHint","Enter an angle",false,STEER_ORIG);return;}
  if(Math.abs(deg)>DEG_MAX){flash("steerHint","Limit ±"+DEG_MAX+"°",false,STEER_ORIG);return;}
  const r=await post("/api/steer_to",{deg});
  flash("steerHint", r.ok?("Seeking "+deg.toFixed(1)+"°"):(r.msg||"Rejected"), r.ok, STEER_ORIG);
}
$("btnGoTo").onclick=()=>goTo(parseFloat($("targetDeg").value));
$("targetDeg").addEventListener("keydown",e=>{
  if(e.key==="Enter"){e.preventDefault();goTo(parseFloat($("targetDeg").value));}});
$("btnCenter").onclick=()=>goTo(0);
$("btnCancel").onclick=async()=>{await post("/api/steer_cancel");
  await post("/api/clear_fault"); flash("steerHint","Cancelled",true,STEER_ORIG);};

async function steerTimed(dir){
  const s=parseFloat($("steerSecs").value);
  if(isNaN(s)||s<=0){flash("steerHint","Enter seconds > 0",false,STEER_ORIG);return;}
  const r=await post("/api/steer_timed",{dir,secs:s});
  flash("steerHint", r.ok?("Steering "+r.msg):(r.msg||"Rejected"), r.ok, STEER_ORIG);
}
$("secLeft").onclick=()=>steerTimed("left");
$("secRight").onclick=()=>steerTimed("right");

async function gripTimed(cmd){
  const s=parseFloat($("gripSecs").value);
  if(isNaN(s)||s<=0){flash("gripHint","Enter seconds > 0",false,GRIP_ORIG);return;}
  const r=await post("/api/grip_timed",{cmd,secs:s});
  flash("gripHint", r.ok?("Gripper "+r.msg):(r.msg||"Rejected"), r.ok, GRIP_ORIG);
}
$("gripOpenT").onclick=()=>gripTimed("open");
$("gripCloseT").onclick=()=>gripTimed("close");

/* hold-to-move */
document.querySelectorAll(".hold-btn").forEach(btn=>{
  const g=btn.dataset.group,c=btn.dataset.cmd;
  const start=e=>{e.preventDefault();btn.classList.add("engaged");post(`/api/${g}`,{cmd:c});};
  const stop=()=>{if(!btn.classList.contains("engaged"))return;
    btn.classList.remove("engaged");post(`/api/${g}`,{cmd:"stop"});};
  btn.addEventListener("pointerdown",start);
  btn.addEventListener("pointerup",stop);
  btn.addEventListener("pointerleave",stop);
  btn.addEventListener("pointercancel",stop);
  btn.addEventListener("contextmenu",e=>e.preventDefault());
});
window.addEventListener("pointerup",()=>{
  document.querySelectorAll(".hold-btn.engaged").forEach(b=>{
    b.classList.remove("engaged");post(`/api/${b.dataset.group}`,{cmd:"stop"});});
});

/* e-stop */
$("estopBtn").onclick=()=>{$("speed").value=0;$("voltIn").value="";post("/api/estop");};
$("resetBtn").onclick=()=>{post("/api/estop_reset");clearBanner();};
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return Response(PAGE, mimetype="text/html")


if __name__ == "__main__":
    print("=" * 64)
    print("  PROJECT AEGIS — AGV OPERATOR CONSOLE  [REV 4]")
    print(f"  Mode          : {'MOCK (no hardware)' if MOCK else 'LIVE HARDWARE'}")
    print(f"  Steering      : BTS#2 GPIO{STEER_RPWM}/{STEER_LPWM} "
          f"(active {'LOW' if STEER_ACTIVE_LOW else 'HIGH'})")
    print(f"  Gripper       : BTS#1 GPIO{GRIP_RPWM}/{GRIP_LPWM} "
          f"(active {'LOW' if GRIP_ACTIVE_LOW else 'HIGH'})")
    print(f"  Direction IN1 : GPIO{RELAY_DIR_PIN}  "
          f"{'LOW' if FORWARD_RELAY_LEVEL == 0 else 'HIGH'} = FORWARD")
    print(f"  Brake     IN2 : GPIO{BRAKE_PIN}  "
          f"{'LOW' if BRAKE_ENGAGE_LEVEL == 0 else 'HIGH'} = APPLIED"
          f"{'' if BRAKE_ENABLE else '  [DISABLED]'}")
    print(f"  AS5600 centre : raw {AS5600_CENTER_RAW}")
    print(f"  Open http://<pi-ip>:{HTTP_PORT}   (hostname -I)")
    print("=" * 64)
    app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True)
