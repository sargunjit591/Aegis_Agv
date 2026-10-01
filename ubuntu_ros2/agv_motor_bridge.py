#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — AGV MOTOR BRIDGE (ROS2 node)  [REV 4]
============================================================
Subscribes /cmd_vel and drives the AGV.

  linear.x  -> direction relay + drive throttle (fixed voltages)
  angular.z -> steering motor (BTS7960 #2)

HARDWARE MAP
  Steering : BTS7960 #2  GPIO19=RPWM(right) GPIO16=LPWM(left)  active HIGH
  Gripper  : BTS7960 #1  GPIO13=RPWM(open)  GPIO12=LPWM(close) active LOW
  Dir relay: relay IN1   GPIO5    LOW = FORWARD, HIGH = REVERSE
  BRAKE    : relay IN2   GPIO6    24 V self-opening actuator;
                                  the relay shorts its two wires
  Throttle : Waveshare HAT DAC0, SPI(0,1) mode 1, CS=GPIO23
             0.80 V idle | 1.50 V forward | 1.28 V reverse | 4.20 V max
  Steering feedback : AS5600, I2C bus 1 @ 0x36, centre raw 3746

BRAKE BEHAVIOUR — PULSED, NEVER HELD
  The actuator is self-opening, so it must not sit energised. The
  brake therefore fires as a short PULSE and always lets go:
    * RELEASED at startup and at shutdown
    * one pulse when the drive stops (moving -> stopped)
    * one pulse on E-stop
    * every engagement auto-releases after brake_max_engage_s
      (default 2.0 s) no matter how it was triggered, including a
      manual APPLY
    * a drive command releases it immediately
    * will not re-fire until the AGV has moved again
  Because of this, the brake is a stopping aid, NOT a parking
  brake — a parked AGV is held by the drive only. Chock the wheels
  if it must stay put on a slope.

  !! VERIFY BRAKE POLARITY ON STANDS BEFORE DRIVING !!
  Which GPIO level means "brake applied" depends on the relay
  module polarity AND on how the actuator responds to having its
  wires shorted. It is the brake_engage_level parameter:
      0 = LOW applies the brake   (default, active-LOW relay)
      1 = HIGH applies the brake
  Test: wheels off the ground, no throttle, toggle it and watch
  which state locks the wheel.

TOPICS
  /cmd_vel          geometry_msgs/Twist   in
  /agv/estop        std_msgs/Bool         in
  /agv/gripper_cmd  std_msgs/String       in   "open"|"close"|"stop"
  /agv/brake        std_msgs/String       in   "apply"|"release"|"auto"
  /agv/status       std_msgs/String       out  JSON at 5 Hz

GPIOCHIP
  The 40-pin header is gpiochip0 on Raspberry Pi OS but gpiochip4
  on Ubuntu 24.04. Opening the wrong one succeeds silently and no
  pin ever moves, so the chip is detected by label. Override with
  -p gpiochip:=<n>.
============================================================
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

try:
    import lgpio
    import spidev
    MOCK = False
except ImportError:
    MOCK = True

try:
    import smbus2
    HAVE_SMBUS = True
except ImportError:
    HAVE_SMBUS = False


class AGVMotorBridge(Node):

    def __init__(self):
        super().__init__('agv_motor_bridge')
        p = self.declare_parameter

        # ---- chip ----
        p('gpiochip', -1)                 # -1 = auto-detect by label

        # ---- steering (BTS7960 #2) ----
        p('steer_rpwm_pin', 19)
        p('steer_lpwm_pin', 16)
        p('steer_active_low', False)
        p('invert_steering', False)

        # ---- gripper (BTS7960 #1) ----
        p('grip_rpwm_pin', 13)            # open
        p('grip_lpwm_pin', 12)            # close
        p('grip_active_low', True)
        p('gripper_max_run_s', 8.0)

        # ---- drive ----
        p('relay_dir_pin', 5)             # relay IN1
        p('invert_direction', False)      # False => GPIO5 LOW = FORWARD
        p('dac_cs_pin', 23)
        p('dac_vref', 5.0)
        p('throttle_idle_v', 0.80)
        p('throttle_max_v', 4.20)
        p('drive_volts_forward', 1.50)    # measured start threshold
        p('drive_volts_reverse', 1.28)    # measured start threshold
        p('fixed_speed', True)
        p('max_linear_mps', 1.0)
        p('linear_deadband', 0.02)
        p('slew_step_v', 0.15)
        p('control_rate_hz', 50.0)
        p('watchdog_timeout_s', 0.5)

        # ---- brake (relay IN2) ----
        p('brake_pin', 6)
        p('brake_engage_level', 0)        # 0 = LOW applies the brake
        p('brake_enable', True)
        p('brake_release_lead_s', 0.35)   # brake lets go before throttle rises
        p('brake_max_engage_s', 2.0)      # HARD LIMIT: auto-release after this
                                          # long. The actuator is self-opening
                                          # and must not be held energised.

        # ---- steering control ----
        p('angular_deadband', 0.1)
        p('closed_loop_steering', False)
        p('wheelbase_m', 0.9)
        p('steer_max_deg', 45.0)
        p('steer_deadband_deg', 2.0)
        p('steer_max_run_s', 4.0)
        p('steer_stop_lead_s', 0.12)      # stop this far AHEAD of the limit,
                                          # scaled by the measured turn rate,
                                          # to cancel latency + coast
        p('steer_rate_alpha', 0.35)       # smoothing on the rate estimate         # backstop: never run the steering
                                          # motor continuously longer than this

        # ---- AS5600 ----
        p('as5600_bus', 1)
        p('as5600_center_raw', 3746)      # only used if unwrapping is OFF
        p('as5600_unwrap', True)          # multi-turn unwrapping (see below)
        p('as5600_sample_hz', 100.0)      # must be fast enough that the shaft
                                          # never moves >180 deg between reads
        p('steer_gear_ratio', 1.0)        # encoder deg per WHEEL deg —
                                          # measure with as5600_calibrate.py
        p('as5600_ccw_positive', True)
        p('runaway_ticks', 10)

        g = lambda n: self.get_parameter(n).value
        self.STEER_R, self.STEER_L = g('steer_rpwm_pin'), g('steer_lpwm_pin')
        self.STEER_AL = bool(g('steer_active_low'))
        self.INV_STEER = bool(g('invert_steering'))
        self.GRIP_R, self.GRIP_L = g('grip_rpwm_pin'), g('grip_lpwm_pin')
        self.GRIP_AL = bool(g('grip_active_low'))
        self.GRIP_MAX_S = float(g('gripper_max_run_s'))
        self.RELAY = int(g('relay_dir_pin'))
        self.INV_DIR = bool(g('invert_direction'))
        self.DAC_CS = int(g('dac_cs_pin'))
        self.VREF = float(g('dac_vref'))
        self.V_IDLE = float(g('throttle_idle_v'))
        self.V_MAX = float(g('throttle_max_v'))
        self.DRIVE_FWD = max(self.V_IDLE, min(self.V_MAX, float(g('drive_volts_forward'))))
        self.DRIVE_REV = max(self.V_IDLE, min(self.V_MAX, float(g('drive_volts_reverse'))))
        self.FIXED = bool(g('fixed_speed'))
        self.MAX_LIN = float(g('max_linear_mps'))
        self.LIN_DB = float(g('linear_deadband'))
        self.SLEW = float(g('slew_step_v'))
        self.RATE = float(g('control_rate_hz'))
        self.WDT = float(g('watchdog_timeout_s'))
        self.BRAKE_PIN = int(g('brake_pin'))
        self.BRAKE_ON = 1 if int(g('brake_engage_level')) else 0
        self.BRAKE_OFF = 1 - self.BRAKE_ON
        self.BRAKE_EN = bool(g('brake_enable'))
        self.BRAKE_LEAD = float(g('brake_release_lead_s'))
        self.BRAKE_MAX_S = float(g('brake_max_engage_s'))
        self.ANG_DB = float(g('angular_deadband'))
        self.CLOSED_LOOP = bool(g('closed_loop_steering'))
        self.WHEELBASE = float(g('wheelbase_m'))
        self.STEER_MAX = math.radians(float(g('steer_max_deg')))
        self.STEER_DB = math.radians(float(g('steer_deadband_deg')))
        self.STEER_MAX_RUN = float(g('steer_max_run_s'))
        self.STEER_LEAD = float(g('steer_stop_lead_s'))
        self.RATE_ALPHA = float(g('steer_rate_alpha'))
        self.CENTER_RAW = int(g('as5600_center_raw'))
        self.UNWRAP = bool(g('as5600_unwrap'))
        self.ENC_HZ = float(g('as5600_sample_hz'))
        self.GEAR = float(g('steer_gear_ratio'))
        if self.GEAR <= 0.0:
            self.GEAR = 1.0
        self.CCW_POS = bool(g('as5600_ccw_positive'))
        self.RUNAWAY_N = int(g('runaway_ticks'))

        # ---- state ----
        self.lock = threading.Lock()
        self.estopped = False
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.last_cmd_time = 0.0
        self.current_v = self.V_IDLE
        self.target_v = self.V_IDLE
        self.direction = 'forward'
        self.want_dir = 'forward'
        self.steer_state = 'stop'
        self.grip_state = 'stop'
        self.grip_started = 0.0
        self.steer_angle = None
        self.steer_target = 0.0
        self.enc_accum_deg = 0.0      # continuous multi-turn encoder angle
        self.enc_last_deg = None
        self.enc_home_deg = None      # accum value at wheel-straight
        self.enc_max_step = 0.0
        self._last_raw = None
        self.enc_lost_count = 0
        self.homed = False
        self.steer_fault = None
        self._prev_abs_err = None
        self._worsening = 0
        self._prev_abs_deg = None
        self._steer_away_cmd = None     # learned at runtime
        self._steer_cmd_since = None
        self.steer_rate_dps = 0.0       # measured wheel turn rate, deg/s
        self._rate_prev_deg = None
        self._rate_prev_t = None
        self.brake_engaged = False        # starts RELEASED
        self.brake_released_at = time.monotonic()
        self.brake_engaged_at = None      # when the current pulse started
        self.brake_pulse_spent = False    # pulse used; wait for motion to re-arm
        self.brake_was_moving = False
        self.brake_override = None        # None=auto, True=apply, False=release

        # ---- hardware ----
        self.h = None
        self.spi = None
        self.as5600 = None
        if not MOCK:
            chip = self._find_header_chip(g('gpiochip'))
            self.h = lgpio.gpiochip_open(chip)
            self._claim_all()
            self.spi = spidev.SpiDev()
            self.spi.open(0, 1)
            self.spi.max_speed_hz = 20000000
            self.spi.mode = 0b01
            self._write_dac(self.V_IDLE)
        else:
            self.get_logger().warn('MOCK MODE — hardware calls printed only')

        if HAVE_SMBUS and not MOCK:
            try:
                self.as5600 = smbus2.SMBus(int(g('as5600_bus')))
                a = self._read_as5600()
                self.get_logger().info(
                    f'AS5600 OK — centre raw {self.CENTER_RAW}, '
                    f'now {math.degrees(a):+.1f} deg')
            except Exception as e:
                self.as5600 = None
                self.get_logger().error(f'AS5600 unavailable: {e}')

        if self.CLOSED_LOOP and self.as5600 is None:
            self.get_logger().error('closed loop requested without AS5600 — open loop')
            self.CLOSED_LOOP = False

        # ---- ROS ----
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Bool, 'agv/estop', self.on_estop, 10)
        self.create_subscription(String, 'agv/gripper_cmd', self.on_gripper, 10)
        self.create_subscription(String, 'agv/brake', self.on_brake_cmd, 10)
        self.create_subscription(String, 'agv/steer_home', self.on_home_cmd, 10)
        self.status_pub = self.create_publisher(String, 'agv/status', 10)
        if self.as5600 is not None and self.UNWRAP:
            self.create_timer(1.0 / self.ENC_HZ, self._enc_sample)
            # assume the wheel is straight at startup; re-home any time with
            #   ros2 topic pub --once /agv/steer_home std_msgs/msg/String "{data: here}"
            self.enc_home_deg = 0.0
            self.homed = True
            self.get_logger().warn(
                'steering homed to the CURRENT wheel position — make sure the '
                'wheel is STRAIGHT at startup, or re-home via /agv/steer_home')
        self.create_timer(1.0 / self.RATE, self.control_tick)
        # 20 Hz, not 5: the steering moves ~100 deg/s, so at 5 Hz a consumer
        # sees 20 deg jumps and cannot catch a few-degree window.
        self.create_timer(0.05, self.publish_status)

        mode = 'CLOSED-loop' if self.CLOSED_LOOP else 'OPEN-loop'
        spd = (f'fwd {self.DRIVE_FWD:.2f} V / rev {self.DRIVE_REV:.2f} V'
               if self.FIXED else f'proportional to {self.V_MAX:.2f} V')
        fwd = 'HIGH' if self.INV_DIR else 'LOW'
        brk = (f'GPIO{self.BRAKE_PIN} {"LOW" if self.BRAKE_ON == 0 else "HIGH"}=APPLIED'
               if self.BRAKE_EN else 'DISABLED')
        enc = (f'unwrap ON, {self.ENC_HZ:.0f} Hz, gear {self.GEAR:.2f} enc-deg/wheel-deg'
               if self.UNWRAP else 'single-turn (legacy)')
        self.get_logger().info(f'steering encoder: {enc}')
        self.get_logger().info(
            f'AGV motor bridge REV4 — steering {mode}, drive {spd}, '
            f'dir relay GPIO{self.RELAY} ({fwd}=forward), brake {brk}, '
            f'watchdog {self.WDT}s' + (' [MOCK]' if MOCK else ''))

    # ================= gpiochip detection =================
    def _find_header_chip(self, requested):
        """The header is gpiochip0 on RPi OS, gpiochip4 on Ubuntu 24.04.
        Opening the wrong chip succeeds silently, so match on label."""
        def probe_lgpio():
            found = {}
            for n in range(8):
                try:
                    h = lgpio.gpiochip_open(n)
                except Exception:
                    continue
                try:
                    info = lgpio.gpio_get_chip_info(h)
                    if isinstance(info, (list, tuple)) and len(info) >= 4:
                        lines, label = info[1], info[3]
                    else:
                        lines = getattr(info, 'lines', 0)
                        label = getattr(info, 'label', '')
                    if isinstance(label, bytes):
                        label = label.decode(errors='ignore')
                    found[n] = (str(label), int(lines))
                except Exception:
                    pass
                finally:
                    try:
                        lgpio.gpiochip_close(h)
                    except Exception:
                        pass
            return found

        def probe_gpiodetect():
            import re
            import subprocess
            found = {}
            try:
                out = subprocess.run(['gpiodetect'], capture_output=True,
                                     text=True, timeout=5).stdout
            except Exception:
                return found
            for line in out.splitlines():
                m = re.match(r'gpiochip(\d+)\s+\[([^\]]+)\]\s+\((\d+) lines\)',
                             line.strip())
                if m:
                    found[int(m.group(1))] = (m.group(2), int(m.group(3)))
            return found

        if requested is not None and int(requested) >= 0:
            n = int(requested)
            self.get_logger().info(f'gpiochip{n} set explicitly')
            return n

        chips = probe_lgpio() or probe_gpiodetect()
        if not chips:
            self.get_logger().error('cannot enumerate gpiochips — using 0')
            return 0
        for n in sorted(chips):
            self.get_logger().info(f'  gpiochip{n}: {chips[n][0]!r} ({chips[n][1]} lines)')
        for n in sorted(chips):
            lb = chips[n][0].lower()
            if 'rp1' in lb or 'bcm2835' in lb or 'bcm2711' in lb:
                self.get_logger().info(f'40-pin header = gpiochip{n} (by label)')
                return n
        for n in sorted(chips):
            if chips[n][1] == 54:
                self.get_logger().info(f'40-pin header = gpiochip{n} (54 lines)')
                return n
        self.get_logger().error('header not identified — using 0; pass -p gpiochip:=<n>')
        return 0

    # ================= hardware =================
    def _claim_all(self):
        rest_s = 1 if self.STEER_AL else 0
        lgpio.gpio_claim_output(self.h, self.STEER_R, rest_s)
        lgpio.gpio_claim_output(self.h, self.STEER_L, rest_s)
        rest_g = 1 if self.GRIP_AL else 0
        lgpio.gpio_claim_output(self.h, self.GRIP_R, rest_g)
        lgpio.gpio_claim_output(self.h, self.GRIP_L, rest_g)
        lgpio.gpio_claim_output(self.h, self.RELAY, self._dir_level('forward'))
        lgpio.gpio_claim_output(self.h, self.DAC_CS, 1)
        if self.BRAKE_EN:
            # boot with the brake RELEASED — the actuator is self-opening and
            # must never sit energised
            lgpio.gpio_claim_output(self.h, self.BRAKE_PIN, self.BRAKE_OFF)

    def _dir_level(self, direction):
        fwd = 1 if self.INV_DIR else 0
        return fwd if direction == 'forward' else (1 - fwd)

    def _pin(self, pin, active, active_low):
        level = (0 if active else 1) if active_low else (1 if active else 0)
        if MOCK:
            print(f'[MOCK] GPIO{pin} -> {level}')
        else:
            lgpio.gpio_write(self.h, pin, level)

    def _write_dac(self, voltage):
        voltage = max(0.0, min(self.VREF, voltage))
        value = int((voltage / self.VREF) * 65535) & 0xFFFF
        if MOCK:
            print(f'[MOCK] DAC0 -> {voltage:.2f} V')
            return
        lgpio.gpio_write(self.h, self.DAC_CS, 0)
        self.spi.xfer2([0x30, (value >> 8) & 0xFF, value & 0xFF])
        lgpio.gpio_write(self.h, self.DAC_CS, 1)

    def _relay(self, direction):
        level = self._dir_level(direction)
        if MOCK:
            print(f'[MOCK] GPIO{self.RELAY} (dir) -> {level} ({direction})')
        else:
            lgpio.gpio_write(self.h, self.RELAY, level)
        self.direction = direction

    def _brake(self, engage):
        """The ONLY place the brake pin is written."""
        if not self.BRAKE_EN or engage == self.brake_engaged:
            return
        level = self.BRAKE_ON if engage else self.BRAKE_OFF
        if MOCK:
            print(f'[MOCK] GPIO{self.BRAKE_PIN} (brake) -> {level} '
                  f'({"APPLY" if engage else "RELEASE"})')
        else:
            lgpio.gpio_write(self.h, self.BRAKE_PIN, level)
        self.brake_engaged = engage
        if engage:
            self.brake_engaged_at = time.monotonic()
        else:
            self.brake_released_at = time.monotonic()
            self.brake_engaged_at = None
        self.get_logger().info(f'brake {"APPLIED" if engage else "RELEASED"}')

    def _steer_hw(self, cmd):
        if self.INV_STEER and cmd in ('left', 'right'):
            cmd = 'left' if cmd == 'right' else 'right'
        self._pin(self.STEER_R, cmd == 'right', self.STEER_AL)
        self._pin(self.STEER_L, cmd == 'left', self.STEER_AL)
        self.steer_state = cmd

    def _grip_hw(self, cmd):
        self._pin(self.GRIP_R, cmd == 'open', self.GRIP_AL)
        self._pin(self.GRIP_L, cmd == 'close', self.GRIP_AL)
        self.grip_state = cmd
        if cmd != 'stop':
            self.grip_started = time.monotonic()

    def _read_raw(self):
        hi = self.as5600.read_byte_data(0x36, 0x0E)
        lo = self.as5600.read_byte_data(0x36, 0x0F)
        raw = ((hi << 8) | lo) & 0x0FFF
        self._last_raw = raw
        return raw

    def _enc_sample(self):
        """Sample the AS5600 and accumulate a CONTINUOUS multi-turn angle.

        The encoder is on the wiper motor shaft, which turns several times
        across the wheel's travel, so a single-turn reading wraps and is
        meaningless on its own. Sampling faster than 180 deg of shaft
        motion per read lets us detect each wrap and keep counting.
        Runs on its own timer at as5600_sample_hz."""
        if self.as5600 is None:
            return
        try:
            deg = self._read_raw() * (360.0 / 4096.0)
        except Exception as e:
            self.get_logger().error(f'AS5600 read failed: {e}',
                                    throttle_duration_sec=5.0)
            return
        with self.lock:
            if self.enc_last_deg is not None:
                d = deg - self.enc_last_deg
                if d > 180.0:
                    d -= 360.0
                elif d < -180.0:
                    d += 360.0
                if abs(d) > self.enc_max_step:
                    self.enc_max_step = abs(d)
                if abs(d) > 150.0:
                    self.enc_lost_count += 1
                    self.get_logger().warn(
                        f'encoder step {abs(d):.0f} deg — sampling too slow, '
                        f'unwrap may have lost count (raise as5600_sample_hz)',
                        throttle_duration_sec=5.0)
                self.enc_accum_deg += d
            self.enc_last_deg = deg

    def _read_as5600(self):
        """Wheel angle in radians, signed, 0 = straight ahead."""
        if self.UNWRAP:
            if self.enc_home_deg is None:
                return None                     # not homed yet
            wheel_deg = (self.enc_accum_deg - self.enc_home_deg) / self.GEAR
            return math.radians(wheel_deg if self.CCW_POS else -wheel_deg)
        # legacy single-turn mode
        raw = self._read_raw()
        diff = (raw - self.CENTER_RAW + 2048) % 4096 - 2048
        ang = diff * (2 * math.pi / 4096.0)
        return ang if self.CCW_POS else -ang

    def home_steering(self):
        """Declare the CURRENT wheel position to be straight ahead.

        Single-turn mode (as5600_unwrap=false): sets CENTER_RAW to the
        raw reading right now. Because the AS5600 is absolute, this
        survives power cycles — you only need to do it when the centre
        is wrong, not at every startup.

        Unwrap mode: zeroes the accumulator instead."""
        try:
            raw = self._read_raw()
        except Exception as e:
            self.get_logger().error(f'cannot home — AS5600 read failed: {e}')
            return
        with self.lock:
            if self.UNWRAP:
                self.enc_home_deg = self.enc_accum_deg
                where = f'encoder {self.enc_accum_deg:+.1f} deg'
            else:
                self.CENTER_RAW = raw
                where = f'raw {raw}'
            self.homed = True
            self.steer_fault = None
            self._prev_abs_err = None
            self._worsening = 0
        self.get_logger().info(
            f'steering CENTRED at {where} — this position is now 0 deg. '
            f'Pass -p as5600_center_raw:={raw} to make it the default.')

    # ================= callbacks =================
    def on_cmd_vel(self, msg: Twist):
        with self.lock:
            self.cmd_lin = float(msg.linear.x)
            self.cmd_ang = float(msg.angular.z)
            self.last_cmd_time = time.monotonic()

    def on_estop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('E-STOP ENGAGED')
            with self.lock:
                self.estopped = True
                self.target_v = self.current_v = self.V_IDLE
                self._write_dac(self.V_IDLE)
                self.brake_override = None
                self.brake_pulse_spent = False
                self._brake(True)          # pulse; auto-releases on the limit
                self._steer_hw('stop')
                self._grip_hw('stop')
                self.cmd_lin = self.cmd_ang = 0.0
        else:
            self.get_logger().info('E-stop released')
            with self.lock:
                self.estopped = False
                self.steer_fault = None
                self._prev_abs_err = None
                self._worsening = 0

    def on_gripper(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in ('open', 'close', 'stop'):
            self.get_logger().warn(f'bad gripper cmd: {msg.data!r}')
            return
        with self.lock:
            if self.estopped and cmd != 'stop':
                return
            self._grip_hw(cmd)

    def on_brake_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in ('apply', 'release', 'auto'):
            self.get_logger().warn(f'bad brake cmd: {msg.data!r}')
            return
        with self.lock:
            if cmd == 'auto':
                self.brake_override = None
                self.get_logger().info('brake -> AUTO')
                return
            if cmd == 'release':
                if self.estopped:
                    self.get_logger().warn('cannot release brake during E-stop')
                    return
                self.brake_override = False
            else:
                self.brake_override = True
            self._brake(self.brake_override)

    def on_home_cmd(self, msg: String):
        if msg.data.strip().lower() in ('here', 'home', 'zero'):
            self.home_steering()
        else:
            self.get_logger().warn(f'bad home cmd: {msg.data!r} (use "here")')

    # ================= control =================
    def control_tick(self):
        with self.lock:
            now = time.monotonic()

            if self.as5600 is not None:
                try:
                    self.steer_angle = self._read_as5600()
                except Exception:
                    self.steer_angle = None
                # estimate how fast the wheel is actually turning, so the
                # limit check can stop early enough to land on target
                if self.steer_angle is not None:
                    d = math.degrees(self.steer_angle)
                    if self._rate_prev_t is not None:
                        dt = now - self._rate_prev_t
                        if dt > 1e-4:
                            raw_rate = (d - self._rate_prev_deg) / dt
                            a = self.RATE_ALPHA
                            self.steer_rate_dps = (a * raw_rate +
                                                   (1.0 - a) * self.steer_rate_dps)
                    self._rate_prev_deg = d
                    self._rate_prev_t = now

            if self.estopped:
                return

            # watchdog
            stale = (now - self.last_cmd_time) > self.WDT
            lin = 0.0 if stale else self.cmd_lin
            ang = 0.0 if stale else self.cmd_ang
            if stale and (self.target_v > self.V_IDLE or self.steer_state != 'stop'):
                self.get_logger().warn('cmd_vel watchdog — stopping',
                                       throttle_duration_sec=2.0)

            moving = abs(lin) > self.LIN_DB
            self.want_dir = 'reverse' if lin < -self.LIN_DB else 'forward'

            # ---- brake: PULSE, never held ----
            # The actuator is self-opening, so every engagement is a short
            # pulse that auto-releases after brake_max_engage_s at the
            # latest. It fires once when the drive stops, then stays off
            # until the AGV moves again.
            if self.BRAKE_EN:
                # 1. hard auto-release — applies to auto, manual and E-stop
                if (self.brake_engaged and self.brake_engaged_at is not None
                        and (now - self.brake_engaged_at) >= self.BRAKE_MAX_S):
                    self._brake(False)
                    self.brake_pulse_spent = True
                    self.brake_override = None      # drop any manual hold
                    self.get_logger().info(
                        f'brake auto-released after {self.BRAKE_MAX_S:.1f}s limit')

                # 2. a drive command always wins — let go immediately
                if moving and self.brake_engaged:
                    self._brake(False)

                # 3. re-arm the pulse once the AGV is actually moving
                if moving:
                    self.brake_pulse_spent = False

                # 4. fire one pulse on the moving -> stopped transition
                if (self.brake_override is None and not moving
                        and self.brake_was_moving
                        and not self.brake_pulse_spent):
                    self._brake(True)

                self.brake_was_moving = moving

            # throttle stays at idle until the brake has actually let go
            released_long_enough = (
                not self.BRAKE_EN
                or (not self.brake_engaged
                    and self.brake_released_at is not None
                    and (now - self.brake_released_at) >= self.BRAKE_LEAD))

            if not moving or not released_long_enough:
                desired_v = self.V_IDLE
            elif self.FIXED:
                desired_v = (self.DRIVE_REV if self.want_dir == 'reverse'
                             else self.DRIVE_FWD)
            else:
                frac = min(abs(lin) / self.MAX_LIN, 1.0)
                desired_v = self.V_IDLE + frac * (self.V_MAX - self.V_IDLE)

            # direction interlock — only switch the relay at idle
            if moving and self.want_dir != self.direction:
                self.target_v = self.V_IDLE
                if abs(self.current_v - self.V_IDLE) < 0.05:
                    self._relay(self.want_dir)
                    self.target_v = desired_v
            else:
                self.target_v = desired_v

            # slew-limited DAC
            delta = self.target_v - self.current_v
            if abs(delta) > 0.005:
                self.current_v += max(-self.SLEW, min(self.SLEW, delta))
                self._write_dac(self.current_v)

            # learn which command drives the wheel AWAY from centre, so the
            # backstop above can block that one without trapping the wheel
            if self.steer_angle is not None:
                ad = abs(math.degrees(self.steer_angle))
                if (self._prev_abs_deg is not None
                        and self.steer_state in ('left', 'right')
                        and ad > self._prev_abs_deg + 0.2):
                    self._steer_away_cmd = self.steer_state
                self._prev_abs_deg = ad

            # steering
            if self.steer_fault:
                self._steer_hw('stop')
            elif self.CLOSED_LOOP:
                self._steer_closed_loop(lin, ang)
            else:
                self._steer_open_loop(ang)

            # gripper stall guard
            if self.grip_state != 'stop' and (now - self.grip_started) > self.GRIP_MAX_S:
                self.get_logger().warn('gripper auto-stop (max run time)')
                self._grip_hw('stop')

    def _at_limit(self, cmd):
        if self.steer_angle is None:
            return False
        deg = math.degrees(self.steer_angle)

        # Normal range, with LOOK-AHEAD. The wheel keeps moving for a moment
        # after the motor is cut (loop latency + mechanical coast), so stop
        # when the PREDICTED angle would exceed the limit, not the current one.
        lead_deg = self.steer_rate_dps * self.STEER_LEAD
        pred = deg + lead_deg
        max_deg = math.degrees(self.STEER_MAX)
        if cmd == 'left' and pred >= max_deg:
            return True
        if cmd == 'right' and pred <= -max_deg:
            return True

        # Backstop. Past 90 deg either the centre is wrong or a sign is
        # inverted. Block only the direction taking us FURTHER from centre
        # so the wheel can still be steered back.
        if abs(deg) > 90.0:
            self.get_logger().error(
                f'wheel is {deg:+.1f} deg from centre (raw {self.CENTER_RAW}). '
                f'Either the centre is wrong, or the encoder sign is inverted '
                f'(commanding left should make this number go POSITIVE). '
                f'Try -p as5600_ccw_positive:=false, or re-centre with "here" '
                f'on /agv/steer_home.', throttle_duration_sec=3.0)
            if self._steer_away_cmd == cmd:
                return True          # this direction makes it worse
        return False

    def _steer_open_loop(self, ang):
        now = time.monotonic()
        if ang > self.ANG_DB:
            cmd = 'left'
        elif ang < -self.ANG_DB:
            cmd = 'right'
        else:
            cmd = 'stop'

        if cmd != 'stop' and self._at_limit(cmd):
            self.get_logger().warn(
                f'steering limit ({cmd}) — holding at '
                f'{math.degrees(self.steer_angle):+.1f} deg '
                f'(rate {self.steer_rate_dps:+.0f} deg/s, '
                f'lead {self.STEER_LEAD:.2f}s)',
                throttle_duration_sec=2.0)
            cmd = 'stop'

        # Backstop against a runaway: the steering has no mechanical end
        # stops, so never run the motor continuously beyond steer_max_run_s.
        if cmd in ('left', 'right'):
            if self.steer_state != cmd or self._steer_cmd_since is None:
                self._steer_cmd_since = now
            elif (now - self._steer_cmd_since) > self.STEER_MAX_RUN:
                self.get_logger().error(
                    f'steering ran {self.STEER_MAX_RUN:.1f}s continuously — '
                    f'stopping. Release the command before steering again.')
                self._steer_cmd_since = None
                cmd = 'stop'
        else:
            self._steer_cmd_since = None

        self._steer_hw(cmd)

    def _steer_closed_loop(self, lin, ang):
        if self.steer_angle is None:
            self._steer_hw('stop')
            return
        v_eff = max(abs(lin), 0.15)
        target = math.atan2(self.WHEELBASE * ang, v_eff)
        self.steer_target = max(-self.STEER_MAX, min(self.STEER_MAX, target))
        err = self.steer_target - self.steer_angle
        abs_err = abs(err)

        if abs_err < self.STEER_DB:
            self._steer_hw('stop')
            self._prev_abs_err = None
            self._worsening = 0
            return

        cmd = 'left' if err > 0 else 'right'
        if self._at_limit(cmd):
            self.get_logger().warn(f'steering limit ({cmd}) — holding',
                                   throttle_duration_sec=2.0)
            self._steer_hw('stop')
            return

        if self.steer_state in ('left', 'right') and self._prev_abs_err is not None:
            if abs_err > self._prev_abs_err + 1e-4:
                self._worsening += 1
            else:
                self._worsening = 0
            if self._worsening >= self.RUNAWAY_N:
                self.steer_fault = ('STEERING RUNAWAY — angle moved away from '
                                    'target. Flip as5600_ccw_positive or '
                                    'invert_steering.')
                self.get_logger().error(self.steer_fault)
                self._steer_hw('stop')
                return
        self._prev_abs_err = abs_err
        self._steer_hw(cmd)

    # ================= status =================
    def publish_status(self):
        with self.lock:
            s = {
                'estopped': self.estopped,
                'direction': self.direction,
                'throttle_v': round(self.current_v, 2),
                'target_v': round(self.target_v, 2),
                'drive_v_forward': round(self.DRIVE_FWD, 2),
                'drive_v_reverse': round(self.DRIVE_REV, 2),
                'brake': 'applied' if self.brake_engaged else 'released',
                'brake_mode': 'auto' if self.brake_override is None else 'manual',
                'brake_max_engage_s': self.BRAKE_MAX_S,
                'cmd_lin': round(self.cmd_lin, 3),
                'cmd_ang': round(self.cmd_ang, 3),
                'steer': self.steer_state,
                'steer_angle_deg': (round(math.degrees(self.steer_angle), 1)
                                    if self.steer_angle is not None else None),
                'steer_target_deg': (round(math.degrees(self.steer_target), 1)
                                     if self.CLOSED_LOOP else None),
                'steer_fault': self.steer_fault,
                'steer_rate_dps': round(self.steer_rate_dps, 1),
                'as5600_raw': self._last_raw,
                'as5600_center_raw': self.CENTER_RAW,
                'enc_accum_deg': round(self.enc_accum_deg, 1),
                'enc_max_step': round(self.enc_max_step, 2),
                'enc_lost': self.enc_lost_count,
                'homed': self.homed,
                'gear_ratio': self.GEAR,
                'grip': self.grip_state,
                'closed_loop': self.CLOSED_LOOP,
                'mock': MOCK,
            }
        m = String()
        m.data = json.dumps(s)
        self.status_pub.publish(m)

    # ================= shutdown =================
    def shutdown_hw(self):
        try:
            with self.lock:
                self.target_v = self.current_v = self.V_IDLE
                if not MOCK:
                    self._write_dac(self.V_IDLE)
                self._steer_hw('stop')
                self._grip_hw('stop')
                self.brake_override = None
                self._brake(False)         # leave de-energised, never held on
                if not MOCK:
                    self.spi.close()
                    lgpio.gpiochip_close(self.h)
                if self.as5600 is not None:
                    self.as5600.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = AGVMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('shutting down — motors idle, brake applied')
        node.shutdown_hw()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
