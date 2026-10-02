#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — AGV MOTOR BRIDGE (ROS2 node)  [REV 5]
============================================================
Subscribes /cmd_vel and drives the AGV.

  linear.x  -> direction relay + drive throttle (fixed voltages)
  angular.z -> steering motor (BTS7960 #2)

HARDWARE MAP  (must match rpi_os_console/agv_console_angle.py)
  Steering : BTS7960 #2  GPIO19=RPWM(right) GPIO16=LPWM(left)  active HIGH
  Gripper  : BTS7960 #1  GPIO13=RPWM(open)  GPIO12=LPWM(close) active LOW
  Dir relay: relay IN1   GPIO5    LOW = FORWARD, HIGH = REVERSE
  BRAKE    : relay IN2   GPIO6    24 V self-opening actuator;
                                  the relay shorts its two wires
  Throttle : Waveshare HAT DAC0, SPI(0,1) mode 1, CS=GPIO23
             0.80 V idle | 1.50 V forward | 1.28 V reverse | 4.20 V max
  Steering feedback : AS5600, I2C bus 1 @ 0x36, 1:1 on the steering
             pivot, single-turn, centre raw 3791, LEFT reads POSITIVE

DEFAULTS ARE THE HARDWARE-VERIFIED VALUES — no launch flags needed:
    python3 agv_motor_bridge.py

STEERING ANGLE
  The AS5600 is absolute and sits 1:1 on the pivot, so the angle is
  (raw - centre) on a single turn. The wheel has NO end stops: if the
  angle ever jumps +/-170 -> -/+170 it crossed the single-turn seam —
  stop and re-home. Re-centre (wheel straight) with
    ros2 topic pub --once /agv/steer_home std_msgs/msg/String "{data: here}"
  and pass the reported raw value as -p as5600_center_raw:=<n> to keep it.

BRAKE BEHAVIOUR — PULSED, NEVER HELD
  The actuator is self-opening, so it must not sit energised. The
  brake therefore fires as a short PULSE and always lets go:
    * RELEASED at startup and at shutdown
    * one pulse when the drive stops (moving -> stopped)
    * one pulse on E-stop
    * every engagement auto-releases after brake_max_engage_s
      (default 2.0 s) no matter how it was triggered, including a
      manual APPLY and an E-stop
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
  /agv/steer_home   std_msgs/String       in   "here"
  /agv/status       std_msgs/String       out  JSON at 20 Hz

GPIOCHIP
  The 40-pin header is gpiochip0 on Raspberry Pi OS but gpiochip4
  on Ubuntu 24.04. Opening the wrong one succeeds silently and no
  pin ever moves, so the chip is detected by label. Override with
  -p gpiochip:=<n>.
============================================================
"""

import json
import math
import re
import subprocess
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


# Every ROS parameter, declared exactly once from this table.
PARAMS = {
    # ---- chip ----
    'gpiochip': -1,                 # -1 = auto-detect by label

    # ---- steering (BTS7960 #2) ----
    'steer_rpwm_pin': 19,           # right
    'steer_lpwm_pin': 16,           # left
    'steer_active_low': False,
    'invert_steering': False,

    # ---- gripper (BTS7960 #1) ----
    'grip_rpwm_pin': 13,            # open
    'grip_lpwm_pin': 12,            # close
    'grip_active_low': True,
    'gripper_max_run_s': 8.0,

    # ---- drive ----
    'relay_dir_pin': 5,             # relay IN1
    'invert_direction': False,      # False => GPIO5 LOW = FORWARD
    'dac_cs_pin': 23,
    'dac_vref': 5.0,
    'throttle_idle_v': 0.80,
    'throttle_max_v': 4.20,
    'drive_volts_forward': 1.50,    # measured start threshold
    'drive_volts_reverse': 1.28,    # measured start threshold
    'fixed_speed': True,            # False = proportional (Nav2 will want it)
    'max_linear_mps': 1.0,
    'linear_deadband': 0.02,
    'slew_step_v': 0.15,            # per control tick
    'control_rate_hz': 50.0,
    'watchdog_timeout_s': 0.5,

    # ---- brake (relay IN2) ----
    'brake_pin': 6,
    'brake_engage_level': 0,        # 0 = LOW applies the brake (UNVERIFIED)
    'brake_enable': True,
    'brake_release_lead_s': 0.35,   # brake lets go before throttle rises
    'brake_max_engage_s': 2.0,      # HARD LIMIT: auto-release after this long.
                                    # The actuator is self-opening and must
                                    # not be held energised.

    # ---- steering control ----
    'angular_deadband': 0.1,
    'closed_loop_steering': False,
    'wheelbase_m': 0.9,
    'steer_max_deg': 45.0,
    'steer_deadband_deg': 2.0,
    'steer_max_run_s': 4.0,         # backstop: never run the steering motor
                                    # continuously longer than this
    'steer_stop_lead_s': 0.12,      # stop this far AHEAD of the limit, scaled
                                    # by the measured turn rate, to cancel
                                    # latency + coast
    'steer_rate_alpha': 0.35,       # smoothing on the rate estimate
    'runaway_ticks': 10,

    # ---- AS5600 (1:1 on the steering pivot, single-turn absolute) ----
    'as5600_bus': 1,
    'as5600_center_raw': 3791,      # raw reading with the wheel straight
    'as5600_ccw_positive': False,   # False => steering LEFT reads POSITIVE
}


def find_header_chip(requested=-1, log=print):
    """Number of the gpiochip that owns the 40-pin header.

    It is gpiochip0 on Raspberry Pi OS and gpiochip4 on Ubuntu 24.04.
    Opening the wrong chip succeeds silently and no pin ever moves, so
    match on the chip label. Keep identical in bridge and console."""
    if requested is not None and int(requested) >= 0:
        log(f'gpiochip{int(requested)} set explicitly')
        return int(requested)

    chips = {}                                  # n -> (label, lines)
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
                lines, label = getattr(info, 'lines', 0), getattr(info, 'label', '')
            if isinstance(label, bytes):
                label = label.decode(errors='ignore')
            chips[n] = (str(label), int(lines))
        except Exception:
            pass
        finally:
            try:
                lgpio.gpiochip_close(h)
            except Exception:
                pass

    if not chips:                               # fall back to gpiodetect
        try:
            out = subprocess.run(['gpiodetect'], capture_output=True,
                                 text=True, timeout=5).stdout
        except Exception:
            out = ''
        for line in out.splitlines():
            m = re.match(r'gpiochip(\d+)\s+\[([^\]]+)\]\s+\((\d+) lines\)',
                         line.strip())
            if m:
                chips[int(m.group(1))] = (m.group(2), int(m.group(3)))

    for n in sorted(chips):
        log(f'  gpiochip{n}: {chips[n][0]!r} ({chips[n][1]} lines)')
    for n in sorted(chips):
        label = chips[n][0].lower()
        if 'rp1' in label or 'bcm2835' in label or 'bcm2711' in label:
            log(f'40-pin header = gpiochip{n} (by label)')
            return n
    for n in sorted(chips):
        if chips[n][1] == 54:
            log(f'40-pin header = gpiochip{n} (54 lines)')
            return n
    log('ERROR: 40-pin header NOT identified — falling back to gpiochip0. '
        'If no pin moves, set the chip number explicitly.')
    return 0


class AGVMotorBridge(Node):

    def __init__(self):
        super().__init__('agv_motor_bridge')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)

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
        self.RUNAWAY_N = int(g('runaway_ticks'))
        self.CENTER_RAW = int(g('as5600_center_raw'))
        self.CCW_POS = bool(g('as5600_ccw_positive'))

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
        self.steer_angle = None         # radians, + = left, None = no encoder
        self.steer_target = 0.0
        self.steer_fault = None
        self.steer_rate_dps = 0.0       # measured wheel turn rate, deg/s
        self._last_raw = None
        self._rate_prev_deg = None
        self._rate_prev_t = None
        self._prev_abs_err = None       # closed-loop runaway guard
        self._worsening = 0
        self._prev_abs_deg = None
        self._steer_away_cmd = None     # learned: which command leaves centre
        self._steer_cmd_since = None    # max-run backstop
        self._steer_blocked = None      # command held off after a max-run trip
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
            chip = find_header_chip(g('gpiochip'), self.get_logger().info)
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
                deg = math.degrees(self._read_as5600())
                self.get_logger().info(
                    f'AS5600 OK — centre raw {self.CENTER_RAW}, now {deg:+.1f} deg')
                if abs(deg) > 10.0:
                    self.get_logger().warn(
                        f'startup wheel angle is {deg:+.1f} deg, not ~0 — '
                        f'straighten the wheel, or re-home via /agv/steer_home '
                        f'if the wheel IS straight')
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
        self.get_logger().info(
            f'AGV motor bridge REV5 — steering {mode}, drive {spd}, '
            f'dir relay GPIO{self.RELAY} ({fwd}=forward), brake {brk}, '
            f'watchdog {self.WDT}s' + (' [MOCK]' if MOCK else ''))

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

    def _read_as5600(self):
        """Wheel angle in radians, signed, 0 = straight ahead, + = left.
        Wrap-safe for a centre near 0/4095."""
        diff = (self._read_raw() - self.CENTER_RAW + 2048) % 4096 - 2048
        ang = diff * (2 * math.pi / 4096.0)
        return ang if self.CCW_POS else -ang

    def home_steering(self):
        """Declare the CURRENT wheel position to be straight ahead.

        The AS5600 is absolute, so a centre survives power cycles — do
        this only when the centre is wrong, not at every startup."""
        if self.as5600 is None:
            self.get_logger().error('cannot home — no AS5600')
            return
        try:
            raw = self._read_raw()
        except Exception as e:
            self.get_logger().error(f'cannot home — AS5600 read failed: {e}')
            return
        with self.lock:
            self.CENTER_RAW = raw
            self.steer_fault = None
            self._prev_abs_err = None
            self._worsening = 0
        self.get_logger().info(
            f'steering CENTRED at raw {raw} — this position is now 0 deg. '
            f'Pass -p as5600_center_raw:={raw} to make it the default.')

    # ================= callbacks =================
    def on_cmd_vel(self, msg: Twist):
        with self.lock:
            if self.estopped:
                return      # nothing sent during an E-stop may act after reset
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
            self._update_steer_angle(now)

            # Brake hard limit. Runs BEFORE the E-stop return: the E-stop
            # pulse must auto-release too — the actuator is self-opening and
            # must never be held energised.
            if (self.BRAKE_EN and self.brake_engaged
                    and self.brake_engaged_at is not None
                    and (now - self.brake_engaged_at) >= self.BRAKE_MAX_S):
                self._brake(False)
                self.brake_pulse_spent = True
                self.brake_override = None      # drop any manual hold
                self.get_logger().info(
                    f'brake auto-released after {self.BRAKE_MAX_S:.1f}s limit')

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
            # Fires once when the drive stops, then stays off until the AGV
            # moves again. (The hard auto-release is above.)
            if self.BRAKE_EN:
                # a drive command always wins — let go immediately
                if moving and self.brake_engaged:
                    self._brake(False)

                # re-arm the pulse once the AGV is actually moving
                if moving:
                    self.brake_pulse_spent = False

                # fire one pulse on the moving -> stopped transition
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

    def _update_steer_angle(self, now):
        """Read the wheel angle, estimate its turn rate, and learn which
        command drives the wheel AWAY from centre (for the >90 deg backstop
        in _at_limit)."""
        if self.as5600 is None:
            return
        try:
            self.steer_angle = self._read_as5600()
        except Exception:
            self.steer_angle = None
            return
        d = math.degrees(self.steer_angle)

        # how fast the wheel is actually turning, so the limit check can
        # stop early enough to land on target
        if self._rate_prev_t is not None:
            dt = now - self._rate_prev_t
            if dt > 1e-4:
                raw_rate = (d - self._rate_prev_deg) / dt
                a = self.RATE_ALPHA
                self.steer_rate_dps = a * raw_rate + (1.0 - a) * self.steer_rate_dps
        self._rate_prev_deg = d
        self._rate_prev_t = now

        if (self._prev_abs_deg is not None
                and self.steer_state in ('left', 'right')
                and abs(d) > self._prev_abs_deg + 0.2):
            self._steer_away_cmd = self.steer_state
        self._prev_abs_deg = abs(d)

    def _at_limit(self, cmd):
        if self.steer_angle is None:
            return False
        deg = math.degrees(self.steer_angle)

        # Normal range, with LOOK-AHEAD. The wheel keeps moving for a moment
        # after the motor is cut (loop latency + mechanical coast), so stop
        # when the PREDICTED angle would exceed the limit, not the current one.
        pred = deg + self.steer_rate_dps * self.STEER_LEAD
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
                f'(commanding left must make this number go POSITIVE). '
                f'Re-centre with "here" on /agv/steer_home, or flip '
                f'as5600_ccw_positive.', throttle_duration_sec=3.0)
            if self._steer_away_cmd == cmd:
                return True          # this direction makes it worse
        return False

    def _steer_open_loop(self, ang):
        now = time.monotonic()
        if ang > self.ANG_DB:
            want = 'left'
        elif ang < -self.ANG_DB:
            want = 'right'
        else:
            want = 'stop'

        # a max-run trip holds until the command is released or reversed
        if want != self._steer_blocked:
            self._steer_blocked = None

        cmd = want
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
        if cmd == 'stop' or self._steer_blocked:
            cmd = 'stop'
            self._steer_cmd_since = None
        elif self.steer_state != cmd or self._steer_cmd_since is None:
            self._steer_cmd_since = now
        elif (now - self._steer_cmd_since) > self.STEER_MAX_RUN:
            self.get_logger().error(
                f'steering ran {self.STEER_MAX_RUN:.1f}s continuously — '
                f'stopping. Release the command before steering again.')
            self._steer_blocked = want
            self._steer_cmd_since = None
            cmd = 'stop'

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
        node.get_logger().info('shutting down — motors idle, brake released')
        node.shutdown_hw()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
