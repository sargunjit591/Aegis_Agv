#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — BNO085 IMU NODE  (UART-RVC)
============================================================
Publishes the AGV's heading from the BNO085 in UART-RVC mode,
plus an experimental IMU-only distance estimate.

WIRING (RVC mode)
    BNO085 VIN ── Pi 3.3V   (pin 1)
    BNO085 GND ── Pi GND    (pin 6)
    BNO085 SDA ── Pi GPIO15 / RXD (pin 10)   <- serial TX in RVC mode
    BNO085 P0  ── Pi 3.3V                    <- selects UART-RVC
    BNO085 PS1 ── leave unconnected

MOUNTING
    Rigid (screws, not tape — vibration integrates into error).
    Board flat, Z truly vertical. X axis along the AGV's forward
    direction; if it points another way use --forward-axis.
    Mount near the REAR AXLE CENTRELINE: that is close to the
    AGV's instantaneous centre of rotation, so turning adds the
    least centripetal acceleration to the forward channel.

TOPICS
    /agv/imu     std_msgs/String   JSON at 50 Hz:
                   yaw_deg      heading, 0 = where it was zeroed
                   raw_yaw_deg  the sensor's own yaw
                   pitch_deg roll_deg
                   yaw_rate_dps
                   accel_fwd    forward acceleration, m/s^2
                   speed_mps    integrated, ZUPT-corrected
                   distance_m   integrated  (SEE THE WARNING)
                   stationary   ZUPT active
                   age_s        seconds since the last packet
    /agv/imu_zero  std_msgs/String  "zero" -> yaw and distance to 0

WARNING ON distance_m
    Double-integrating acceleration drifts as t^2. Expect roughly
    10 cm of error over a 5 s leg and far worse beyond 10 s, and
    expect it to vary between runs. Use it as a cross-check, not
    as truth. Wheel encoders are the real answer for distance.
    Zero-velocity updates (ZUPT) bound the drift while stopped
    but cannot undo it while moving.

USAGE
    python3 agv_imu_node.py                       # defaults
    python3 agv_imu_node.py --port /dev/serial0
    python3 agv_imu_node.py --test                # no ROS, just print

DEPENDENCIES
    pip install adafruit-circuitpython-bno08x-rvc pyserial \\
        --break-system-packages
============================================================
"""

import argparse
import json
import math
import threading
import time

import serial

try:
    from adafruit_bno08x_rvc import BNO08x_RVC
    HAVE_RVC = True
except ImportError:
    HAVE_RVC = False


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


class IMUReader:
    """Reads the BNO085 RVC stream and maintains heading + distance."""

    def __init__(self, port, baud, forward_axis, accel_sign,
                 zupt_accel, zupt_rate, zupt_hold_s, accel_deadband,
                 calib_s=3.0, max_speed=3.0):
        self.port = port
        self.baud = baud
        self.forward_axis = forward_axis      # 'x' | 'y' | 'z'
        self.accel_sign = accel_sign
        self.zupt_accel = zupt_accel
        self.zupt_rate = zupt_rate
        self.zupt_hold_s = zupt_hold_s
        self.accel_deadband = accel_deadband
        self.calib_s = calib_s
        self.max_speed = max_speed

        self.lock = threading.Lock()
        self.ok = False
        self.err = None
        self.raw_yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_offset = 0.0
        self.yaw_rate = 0.0
        self.accel_fwd = 0.0
        self.speed = 0.0
        self.distance = 0.0
        self.stationary = True
        self.last_pkt = 0.0
        self.samples = 0

        self._prev_yaw = None
        self._prev_t = None
        self._still_since = None
        self.drive_active = False      # set from /agv/status
        self.drive_stamp = 0.0
        self._bias = 0.0
        self._bias_n = 0
        self._calib_until = None      # startup calibration window
        self._calib_sum = 0.0
        self._calib_cnt = 0
        self.calibrated = False
        self.distance_valid = True
        self._stop = threading.Event()

    # ---------- lifecycle ----------
    def start(self):
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def stop(self):
        self._stop.set()

    def start_calibration(self):
        """Assume the AGV is STATIONARY for calib_s and learn the resting
        forward-accel offset, whatever its size. Without this a large
        offset (e.g. gravity leaking in from a tilted mount) can never be
        detected as 'still', so the bias is never learned and the velocity
        integrator runs away."""
        with self.lock:
            self._calib_until = time.monotonic() + self.calib_s
            self._calib_sum = 0.0
            self._calib_cnt = 0
            self.calibrated = False
            self.speed = 0.0
            self.distance = 0.0
            self.distance_valid = True

    def set_drive_active(self, active):
        with self.lock:
            self.drive_active = bool(active)
            self.drive_stamp = time.monotonic()

    def zero(self):
        """Make the current heading 0 and restart the distance integrator."""
        with self.lock:
            self.yaw_offset = self.raw_yaw
        self.start_calibration()
        return True

    # ---------- reader ----------
    def _run(self):
        while not self._stop.is_set():
            try:
                uart = serial.Serial(self.port, self.baud, timeout=0.1)
                rvc = BNO08x_RVC(uart)
                with self.lock:
                    self.ok = True
                    self.err = None
                print(f'[IMU] connected on {self.port} @ {self.baud}')
                print(f'[IMU] hold the AGV STILL for {self.calib_s:.0f}s '
                      f'— calibrating…')
                self.start_calibration()
                while not self._stop.is_set():
                    try:
                        yaw, pitch, roll, ax, ay, az = rvc.heading
                    except Exception:
                        continue          # partial packet, just resync
                    self._update(yaw, pitch, roll, ax, ay, az)
            except Exception as e:
                with self.lock:
                    self.ok = False
                    self.err = str(e)
                print(f'[IMU] {e} — retrying in 2 s')
                time.sleep(2.0)

    def _update(self, yaw, pitch, roll, ax, ay, az):
        now = time.monotonic()
        a = {'x': ax, 'y': ay, 'z': az}[self.forward_axis] * self.accel_sign

        with self.lock:
            self.raw_yaw = yaw
            self.pitch = pitch
            self.roll = roll
            self.last_pkt = now
            self.samples += 1

            # --- yaw rate from consecutive headings (wrap-safe) ---
            if self._prev_t is not None:
                dt = now - self._prev_t
                if dt > 1e-4:
                    self.yaw_rate = wrap180(yaw - self._prev_yaw) / dt
            self._prev_yaw = yaw
            prev_t = self._prev_t
            self._prev_t = now

            self.accel_fwd = a

            # --- startup calibration window ---
            if self._calib_until is not None:
                if now < self._calib_until:
                    self._calib_sum += a
                    self._calib_cnt += 1
                    self.speed = 0.0
                    return
                if self._calib_cnt > 0:
                    self._bias = self._calib_sum / self._calib_cnt
                    self._bias_n = self._calib_cnt
                    self.calibrated = True
                    print(f'\n[IMU] calibrated: resting forward accel '
                          f'{self._bias:+.3f} m/s^2'
                          + ('  <-- large, the board is probably not level'
                             if abs(self._bias) > 0.5 else ''))
                self._calib_until = None

            if prev_t is None:
                return
            dt = now - prev_t
            if dt <= 0 or dt > 0.5:
                return                      # gap in the stream, skip

            # --- are we standing still? ---
            # An accelerometer CANNOT distinguish cruising at constant speed
            # from being stopped: both read zero acceleration. So ZUPT is
            # gated on the drive actually being commanded to idle, taken
            # from /agv/status. Without that signal we never ZUPT while
            # moving, and the estimate just drifts (as it must).
            drive_known = (time.monotonic() - self.drive_stamp) < 1.0
            drive_idle = (not self.drive_active) if drive_known else True
            still = (drive_idle
                     and abs(a - self._bias) < self.zupt_accel
                     and abs(self.yaw_rate) < self.zupt_rate)
            if still:
                if self._still_since is None:
                    self._still_since = now
                if (now - self._still_since) >= self.zupt_hold_s:
                    # ZUPT: known to be stopped, so velocity is exactly zero.
                    # Also refine the accelerometer bias while we are here.
                    self.stationary = True
                    self.speed = 0.0
                    self._bias_n += 1
                    k = 1.0 / min(self._bias_n, 500)
                    self._bias += k * (a - self._bias)
                    return
            else:
                self._still_since = None
                self.stationary = False

            # --- integrate ---
            acc = a - self._bias
            if abs(acc) < self.accel_deadband:
                acc = 0.0
            self.speed += acc * dt
            self.distance += self.speed * dt
            # A ground AGV cannot exceed a few m/s. Anything past that means
            # the integrator has diverged — flag it instead of reporting
            # nonsense like 500 m/s.
            if abs(self.speed) > self.max_speed:
                self.distance_valid = False

    # ---------- output ----------
    def state(self):
        with self.lock:
            return {
                'ok': self.ok,
                'err': self.err,
                'yaw_deg': round(wrap180(self.raw_yaw - self.yaw_offset), 2),
                'raw_yaw_deg': round(self.raw_yaw, 2),
                'pitch_deg': round(self.pitch, 2),
                'roll_deg': round(self.roll, 2),
                'yaw_rate_dps': round(self.yaw_rate, 1),
                'accel_fwd': round(self.accel_fwd, 3),
                'accel_bias': round(self._bias, 3),
                'speed_mps': round(self.speed, 3),
                'distance_m': (round(self.distance, 3)
                               if self.distance_valid else None),
                'distance_valid': self.distance_valid,
                'calibrated': self.calibrated,
                'calibrating': self._calib_until is not None,
                'stationary': self.stationary,
                'drive_active': self.drive_active,
                'samples': self.samples,
                'age_s': round(time.monotonic() - self.last_pkt, 2)
                if self.last_pkt else None,
            }


# ------------------------------------------------------------------
def build(args):
    if not HAVE_RVC:
        raise SystemExit(
            'adafruit_bno08x_rvc not installed:\n'
            '  pip install adafruit-circuitpython-bno08x-rvc pyserial '
            '--break-system-packages')
    return IMUReader(args.port, args.baud, args.forward_axis, args.accel_sign,
                     args.zupt_accel, args.zupt_rate, args.zupt_hold,
                     args.accel_deadband, args.calib_s, args.max_speed)


def run_test(args):
    imu = build(args)
    imu.start()
    print('=' * 70)
    print('  BNO085 RVC TEST — rotate the AGV and watch yaw_deg')
    print('  Ctrl+C to stop.  Press Enter to zero heading + distance.')
    print('=' * 70)
    threading.Thread(target=lambda: [imu.zero() for _ in iter(input, None)],
                     daemon=True).start()
    try:
        while True:
            s = imu.state()
            if not s['ok']:
                print(f"\r  waiting for the sensor… {s['err']}          ",
                      end='', flush=True)
            else:
                d = ('CALIBRATING' if s['calibrating']
                     else (f"{s['distance_m']:+6.2f} m"
                           if s['distance_m'] is not None else 'DIVERGED'))
                print(f"\r  yaw {s['yaw_deg']:+7.2f}  rate {s['yaw_rate_dps']:+6.1f}/s  "
                      f"| a_fwd {s['accel_fwd']:+5.2f} (bias {s['accel_bias']:+5.2f})  "
                      f"v {s['speed_mps']:+6.2f}  dist {d}  "
                      f"{'STILL' if s['stationary'] else 'MOVING'}      ",
                      end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nstopped.')
    finally:
        imu.stop()


def run_ros(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class IMUNode(Node):
        def __init__(self):
            super().__init__('agv_imu')
            self.imu = build(args)
            self.imu.start()
            self.pub = self.create_publisher(String, 'agv/imu', 10)
            self.create_subscription(String, 'agv/imu_zero', self.on_zero, 10)
            # the motor bridge tells us whether the drive is actually running,
            # which is the only reliable 'am I stopped' signal an IMU has
            self.create_subscription(String, 'agv/status', self.on_bridge, 10)
            self.create_timer(1.0 / args.rate, self.tick)
            self.warned = False
            self.get_logger().info(
                f'BNO085 RVC on {args.port}, forward axis '
                f'{args.forward_axis}{"+" if args.accel_sign > 0 else "-"}, '
                f'publishing /agv/imu at {args.rate:.0f} Hz')

        def on_bridge(self, msg):
            try:
                st = json.loads(msg.data)
            except Exception:
                return
            v = st.get('throttle_v')
            idle = 0.8
            if v is not None:
                self.imu.set_drive_active(v > idle + 0.05)

        def on_zero(self, msg):
            if msg.data.strip().lower() in ('zero', 'here', 'reset'):
                self.imu.zero()
                self.get_logger().info('IMU heading and distance zeroed')

        def tick(self):
            s = self.imu.state()
            if not s['ok'] and not self.warned:
                self.get_logger().error(f"IMU not connected: {s['err']}")
                self.warned = True
            elif s['ok']:
                self.warned = False
            m = String()
            m.data = json.dumps(s)
            self.pub.publish(m)

    rclpy.init()
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.imu.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description='BNO085 UART-RVC IMU node')
    p.add_argument('--port', default='/dev/ttyAMA0',
                   help='try /dev/serial0 if this one is not there')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--rate', type=float, default=50.0, help='publish Hz')
    p.add_argument('--forward-axis', choices=('x', 'y', 'z'), default='x',
                   help='which IMU axis points along the AGV forward')
    p.add_argument('--accel-sign', type=float, default=1.0,
                   help='-1 if that axis points backwards')
    p.add_argument('--zupt-accel', type=float, default=0.15,
                   help='m/s^2 below which we may be stopped')
    p.add_argument('--zupt-rate', type=float, default=2.0,
                   help='deg/s below which we may be stopped')
    p.add_argument('--zupt-hold', type=float, default=0.30,
                   help='seconds of stillness before ZUPT fires')
    p.add_argument('--accel-deadband', type=float, default=0.05,
                   help='ignore accelerations smaller than this')
    p.add_argument('--calib-s', type=float, default=3.0,
                   help='hold still this long at startup to learn the '
                        'resting forward-accel offset')
    p.add_argument('--max-speed', type=float, default=3.0,
                   help='m/s above which the integrator is declared diverged')
    p.add_argument('--test', action='store_true',
                   help='plain terminal readout, no ROS')
    a = p.parse_args()

    if a.test:
        run_test(a)
    else:
        run_ros(a)


if __name__ == '__main__':
    main()
