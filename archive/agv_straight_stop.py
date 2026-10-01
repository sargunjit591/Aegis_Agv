#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — STRAIGHT LINE + OBSTACLE STOP  (demo)
============================================================
Drives the AGV straight forward and stops before an obstacle,
using the RPLiDAR C1 only. No SLAM, no odometry, no TF.

HOW IT WORKS
  subscribe /scan -> look at a cone around "forward" ->
  take the closest valid return -> if closer than STOP_DISTANCE,
  publish zero /cmd_vel and latch stopped. Otherwise keep
  publishing forward /cmd_vel at 20 Hz.

WHY IT PUBLISHES /cmd_vel INSTEAD OF DRIVING GPIO
  agv_motor_bridge.py is already tested and owns the hardware.
  It also gives you, for free:
    * 0.5 s cmd_vel watchdog — if THIS node dies or is killed,
      the AGV stops on its own within half a second
    * slew-limited throttle, direction interlock, E-stop topic
  Run the bridge first, then this.

  !! STEP 1 IS ALWAYS --check !!
  You must know which bearing in the scan is the AGV's forward
  direction. If the lidar is mounted rotated, the cone looks the
  wrong way and the AGV will drive into the object.

      python3 agv_straight_stop.py --check

  Put the obstacle directly in front of the AGV, run that, and
  read which bearing reports the short distance. If it is not
  0 deg, pass it:  --forward-bearing <deg>

USAGE
  python3 agv_straight_stop.py --check              # find forward
  python3 agv_straight_stop.py --dry-run            # no motion
  python3 agv_straight_stop.py                      # the demo
  python3 agv_straight_stop.py --stop-distance 1.2 --forward-bearing 180

SAFETY
  * Ctrl+C stops immediately (publishes zeros, then the bridge
    watchdog catches anything missed)
  * hard run-time limit — will not drive longer than MAX_RUN_S
  * stops if the scan goes stale (lidar unplugged / node died)
  * starts with a 3 s countdown so you can step back
  * STOP_DISTANCE must include coasting distance — the BLDC does
    not brake hard. Start at 1.5 m, watch where it actually
    halts, then tighten.
============================================================
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class StraightStop(Node):

    def __init__(self, args):
        super().__init__('agv_straight_stop')
        self.a = args
        self.latest = None          # (stamp_monotonic, min_dist, n_valid)
        self.stopped_reason = None
        self.started = time.monotonic()
        self.driving = False

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        # subscribe twice-compatible: try BEST_EFFORT, most drivers publish
        # RELIABLE which is compatible with a BEST_EFFORT subscriber.
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / 20.0, self.tick)

    # ---------------- scan processing ----------------
    def cone_min(self, msg: LaserScan):
        """Closest valid return inside the forward cone.
        Returns (min_dist, n_valid). Uses the k-th smallest rather than
        the absolute minimum so one noisy pixel cannot false-trigger."""
        half = math.radians(self.a.cone_half_angle)
        centre = math.radians(self.a.watch_bearing)

        vals = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < max(msg.range_min, self.a.ignore_closer_than):
                continue          # self-hits / body of the AGV
            if r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            # wrap the difference to [-pi, pi]
            d = (ang - centre + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= half:
                vals.append(r)

        if not vals:
            return None, 0
        vals.sort()
        k = min(self.a.noise_reject, len(vals) - 1)
        return vals[k], len(vals)

    def on_scan(self, msg: LaserScan):
        d, n = self.cone_min(msg)
        self.latest = (time.monotonic(), d, n)

    # ---------------- control ----------------
    def publish(self, go):
        t = Twist()
        # a.speed is already signed: negative for reverse
        t.linear.x = float(self.a.speed) if go else 0.0
        t.angular.z = 0.0
        self.pub.publish(t)

    def halt(self, reason):
        if self.stopped_reason is None:
            self.stopped_reason = reason
        for _ in range(6):
            self.publish(False)
            time.sleep(0.02)

    def tick(self):
        if self.stopped_reason:
            self.publish(False)
            return

        now = time.monotonic()

        # hard time limit
        if self.driving and (now - self.started) > self.a.max_run:
            self.get_logger().warn(f'time limit {self.a.max_run}s — stopping')
            self.halt('time limit')
            return

        # no scan yet
        if self.latest is None:
            self.publish(False)
            self.get_logger().warn('waiting for /scan …',
                                   throttle_duration_sec=2.0)
            return

        stamp, dist, n = self.latest

        # stale scan = lidar died -> stop
        if now - stamp > 0.5:
            self.get_logger().error('scan is stale — stopping')
            self.halt('scan stale')
            return

        if dist is None:
            self.get_logger().warn('no valid returns in the cone — stopping',
                                   throttle_duration_sec=1.0)
            self.halt('no returns')
            return

        if dist <= self.a.stop_distance:
            self.get_logger().info(f'OBSTACLE at {dist:.2f} m — STOPPING')
            self.halt(f'obstacle at {dist:.2f} m')
            return

        self.driving = True
        self.publish(True)
        self.get_logger().info(f'clear ahead: {dist:5.2f} m  '
                               f'(stop at {self.a.stop_distance:.2f})',
                               throttle_duration_sec=0.5)


# ------------------------------------------------------------------
class ScanCheck(Node):
    """Prints the closest return in each 45 deg sector so you can see
    which bearing the AGV's forward direction corresponds to."""

    def __init__(self):
        super().__init__('agv_scan_check')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.n = 0

    def on_scan(self, msg: LaserScan):
        self.n += 1
        if self.n % 10:          # ~1 Hz at 10 Hz scans
            return
        buckets = {b: [] for b in range(-180, 180, 45)}
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            ang = (ang + 180) % 360 - 180
            b = int(math.floor(ang / 45.0) * 45)
            if b in buckets:
                buckets[b].append(r)
        line = []
        for b in sorted(buckets):
            v = buckets[b]
            line.append(f'{b:+4d}°:{min(v):5.2f}m' if v else f'{b:+4d}°:  --  ')
        print('  '.join(line), flush=True)


# ------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description='AGV straight-line obstacle stop')
    p.add_argument('--check', action='store_true',
                   help='print per-sector distances to identify forward, then exit')
    p.add_argument('--dry-run', action='store_true',
                   help='read the scan and report, but never command motion')
    p.add_argument('--stop-distance', type=float, default=1.5,
                   help='metres from the obstacle to stop (default 1.5)')
    p.add_argument('--forward-bearing', type=float, default=0.0,
                   help='bearing in the scan that points forward (deg)')
    p.add_argument('--reverse', action='store_true',
                   help='drive BACKWARDS and watch the rear cone instead')
    p.add_argument('--rear-bearing', type=float, default=None,
                   help='override the rear bearing (default: forward + 180)')
    p.add_argument('--cone-half-angle', type=float, default=15.0,
                   help='half-width of the forward cone (deg)')
    p.add_argument('--speed', type=float, default=0.5,
                   help='linear.x sent to the bridge (bridge uses fixed volts)')
    p.add_argument('--max-run', type=float, default=20.0,
                   help='hard time limit in seconds')
    p.add_argument('--ignore-closer-than', type=float, default=0.12,
                   help='ignore returns closer than this (AGV body)')
    p.add_argument('--noise-reject', type=int, default=2,
                   help='use the k-th smallest return instead of the min')
    a = p.parse_args()

    # Which cone do we watch, and which way do we drive?
    # The rear is simply the opposite bearing to forward, wrapped to
    # [-180, 180) — unless the user overrides it.
    if a.reverse:
        if a.rear_bearing is not None:
            a.watch_bearing = a.rear_bearing
        else:
            a.watch_bearing = (a.forward_bearing + 180.0 + 180.0) % 360.0 - 180.0
        a.speed = -abs(a.speed)
        a.mode = 'REVERSE'
    else:
        a.watch_bearing = a.forward_bearing
        a.speed = abs(a.speed)
        a.mode = 'FORWARD'

    rclpy.init()

    if a.check:
        print('=' * 66)
        print('  SCAN DIRECTION CHECK')
        print('  Put the obstacle DIRECTLY IN FRONT of the AGV.')
        print('  The sector showing the short distance is your forward bearing.')
        print('  Pass it as --forward-bearing <deg>.   Ctrl+C to exit.\n  For the reverse demo, also note what sits BEHIND the AGV.')
        print('=' * 66)
        node = ScanCheck()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    if a.dry_run:
        a.speed = 0.0

    print('=' * 66)
    print(f'  AGV STRAIGHT LINE + OBSTACLE STOP  —  {a.mode}')
    print(f'  stop distance   : {a.stop_distance:.2f} m')
    print(f'  forward bearing : {a.forward_bearing:+.1f} deg')
    print(f'  WATCHING cone   : {a.watch_bearing:+.1f} deg '
          f'(+/-{a.cone_half_angle:.0f} deg)  <-- the {"REAR" if a.reverse else "FRONT"}')
    print(f'  time limit      : {a.max_run:.0f} s')
    if a.reverse:
        print('  *** REVERSING — the AGV moves AWAY from the front. Make')
        print('      sure the path behind is clear and you can reach it. ***')
    if a.dry_run:
        print('  *** DRY RUN — no motion will be commanded ***')
    print('=' * 66)

    if not a.dry_run:
        if input('Area clear and E-stop/power within reach? (yes/no): ').strip().lower() != 'yes':
            print('Aborting.')
            if rclpy.ok():
                rclpy.shutdown()
            return
        for i in (3, 2, 1):
            print(f'  starting in {i} …', flush=True)
            time.sleep(1.0)

    node = StraightStop(a)

    def on_sig(_s, _f):
        print('\nCtrl+C — stopping AGV')
        node.halt('user abort')
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.halt('shutdown')
        reason = node.stopped_reason or 'shutdown'
        print(f'\nRESULT: {reason}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
