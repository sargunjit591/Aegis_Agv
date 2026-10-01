#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — DUAL-CONE OBSTACLE GUARD
============================================================
Drives the AGV straight (forward OR reverse) while watching
BOTH the front and rear cones at once. Stops for an obstacle
on EITHER side, whichever way it is travelling.

  * driving forward, someone steps in behind it -> STOPS
  * driving in reverse, something appears ahead  -> STOPS
  * one --stop-distance applies to both cones

Uses the RPLiDAR C1 only. No SLAM, no odometry, no TF.
Publishes /cmd_vel to agv_motor_bridge.py, so you keep the
bridge's 0.5 s watchdog, slew limiter, direction interlock and
E-stop topic. Run the bridge first.

  !! ALWAYS RUN --check FIRST !!
  You must know which scan bearing is the AGV's forward
  direction. The rear is computed as the opposite bearing.

      python3 agv_obstacle_guard.py --check

USAGE
  python3 agv_obstacle_guard.py --check
  python3 agv_obstacle_guard.py --forward-bearing -180 --dry-run
  python3 agv_obstacle_guard.py --forward-bearing -180 --stop-distance 1.5
  python3 agv_obstacle_guard.py --forward-bearing -180 --reverse

SAFETY
  * Ctrl+C stops immediately; if this node dies the bridge
    watchdog stops the AGV within 0.5 s
  * hard run-time limit (--max-run)
  * stops if the scan goes stale (lidar unplugged)
  * stops if a cone has no valid returns at all
  * 3 s countdown before motion
  * STOP_DISTANCE must include coasting distance — the BLDC
    does not brake. Start generous, then tighten.
============================================================
"""

import argparse
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


class ObstacleGuard(Node):

    def __init__(self, a):
        self.a = a
        super().__init__('agv_obstacle_guard')
        self.latest = None           # (t, front, rear, nf, nr)
        self.stopped_reason = None
        self.started = time.monotonic()
        self.driving = False

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / 20.0, self.tick)

    # ---------------- scan processing ----------------
    def cone_min(self, msg, centre_deg):
        """Closest valid return in a cone. Uses the k-th smallest so a
        single noisy pixel cannot false-trigger the stop."""
        half = math.radians(self.a.cone_half_angle)
        centre = math.radians(centre_deg)
        lo = max(msg.range_min, self.a.ignore_closer_than)

        vals = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < lo or r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            d = (ang - centre + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= half:
                vals.append(r)

        if not vals:
            return None, 0
        vals.sort()
        return vals[min(self.a.noise_reject, len(vals) - 1)], len(vals)

    def on_scan(self, msg):
        f, nf = self.cone_min(msg, self.a.front_bearing)
        r, nr = self.cone_min(msg, self.a.rear_bearing)
        self.latest = (time.monotonic(), f, r, nf, nr)

    # ---------------- control ----------------
    def publish(self, go):
        t = Twist()
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

        if self.driving and (now - self.started) > self.a.max_run:
            self.get_logger().warn(f'time limit {self.a.max_run:.0f}s reached')
            self.halt('time limit')
            return

        if self.latest is None:
            self.publish(False)
            self.get_logger().warn('waiting for /scan …',
                                   throttle_duration_sec=2.0)
            return

        stamp, front, rear, nf, nr = self.latest

        if now - stamp > 0.5:
            self.get_logger().error('scan is stale — stopping')
            self.halt('scan stale')
            return

        if front is None or rear is None:
            side = 'front' if front is None else 'rear'
            self.get_logger().warn(f'no valid returns in the {side} cone')
            self.halt(f'no returns ({side})')
            return

        # --- the guard: EITHER cone triggers, whichever way we travel ---
        if front <= self.a.stop_distance and rear <= self.a.stop_distance:
            self.get_logger().info(
                f'OBSTACLE BOTH SIDES — front {front:.2f} m, rear {rear:.2f} m')
            self.halt(f'obstacle both sides (F {front:.2f} / R {rear:.2f} m)')
            return
        if front <= self.a.stop_distance:
            self.get_logger().info(f'OBSTACLE IN FRONT at {front:.2f} m — STOPPING')
            self.halt(f'obstacle in front at {front:.2f} m')
            return
        if rear <= self.a.stop_distance:
            self.get_logger().info(f'OBSTACLE BEHIND at {rear:.2f} m — STOPPING')
            self.halt(f'obstacle behind at {rear:.2f} m')
            return

        self.driving = True
        self.publish(True)
        arrow = 'REV <<' if self.a.reverse else '>> FWD'
        self.get_logger().info(
            f'{arrow}   front {front:5.2f} m | rear {rear:5.2f} m   '
            f'(stop <= {self.a.stop_distance:.2f})',
            throttle_duration_sec=0.4)


# ------------------------------------------------------------------
class ScanCheck(Node):
    """Closest return per 45 deg sector, so you can identify which
    bearing is the AGV's forward direction."""

    def __init__(self):
        super().__init__('agv_scan_check')
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.n = 0

    def on_scan(self, msg):
        self.n += 1
        if self.n % 10:
            return
        buckets = {b: [] for b in range(-180, 180, 45)}
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = wrap180(math.degrees(msg.angle_min + i * msg.angle_increment))
            b = int(math.floor(ang / 45.0) * 45)
            if b in buckets:
                buckets[b].append(r)
        out = []
        for b in sorted(buckets):
            v = buckets[b]
            out.append(f'{b:+4d}°:{min(v):5.2f}m' if v else f'{b:+4d}°:  --  ')
        print('  '.join(out), flush=True)


# ------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(
        description='AGV straight drive with front AND rear obstacle stop')
    p.add_argument('--check', action='store_true',
                   help='print per-sector distances, then exit')
    p.add_argument('--dry-run', action='store_true',
                   help='monitor and report but never command motion')
    p.add_argument('--stop-distance', type=float, default=1.5,
                   help='metres — applies to BOTH cones (default 1.5)')
    p.add_argument('--forward-bearing', type=float, default=0.0,
                   help='scan bearing that points forward (deg)')
    p.add_argument('--rear-bearing', type=float, default=None,
                   help='override rear bearing (default: forward + 180)')
    p.add_argument('--reverse', action='store_true',
                   help='travel backwards (both cones still monitored)')
    p.add_argument('--cone-half-angle', type=float, default=15.0,
                   help='half-width of each cone (deg)')
    p.add_argument('--speed', type=float, default=0.5,
                   help='linear.x magnitude sent to the bridge')
    p.add_argument('--max-run', type=float, default=20.0,
                   help='hard time limit in seconds')
    p.add_argument('--ignore-closer-than', type=float, default=0.12,
                   help='ignore returns closer than this (AGV body)')
    p.add_argument('--noise-reject', type=int, default=2,
                   help='use the k-th smallest return instead of the min')
    a = p.parse_args()

    a.front_bearing = wrap180(a.forward_bearing)
    a.rear_bearing = (wrap180(a.rear_bearing) if a.rear_bearing is not None
                      else wrap180(a.forward_bearing + 180.0))
    a.speed = -abs(a.speed) if a.reverse else abs(a.speed)

    rclpy.init()

    if a.check:
        print('=' * 70)
        print('  SCAN DIRECTION CHECK')
        print('  Place the obstacle DIRECTLY IN FRONT; the sector whose')
        print('  reading changes is your forward bearing.')
        print('  Note what sits BEHIND too — the rear cone is monitored as well.')
        print('  Ctrl+C to exit.')
        print('=' * 70)
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

    print('=' * 70)
    print(f'  AGV OBSTACLE GUARD  —  travelling {"REVERSE" if a.reverse else "FORWARD"}')
    print(f'  stop distance : {a.stop_distance:.2f} m  (applies to BOTH cones)')
    print(f'  front cone    : {a.front_bearing:+.1f} deg  '
          f'+/-{a.cone_half_angle:.0f} deg')
    print(f'  rear cone     : {a.rear_bearing:+.1f} deg  '
          f'+/-{a.cone_half_angle:.0f} deg')
    print(f'  time limit    : {a.max_run:.0f} s')
    print('  Stops for an obstacle on EITHER side, whichever way it moves.')
    if a.dry_run:
        print('  *** DRY RUN — no motion will be commanded ***')
    print('=' * 70)

    if not a.dry_run:
        if input('Path clear both ends and power within reach? (yes/no): ')\
                .strip().lower() != 'yes':
            print('Aborting.')
            if rclpy.ok():
                rclpy.shutdown()
            return
        for i in (3, 2, 1):
            print(f'  starting in {i} …', flush=True)
            time.sleep(1.0)

    node = ObstacleGuard(a)

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
        print(f'\nRESULT: {node.stopped_reason or "shutdown"}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
