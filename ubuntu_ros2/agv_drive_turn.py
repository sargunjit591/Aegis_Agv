#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — DRIVE N m, THEN TURN N DEGREES
============================================================
SENSOR SPLIT (each source doing what it is actually good at)

  distance      lidar SLAM,  map -> base_link from slam_toolbox
  heading       BNO085 IMU,  /agv/imu  (JSON from agv_imu_node.py)
  wheel angle   AS5600,      /agv/status (from agv_motor_bridge.py)
  obstacles     raw /scan forward cone

  Lidar yaw smears while the vehicle rotates, so the turn is
  measured by the IMU. IMU distance drifts as t^2, so the
  straight leg is measured by the lidar. The IMU distance is
  shown alongside for comparison but is never used.

PHASES
  1. STRAIGHT       drive until the SLAM pose says distance reached
  2. STEER_TO_LOCK  creep forward while swinging the front wheel,
                    until the AS5600 confirms it reached lock
  3. ARC            full speed, wheel held at lock, until the IMU
                    yaw has swept turn_deg
  4. STRAIGHTEN     counter-steer until the wheel is back near 0
  5. DONE

SAFETY
  * front obstacle cone stops the run
  * Ctrl+C stops immediately; if this node dies the motor bridge
    watchdog stops the AGV within 0.5 s
  * per-phase timeout with a diagnostic message
  * stops if the pose, the scan or the IMU goes stale
  * 3 s countdown before motion

PREREQUISITES (all running)
  sllidar_ros2, both static transforms, slam_toolbox,
  agv_motor_bridge.py, agv_imu_node.py

USAGE
  python3 agv_drive_turn.py --dry-run
  python3 agv_drive_turn.py --distance 2 --turn-deg 45 --turn-dir right
  python3 agv_drive_turn.py --heading lidar      # if the IMU is down
============================================================
"""

import argparse
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriveTurn(Node):

    def __init__(self, a):
        super().__init__('agv_drive_turn')
        self.a = a
        self.phase = 'WAIT'
        self.result = None
        self.t_phase = time.monotonic()

        self.start_xy = None
        self.start_head = None
        self.travelled = 0.0
        self.imu_dist = None
        self.turned = 0.0

        self.front = None
        self.scan_t = 0.0
        self.imu = None
        self.imu_t = 0.0
        self.steer_deg = None
        self.steer_fault = None
        self.status_t = 0.0

        self.turn_sign = 1.0 if a.turn_dir == 'left' else -1.0
        self._straighten_sign0 = None   # wheel's sign when straightening began

        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.create_subscription(String, '/agv/status', self.on_status, 10)
        self.create_subscription(String, '/agv/imu', self.on_imu, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_zero = self.create_publisher(String, '/agv/imu_zero', 10)
        self.create_timer(1.0 / 20.0, self.tick)

    # ================= inputs =================
    def on_scan(self, msg):
        half = math.radians(self.a.cone_half_angle)
        centre = math.radians(self.a.forward_bearing)
        lo = max(msg.range_min, self.a.ignore_closer_than)
        vals = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < lo or r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            d = (ang - centre + math.pi) % (2 * math.pi) - math.pi
            if abs(d) <= half:
                vals.append(r)
        if vals:
            vals.sort()
            self.front = vals[min(self.a.noise_reject, len(vals) - 1)]
        else:
            self.front = None
        self.scan_t = time.monotonic()

    def on_status(self, msg):
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        self.steer_deg = s.get('steer_angle_deg')
        self.steer_fault = s.get('steer_fault')
        self.status_t = time.monotonic()

    def on_imu(self, msg):
        try:
            self.imu = json.loads(msg.data)
        except Exception:
            return
        d = self.imu.get('distance_m')
        self.imu_dist = abs(d) if d is not None else None
        self.imu_t = time.monotonic()

    def imu_ok(self):
        return (self.imu is not None and self.imu.get('ok')
                and (time.monotonic() - self.imu_t) < 0.5)

    def pose(self):
        try:
            tr = self.tf_buf.lookup_transform(
                self.a.map_frame, self.a.base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return None
        t = tr.transform.translation
        return t.x, t.y, yaw_from_quat(tr.transform.rotation)

    def heading(self, lidar_yaw):
        """Radians. IMU when available and selected, else lidar SLAM yaw."""
        if self.a.heading == 'imu' and self.imu_ok():
            return math.radians(self.imu['yaw_deg'])
        return lidar_yaw

    def sdeg(self):
        return '--' if self.steer_deg is None else f'{self.steer_deg:+.1f}'

    def idist(self):
        return '--' if self.imu_dist is None else f'{self.imu_dist:4.2f}'

    # ================= output =================
    def drive(self, lin, ang=0.0):
        m = Twist()
        m.linear.x = float(lin)
        m.angular.z = float(ang)
        self.pub.publish(m)

    def halt(self, result):
        if self.result is None:
            self.result = result
        self.phase = 'DONE'
        for _ in range(6):
            self.drive(0.0, 0.0)
            time.sleep(0.02)

    def goto(self, phase):
        self.phase = phase
        self.t_phase = time.monotonic()
        if phase == 'STRAIGHTEN':
            self._straighten_sign0 = None
        self.get_logger().info(f'--> {phase}')

    # ================= loop =================
    def tick(self):
        if self.phase == 'DONE':
            self.drive(0.0, 0.0)
            return

        now = time.monotonic()
        moving = self.phase in ('STRAIGHT', 'STEER_TO_LOCK', 'ARC')

        if moving:
            if now - self.scan_t > 0.5:
                self.halt('scan stale'); return
            if self.front is None:
                self.halt('no returns in the front cone'); return
            if self.front <= self.a.stop_distance:
                self.get_logger().warn(f'OBSTACLE at {self.front:.2f} m')
                self.halt(f'obstacle at {self.front:.2f} m'); return
            if self.a.heading == 'imu' and not self.imu_ok():
                self.halt('IMU went stale mid-run'); return
        if self.steer_fault:
            self.halt(f'steering fault: {self.steer_fault}'); return

        if moving and (now - self.t_phase) > self.a.phase_timeout:
            msg = f'{self.phase} timed out'
            if self.phase == 'STEER_TO_LOCK':
                msg += (f' — wheel only reached {self.sdeg()} of '
                        f'{self.a.steer_lock_deg:.0f} deg. Check the bridge '
                        f'log for "steering limit".')
            self.get_logger().error(msg)
            self.halt(msg); return

        p = self.pose()

        # ---------- wait for every source ----------
        if self.phase == 'WAIT':
            self.drive(0.0, 0.0)
            missing = []
            if p is None:
                missing.append(f'{self.a.map_frame}->{self.a.base_frame} (slam_toolbox)')
            if self.a.heading == 'imu' and not self.imu_ok():
                missing.append('/agv/imu (agv_imu_node.py)')
            if self.steer_deg is None:
                missing.append('/agv/status steering angle (agv_motor_bridge.py)')
            if missing:
                self.get_logger().warn('waiting for: ' + ', '.join(missing),
                                       throttle_duration_sec=2.0)
                if (now - self.t_phase) > 20.0:
                    self.halt('missing: ' + ', '.join(missing))
                return
            z = String(); z.data = 'zero'
            self.imu_zero.publish(z)      # heading and IMU distance start here
            self.start_xy = (p[0], p[1])
            self.get_logger().info(
                f'ready — x {p[0]:.2f} y {p[1]:.2f}, wheel {self.sdeg()} deg, '
                f'IMU yaw {self.imu["yaw_deg"]:+.1f}'
                if self.imu_ok() else 'ready')
            self.goto('STRAIGHT')
            return

        if p is None:
            self.halt('pose lost'); return
        x, y, lidar_yaw = p

        # ---------- 1. straight ----------
        if self.phase == 'STRAIGHT':
            self.travelled = math.hypot(x - self.start_xy[0],
                                        y - self.start_xy[1])
            if self.travelled >= self.a.distance - self.a.distance_tol:
                self.get_logger().info(f'travelled {self.travelled:.2f} m')
                self.start_head = self.heading(lidar_yaw)
                self.goto('STEER_TO_LOCK')
                return
            self.drive(self.a.speed, 0.0)
            self.get_logger().info(
                f'STRAIGHT {self.travelled:4.2f}/{self.a.distance:.2f} m '
                f'[imu {self.idist()}] | wheel {self.sdeg()} | '
                f'front {self.front:4.2f} m', throttle_duration_sec=0.5)
            return

        # ---------- 2. steer to lock ----------
        if self.phase == 'STEER_TO_LOCK':
            self.drive(self.a.creep_speed, self.turn_sign * self.a.steer_cmd)
            self.turned = abs(wrap_pi(self.heading(lidar_yaw) - self.start_head))

            if self.a.steer_time > 0.0:
                reached = (now - self.t_phase) >= self.a.steer_time
            elif self.steer_deg is not None:
                reached = (self.steer_deg * self.turn_sign) >= self.a.steer_lock_deg
            else:
                reached = False

            self.get_logger().info(
                f'STEER_TO_LOCK wheel {self.sdeg()} -> '
                f'{self.turn_sign * self.a.steer_lock_deg:+.0f} | '
                f'yaw {math.degrees(self.turned):4.1f} | '
                f'front {self.front:4.2f} m', throttle_duration_sec=0.5)

            if reached:
                self.get_logger().info(f'wheel at lock ({self.sdeg()} deg)')
                self.goto('ARC')
            return

        # ---------- 3. arc ----------
        if self.phase == 'ARC':
            self.turned = abs(wrap_pi(self.heading(lidar_yaw) - self.start_head))
            if self.turned >= math.radians(self.a.turn_deg - self.a.turn_tol_deg):
                self.get_logger().info(
                    f'heading swept {math.degrees(self.turned):.1f} deg')
                self.goto('STRAIGHTEN')
                return
            self.drive(self.a.turn_speed, self.turn_sign * self.a.steer_cmd)
            self.get_logger().info(
                f'ARC yaw {math.degrees(self.turned):5.1f}/{self.a.turn_deg:.0f} '
                f'| wheel {self.sdeg()} | front {self.front:4.2f} m',
                throttle_duration_sec=0.5)
            return

        # ---------- 4. straighten ----------
        if self.phase == 'STRAIGHTEN':
            done = False

            if self.a.steer_time > 0.0:
                done = (now - self.t_phase) >= self.a.steer_time
            elif self.steer_deg is not None:
                # Latch which side of centre we started on.
                if self._straighten_sign0 is None:
                    self._straighten_sign0 = (1.0 if self.steer_deg >= 0 else -1.0)
                # Two stop conditions. The band alone is not enough: the wheel
                # moves ~100 deg/s, so even at 20 Hz status it covers 5 deg per
                # sample and a narrow window can be stepped over entirely.
                # Crossing zero is unmissable, so use that as well.
                inside = abs(self.steer_deg) <= self.a.centre_tol_deg
                crossed = (self.steer_deg * self._straighten_sign0) < 0.0
                done = inside or crossed
                if crossed and not inside:
                    self.get_logger().info(
                        f'wheel crossed centre at {self.sdeg()} deg — stopping')

            if (now - self.t_phase) > self.a.straighten_timeout:
                done = True
                self.get_logger().warn('straighten timed out — stopping')

            # stop commanding the moment we are done, then coast
            self.drive(0.0, 0.0 if done else -self.turn_sign * self.a.steer_cmd)

            self.get_logger().info(f'STRAIGHTEN wheel {self.sdeg()} -> 0',
                                   throttle_duration_sec=0.3)
            if done:
                self.halt(f'complete: {self.travelled:.2f} m then '
                          f'{math.degrees(self.turned):.1f} deg')
            return


# ================= main =================
def main():
    p = argparse.ArgumentParser(
        description='AGV: drive a distance, then turn the whole vehicle')
    p.add_argument('--distance', type=float, default=2.0)
    p.add_argument('--turn-deg', type=float, default=45.0,
                   help='HEADING change of the whole AGV')
    p.add_argument('--turn-dir', choices=('left', 'right'), default='right')
    p.add_argument('--heading', choices=('imu', 'lidar'), default='imu',
                   help='what measures the turn (default imu)')
    p.add_argument('--steer-lock-deg', type=float, default=35.0,
                   help='keep below the bridge steer_max_deg (45)')
    p.add_argument('--steer-time', type=float, default=0.0,
                   help='>0 = ignore the encoder, steer for this many seconds')
    p.add_argument('--creep-speed', type=float, default=0.35)
    p.add_argument('--speed', type=float, default=0.5)
    p.add_argument('--turn-speed', type=float, default=0.5)
    p.add_argument('--steer-cmd', type=float, default=1.0)
    p.add_argument('--forward-bearing', type=float, default=-180.0)
    p.add_argument('--stop-distance', type=float, default=1.0)
    p.add_argument('--cone-half-angle', type=float, default=15.0)
    p.add_argument('--distance-tol', type=float, default=0.10)
    p.add_argument('--turn-tol-deg', type=float, default=3.0)
    p.add_argument('--centre-tol-deg', type=float, default=6.0,
                   help='stop straightening inside this band, or '
                        'as soon as the wheel crosses centre')
    p.add_argument('--phase-timeout', type=float, default=60.0)
    p.add_argument('--straighten-timeout', type=float, default=6.0)
    p.add_argument('--map-frame', default='map')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--ignore-closer-than', type=float, default=0.12)
    p.add_argument('--noise-reject', type=int, default=2)
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()

    if a.dry_run:
        a.speed = a.turn_speed = a.creep_speed = 0.0
        a.steer_cmd = 0.0

    print('=' * 70)
    print('  AGV MISSION — STRAIGHT, THEN TURN THE WHOLE VEHICLE')
    print(f'  straight : {a.distance:.2f} m   measured by LIDAR SLAM')
    print(f'  turn     : {a.turn_deg:.0f} deg {a.turn_dir}   measured by '
          f'{"BNO085 /agv/imu" if a.heading == "imu" else "slam_toolbox yaw"}')
    print(f'  wheel    : AS5600 via /agv/status, lock at '
          f'{a.steer_lock_deg:.0f} deg')
    print(f'  obstacle : cone {a.forward_bearing:+.0f} deg, '
          f'stop <= {a.stop_distance:.2f} m')
    if a.dry_run:
        print('  *** DRY RUN — reports sensors, commands no motion ***')
    print('=' * 70)

    rclpy.init()

    if not a.dry_run:
        if input('Clear space for the straight AND the arc? (yes/no): ')\
                .strip().lower() != 'yes':
            print('Aborting.')
            if rclpy.ok():
                rclpy.shutdown()
            return
        for i in (3, 2, 1):
            print(f'  starting in {i} …', flush=True)
            time.sleep(1.0)

    node = DriveTurn(a)

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
        print(f'\nRESULT: {node.result or "shutdown"}')
        print(f'  distance : lidar {node.travelled:.2f} m | '
              f'imu {node.idist()} m')
        print(f'  heading  : {math.degrees(node.turned):.1f} deg '
              f'(target {a.turn_deg:.0f}, measured by {a.heading})')
        print(f'  wheel    : {node.sdeg()} deg')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
