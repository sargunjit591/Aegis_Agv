#!/usr/bin/env python3
"""
AGV wheel odometry: drive-motor hall count + AS5600 steering angle -> /odom.

DESIGN ONLY. Halls are not wired, m_per_count is not calibrated, and the ROS
node below has NEVER RUN (no rclpy where it was written). Only the
WheelOdometry core is tested: python tests/test_wheel_odom.py
Do not add it to a launch sequence until the halls are wired and calibrated.

Model (front-steer tricycle, base_link = rear-axle centre):
    d      = (count - last_count) * m_per_count      signed, rear-axle centre
    kappa  = tan(steer) / L                          + = left (REP-103)
    dtheta = d * kappa

Integration: each step is treated as ONE circular arc whose curvature is the
mean of the curvature at the previous and the current sample. The pose moves
along that arc's chord, which is exact for constant steering (any step size)
and second-order when the steering is slewing between samples.

Node behaviour
  steering None / stale / out of range
      "stale" = no /agv/status for steer_timeout_s; "out of range" =
      |angle| > steer_valid_deg (the wheel has no end stops; tan() of a
      runaway angle is meaningless). Before the first good angle and while
      the angle is bad, the node publishes NOTHING (no /odom, no TF) and
      keeps re-baselining the hall count, so distance covered at an unknown
      angle is discarded, never integrated at a guessed one. Consumers see
      the TF go stale and stop. On recovery the pose resumes from where it
      froze (odom drifts by the missed motion; SLAM's map->odom absorbs it).
  hall count jump
      A step longer than max_step_m between two ticks is rejected: pose
      unchanged, count re-baselined, v = omega = 0 for that tick, warning
      logged, `rejected` incremented. Covers a counter reset/restart or a
      corrupted read if counting later moves to a Pico over serial.

Publishes only /odom and TF odom -> base_link. Subscribes only /agv/status.
It replaces the static odom -> base_link publisher; never run both.
"""

import math


class WheelOdometry:
    """Pure-Python bicycle-model dead reckoning. No ROS, no GPIO.

    m_per_count may be negative if the hall sequence counts down when the
    vehicle drives forward (unknown until the halls are wired)."""

    def __init__(self, m_per_count, wheelbase_m=0.9, max_step_m=None):
        if not m_per_count or wheelbase_m <= 0:
            raise ValueError('m_per_count must be non-zero, wheelbase_m > 0')
        self.m_per_count = m_per_count
        self.wheelbase_m = wheelbase_m
        self.max_step_m = max_step_m      # None = no jump check
        self.x = self.y = self.theta = 0.0
        self.v = self.omega = 0.0
        self.rejected = 0                 # steps dropped by the jump check
        self._count = None                # None -> next update only baselines
        self._kappa = 0.0
        self._t = None

    def rebase(self):
        """Forget the last count: the next update() moves nothing."""
        self._count = None

    def update(self, count, steer_rad, t=None):
        kappa = math.tan(steer_rad) / self.wheelbase_m
        if self._count is None:
            d = 0.0
        else:
            d = (count - self._count) * self.m_per_count
            if self.max_step_m is not None and abs(d) > self.max_step_m:
                self.rejected += 1
                d = 0.0
        dtheta = d * 0.5 * (kappa + self._kappa)
        half = 0.5 * dtheta
        chord = d * (math.sin(half) / half if half else 1.0)
        self.x += chord * math.cos(self.theta + half)
        self.y += chord * math.sin(self.theta + half)
        self.theta = math.remainder(self.theta + dtheta, 2.0 * math.pi)
        if self._count is None:
            self.v = self.omega = 0.0
        elif t is not None and self._t is not None and t > self._t:
            self.v = d / (t - self._t)
            self.omega = dtheta / (t - self._t)
        self._count, self._kappa, self._t = count, kappa, t


def main():
    # ROS / GPIO imports are deferred so the core imports on any machine.
    import json
    import time

    import lgpio
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster

    from hall_odometry import HallCounter, find_header_chip

    class WheelOdomNode(Node):
        def __init__(self):
            super().__init__('agv_wheel_odom')
            p = self.declare_parameter
            mpc = p('m_per_count', 0.0).value          # from --calibrate
            wheelbase = p('wheelbase_m', 0.9).value
            max_step = p('max_step_m', 0.5).value      # 10 m/s at 20 Hz
            rate = p('rate_hz', 20.0).value            # = /agv/status rate
            self.timeout = p('steer_timeout_s', 0.25).value
            self.valid = math.radians(p('steer_valid_deg', 60.0).value)
            pins = p('hall_pins', [17, 27, 22]).value
            self.odom_frame = p('odom_frame', 'odom').value
            self.base_frame = p('base_frame', 'base_link').value
            if not mpc:
                raise RuntimeError('m_per_count not set - run '
                                   'hall_odometry.py --calibrate first')

            self.odo = WheelOdometry(mpc, wheelbase, max_step)
            self.steer = None                 # rad, + = left
            self.steer_t = 0.0
            self.chip = lgpio.gpiochip_open(find_header_chip())
            self.hall = HallCounter(self.chip, pins)

            self.pub = self.create_publisher(Odometry, 'odom', 10)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(String, 'agv/status', self.on_status, 10)
            self.create_timer(1.0 / rate, self.tick)

        def on_status(self, msg):
            try:
                a = json.loads(msg.data).get('steer_angle_deg')
            except (ValueError, AttributeError):
                a = None
            self.steer = math.radians(a) if isinstance(a, (int, float)) else None
            self.steer_t = time.monotonic()

        def tick(self):
            count = self.hall.snapshot()[0]
            now = time.monotonic()
            if (self.steer is None or now - self.steer_t > self.timeout
                    or not abs(self.steer) <= self.valid):     # NaN fails too
                self.odo.rebase()
                self.get_logger().warn(
                    'steering angle missing/stale/out of range - odometry '
                    'frozen, not publishing', throttle_duration_sec=2.0)
                return
            rejected = self.odo.rejected
            self.odo.update(count, self.steer, now)
            if self.odo.rejected != rejected:
                self.get_logger().warn(
                    f'hall count jump rejected (count now {count})')

            o = self.odo
            stamp = self.get_clock().now().to_msg()
            qz, qw = math.sin(o.theta / 2.0), math.cos(o.theta / 2.0)

            # Covariances are left at zero on purpose: there is no data to
            # justify numbers yet. Set them when the EKF step is tuned.
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = self.odom_frame
            m.child_frame_id = self.base_frame
            m.pose.pose.position.x = o.x
            m.pose.pose.position.y = o.y
            m.pose.pose.orientation.z = qz
            m.pose.pose.orientation.w = qw
            m.twist.twist.linear.x = o.v
            m.twist.twist.angular.z = o.omega
            self.pub.publish(m)

            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = o.x
            tf.transform.translation.y = o.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf.sendTransform(tf)

        def close(self):
            self.hall.cancel()
            lgpio.gpiochip_close(self.chip)

    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
