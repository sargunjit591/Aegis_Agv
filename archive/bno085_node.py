#!/usr/bin/env python3
"""
============================================================
PROJECT AEGIS — BNO085 HEADING NODE (UART-RVC)
============================================================
Reads the BNO085 in UART-RVC mode and publishes the AGV's
heading. RVC is gyro-referenced (it does NOT use the
magnetometer), which is exactly right in a warehouse full of
steel drums and next to 24 V motors.

WIRING (RVC mode)
    BNO085 VIN ──── Pi 3.3V        (pin 1)
    BNO085 GND ──── Pi GND         (pin 6)
    BNO085 SDA ──── Pi GPIO15/RXD  (pin 10)   SDA = serial TX in RVC
    BNO085 P0  ──── Pi 3.3V                   selects UART-RVC
    BNO085 PS1 ──── leave unconnected         must stay low

  Pi TX is not used — RVC is output only, the sensor just talks.

MOUNTING
  Heading does not care WHERE on the chassis it sits (angular
  rate is the same everywhere on a rigid body). It cares that
  the mount is RIGID, that Z is vertical, and that X points
  forward. If X cannot point forward, set mount_yaw_offset_deg.

PI SETUP (once)
    sudo nano /boot/firmware/config.txt      -> add:  enable_uart=1
    sudo systemctl disable --now serial-getty@ttyAMA0.service
    sudo nano /boot/firmware/cmdline.txt     -> remove console=serial0,...
    sudo reboot

TOPICS OUT
    /agv/heading   std_msgs/Float32   yaw in degrees, -180..180
    /imu/data      sensor_msgs/Imu    orientation + linear acceleration
                                      (RVC provides no gyro rates, so the
                                       angular_velocity covariance is -1)

RUN
    python3 bno085_node.py                       # ROS2 node
    python3 bno085_node.py --test                # plain terminal readout
    python3 bno085_node.py --ros-args -p port:=/dev/ttyAMA0

RVC PACKET (19 bytes, 100 Hz)
    0xAA 0xAA idx  yaw(2) pitch(2) roll(2)
    ax(2) ay(2) az(2)  reserved(3)  checksum(1)
  angles are int16 little-endian in 0.01 deg
  accelerations are int16 in milli-g
  checksum = sum(bytes 2..17) & 0xFF
============================================================
"""

import argparse
import math
import struct
import sys
import threading
import time

import serial

RVC_LEN = 19
RVC_HEADER = b'\xAA\xAA'


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


class RVCReader:
    """Parses the BNO085 UART-RVC stream. No external library needed."""

    def __init__(self, port, baud=115200, timeout=0.1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.lock = threading.Lock()
        self.yaw = None          # deg, raw from sensor
        self.pitch = None
        self.roll = None
        self.accel = (0.0, 0.0, 0.0)   # m/s^2
        self.stamp = 0.0
        self.packets = 0
        self.bad_checksum = 0
        self.resyncs = 0
        self._stop = threading.Event()
        self._buf = bytearray()

    def _parse(self, p):
        """p is a 19-byte packet starting with AA AA."""
        chk = sum(p[2:18]) & 0xFF
        if chk != p[18]:
            self.bad_checksum += 1
            return False
        yaw, pitch, roll, ax, ay, az = struct.unpack_from('<hhhhhh', p, 3)
        with self.lock:
            self.yaw = yaw / 100.0
            self.pitch = pitch / 100.0
            self.roll = roll / 100.0
            # milli-g -> m/s^2
            self.accel = (ax * 0.0098, ay * 0.0098, az * 0.0098)
            self.stamp = time.monotonic()
            self.packets += 1
        return True

    def run(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(64)
            except Exception:
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            self._buf.extend(chunk)

            while True:
                i = self._buf.find(RVC_HEADER)
                if i < 0:
                    # keep a byte in case a header straddles the boundary
                    if len(self._buf) > 1:
                        del self._buf[:-1]
                    break
                if i > 0:
                    del self._buf[:i]
                    self.resyncs += 1
                if len(self._buf) < RVC_LEN:
                    break
                pkt = bytes(self._buf[:RVC_LEN])
                if self._parse(pkt):
                    del self._buf[:RVC_LEN]
                else:
                    del self._buf[:2]        # bad packet: hunt for the next header
                    self.resyncs += 1

    def start(self):
        self.t = threading.Thread(target=self.run, daemon=True)
        self.t.start()

    def stop(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass

    def snapshot(self):
        with self.lock:
            return (self.yaw, self.pitch, self.roll, self.accel,
                    self.stamp, self.packets, self.bad_checksum, self.resyncs)


# ------------------------------------------------------------------
def quat_from_euler(yaw, pitch, roll):
    """radians -> (x, y, z, w)"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def run_test(port):
    print('=' * 62)
    print('  BNO085 UART-RVC TEST')
    print(f'  port {port} @ 115200')
    print('  Rotate the AGV and watch yaw. Ctrl+C to stop.')
    print('=' * 62)
    r = RVCReader(port)
    r.start()
    t0 = time.monotonic()
    try:
        while True:
            yaw, pitch, roll, acc, stamp, n, bad, rs = r.snapshot()
            age = time.monotonic() - stamp if stamp else 999
            rate = n / max(time.monotonic() - t0, 1e-3)
            if yaw is None:
                print('\r  no packets yet — check P0 is HIGH, PS1 floating, '
                      'and SDA -> Pi RX      ', end='', flush=True)
            else:
                print(f'\r  yaw {yaw:+7.2f}  pitch {pitch:+6.2f}  roll {roll:+6.2f}  '
                      f'| {rate:5.1f} Hz  age {age * 1000:4.0f} ms  '
                      f'bad {bad:3d}  resync {rs:3d}  ', end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
        yaw, _, _, _, _, n, bad, rs = r.snapshot()
        print(f'\n  packets {n}, bad checksums {bad}, resyncs {rs}')
        if n == 0:
            print('  NOTHING RECEIVED. Check in this order:')
            print('    1. P0 tied to 3.3 V (this selects RVC mode)')
            print('    2. PS1 unconnected and NOT bridged by a solder blob')
            print('    3. BNO085 SDA goes to the Pi RX pin (GPIO15, pin 10)')
            print('    4. enable_uart=1 set and the serial console disabled')
            print('    5. correct port — try /dev/ttyAMA0 and /dev/serial0')
        elif bad > n * 0.02:
            print('  Many bad checksums — noisy or long wiring.')
    finally:
        r.stop()


# ------------------------------------------------------------------
def run_node(argv):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    from sensor_msgs.msg import Imu

    class BNO085Node(Node):
        def __init__(self):
            super().__init__('bno085_heading')
            self.declare_parameter('port', '/dev/ttyAMA0')
            self.declare_parameter('frame_id', 'imu_link')
            self.declare_parameter('mount_yaw_offset_deg', 0.0)
            self.declare_parameter('invert_yaw', False)
            self.declare_parameter('publish_rate_hz', 50.0)

            port = self.get_parameter('port').value
            self.frame = self.get_parameter('frame_id').value
            self.offset = float(self.get_parameter('mount_yaw_offset_deg').value)
            self.invert = bool(self.get_parameter('invert_yaw').value)
            rate = float(self.get_parameter('publish_rate_hz').value)

            try:
                self.r = RVCReader(port)
            except Exception as e:
                self.get_logger().error(
                    f'cannot open {port}: {e}. Is enable_uart=1 set and the '
                    f'serial console disabled? Try /dev/serial0.')
                raise
            self.r.start()

            self.pub_h = self.create_publisher(Float32, '/agv/heading', 10)
            self.pub_i = self.create_publisher(Imu, '/imu/data', 10)
            self.create_timer(1.0 / rate, self.tick)
            self.warned = False

            self.get_logger().info(
                f'BNO085 RVC on {port} — offset {self.offset:+.1f} deg, '
                f'{"inverted" if self.invert else "normal"} sense')

        def tick(self):
            yaw, pitch, roll, acc, stamp, n, bad, rs = self.r.snapshot()
            if yaw is None:
                self.get_logger().warn(
                    'no RVC packets — check P0 high, PS1 floating, SDA to Pi RX',
                    throttle_duration_sec=3.0)
                return
            if time.monotonic() - stamp > 0.5:
                self.get_logger().error('BNO085 stream stalled',
                                        throttle_duration_sec=3.0)
                return

            h = wrap180((-yaw if self.invert else yaw) + self.offset)

            m = Float32()
            m.data = float(h)
            self.pub_h.publish(m)

            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = self.frame
            qx, qy, qz, qw = quat_from_euler(math.radians(h),
                                             math.radians(pitch),
                                             math.radians(roll))
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            imu.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
            # RVC does not report gyro rates
            imu.angular_velocity_covariance = [-1.0] + [0.0] * 8
            imu.linear_acceleration.x = acc[0]
            imu.linear_acceleration.y = acc[1]
            imu.linear_acceleration.z = acc[2]
            imu.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                  0.0, 0.1, 0.0,
                                                  0.0, 0.0, 0.1]
            self.pub_i.publish(imu)

        def destroy_node(self):
            self.r.stop()
            super().destroy_node()

    rclpy.init(args=argv)
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ------------------------------------------------------------------
if __name__ == '__main__':
    if '--test' in sys.argv:
        ap = argparse.ArgumentParser()
        ap.add_argument('--test', action='store_true')
        ap.add_argument('--port', default='/dev/ttyAMA0')
        a = ap.parse_args()
        run_test(a.port)
    else:
        run_node(sys.argv)
