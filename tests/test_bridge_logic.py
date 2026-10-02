#!/usr/bin/env python3
"""Simulated test of the motor bridge's control logic WITHOUT ROS2.

rclpy and the message packages are replaced by tiny stand-ins, the clock is
fake, and the wheel + AS5600 are simulated (100 deg/s, coasts 0.12 s after
the motor is cut). This checks the logic only. It is NOT a run of the real
node under ROS2 and proves nothing about hardware behaviour.

Run:  python tests/test_bridge_logic.py
"""
import json
import pathlib
import re
import sys
import types

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'ubuntu_ros2'))


# ---------------- stand-ins for rclpy and the message types ----------------
class Logger:
    def __init__(self):
        self.lines = []

    def _log(self, level, msg):
        self.lines.append((level, msg))

    def info(self, msg, **kw): self._log('info', msg)
    def warn(self, msg, **kw): self._log('warn', msg)
    def error(self, msg, **kw): self._log('error', msg)


class Param:
    def __init__(self, value):
        self.value = value


class Node:
    def __init__(self, name):
        self._params, self._log, self.timers, self.subs = {}, Logger(), [], {}

    def declare_parameter(self, name, default):
        if name in self._params:        # ROS2 raises on a second declaration
            raise RuntimeError(f'ParameterAlreadyDeclaredException: {name}')
        self._params[name] = default

    def get_parameter(self, name):
        return Param(self._params[name])   # KeyError = read but never declared

    def get_logger(self):
        return self._log

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_publisher(self, typ, topic, qos):
        pub = types.SimpleNamespace(sent=[])
        pub.publish = pub.sent.append
        return pub

    def create_timer(self, period, cb):
        self.timers.append((round(period, 4), cb.__name__))


def msg_class(**fields):
    def __init__(self, **kw):
        for k, v in fields.items():
            setattr(self, k, kw.get(k, v() if callable(v) else v))
    return type('Msg', (), {'__init__': __init__})


Vec = msg_class(x=0.0, y=0.0, z=0.0)
Twist = msg_class(linear=Vec, angular=Vec)
Bool = msg_class(data=False)
String = msg_class(data='')


def module(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


module('rclpy', init=lambda: None, spin=lambda n: None, ok=lambda: False, shutdown=lambda: None)
module('rclpy.node', Node=Node)
module('geometry_msgs')
module('geometry_msgs.msg', Twist=Twist)
module('std_msgs')
module('std_msgs.msg', Bool=Bool, String=String)

import agv_motor_bridge as br   # noqa: E402

# ---------------- fake clock, recorded outputs ----------------
clock = types.SimpleNamespace(t=1000.0)
br.time = types.SimpleNamespace(monotonic=lambda: clock.t)

pins, dac, events = {}, [], []      # last level per GPIO, [(t, V)], [(t, text)]


def recorder(*args, **kw):
    line = ' '.join(str(a) for a in args)
    events.append((clock.t, line))
    m = re.match(r'\[MOCK\] GPIO(\d+).*?-> (\d)', line)
    if m:
        pins[int(m.group(1))] = int(m.group(2))
    m = re.match(r'\[MOCK\] DAC0 -> ([\d.]+) V', line)
    if m:
        dac.append((clock.t, float(m.group(1))))


br.print = recorder
failures = []


def check(ok, msg):
    print(('  ok   ' if ok else '  FAIL ') + msg)
    if not ok:
        failures.append(msg)


# ---------------- simulated wheel + AS5600 ----------------
class Wheel:
    """Physical wheel angle (deg, + = left) behind a fake AS5600."""
    RATE, COAST = 100.0, 0.12

    def __init__(self, node, centre=3791, sign_ok=True):
        self.node, self.centre, self.sign_ok = node, centre, sign_ok
        self.deg, self.rate, self.coast_until = 0.0, 0.0, 0.0

    def step(self, dt):
        s = self.node.steer_state
        if s in ('left', 'right'):
            self.rate = self.RATE if s == 'left' else -self.RATE
            self.coast_until = clock.t + self.COAST
        elif clock.t >= self.coast_until:
            self.rate = 0.0
        self.deg += self.rate * dt

    def read_byte_data(self, addr, reg):
        # verified hardware: steering LEFT makes the raw count go DOWN
        counts = self.deg * 4096.0 / 360.0
        raw = int(round(self.centre + (-counts if self.sign_ok else counts))) % 4096
        return (raw >> 8) & 0x0F if reg == 0x0E else raw & 0xFF

    def close(self):
        pass


DT = 0.02
wheel = None


def run(node, secs, lin=None, ang=0.0):
    """Advance the simulation. lin=None means no /cmd_vel is published."""
    for _ in range(round(secs / DT)):
        clock.t += DT
        if wheel is not None:
            wheel.step(DT)
        if lin is not None:
            node.on_cmd_vel(Twist(linear=Vec(x=lin), angular=Vec(z=ang)))
        node.control_tick()


def angle(node):
    return None if node.steer_angle is None else br.math.degrees(node.steer_angle)


# =====================================================================
print('== construction ==')
node = br.AGVMotorBridge()
check(br.MOCK, 'bridge is in MOCK mode (no lgpio/spidev here)')
check(len(node._params) == len(br.PARAMS) == 40,
      f'{len(node._params)} parameters declared, none twice (the stand-in raises on a duplicate)')
check(node.CENTER_RAW == 3791 and node.CCW_POS is False
      and br.math.isclose(br.math.degrees(node.STEER_MAX), 45.0),
      'defaults with NO launch flags: centre 3791, ccw_positive False, limit 45 deg')
check(node.timers == [(0.02, 'control_tick'), (0.05, 'publish_status')],
      f'timers: control 50 Hz, status 20 Hz {node.timers}')
check(not node.brake_engaged and node.current_v == 0.80, 'starts with brake RELEASED, throttle idle')

print('== drive: slew, watchdog, brake pulse ==')
run(node, 1.0)                                   # idle, no commands
check(not dac, 'no /cmd_vel -> no DAC movement')
run(node, 1.0, lin=0.5)
steps = [b[1] - a[1] for a, b in zip(dac, dac[1:])]
check(node.current_v == 1.50 and pins.get(5) is None and max(steps) <= 0.151,
      f'forward: throttle slewed to {node.current_v:.2f} V, biggest step {max(steps):.2f} V (limit 0.15)')
t_last = clock.t
run(node, 0.4)                                   # commands stop arriving
check(node.current_v == 1.50, 'cmd_vel 0.4 s old: still driving (watchdog is 0.5 s)')
run(node, 0.3)
t_idle = next(t for t, v in dac if t > t_last and v == 0.80)
check(node.current_v == 0.80 and 0.5 <= t_idle - t_last <= 0.75,
      f'watchdog: throttle back at idle {t_idle - t_last:.2f} s after the last cmd_vel')
check(node.brake_engaged and pins.get(6) == 0, 'moving -> stopped: brake pulse (GPIO6=0)')
t_on = node.brake_engaged_at
run(node, 2.2)
t_off = next(t for t, e in events if t > t_on and 'RELEASE' in e)
check(not node.brake_engaged and pins.get(6) == 1 and t_off - t_on <= 2.03,
      f'brake auto-released after {t_off - t_on:.2f} s (limit 2.0 s)')
run(node, 3.0)
check(not node.brake_engaged, 'brake does not re-fire while parked')

print('== brake release lead, direction interlock ==')
node.on_brake_cmd(String(data='apply'))
check(node.brake_engaged, 'manual APPLY')
t0 = clock.t
dac.clear()
run(node, 1.0, lin=0.5)
t_rise = dac[0][0]
check(not node.brake_engaged and t_rise - t0 >= 0.35,
      f'drive command releases the brake; throttle waits {t_rise - t0:.2f} s (lead 0.35 s)')
events.clear()
run(node, 1.5, lin=-0.5)
i_relay = next(i for i, (t, e) in enumerate(events) if '(dir) -> 1' in e)
v_at_flip = float([e for t, e in events[:i_relay] if 'DAC0' in e][-1].split()[3])
check(node.direction == 'reverse' and v_at_flip <= 0.85 and node.current_v == 1.28,
      f'reverse: relay flipped only at {v_at_flip:.2f} V (idle), then drove at {node.current_v:.2f} V')

print('== E-stop ==')
node.on_gripper(String(data='open'))
node.on_estop(Bool(data=True))
t_on = clock.t
check(node.current_v == 0.80 and dac[-1][1] == 0.80, 'throttle to idle immediately (no slew)')
check(node.brake_engaged and node.grip_state == 'stop' and pins.get(13) == 1,
      'brake pulse fired, gripper stopped')
run(node, 1.0, lin=0.5, ang=1.0)
node.on_gripper(String(data='close'))
node.on_brake_cmd(String(data='release'))
check(node.current_v == 0.80 and node.steer_state == 'stop' and node.grip_state == 'stop'
      and node.brake_engaged,
      'cmd_vel, gripper and brake-release all ignored while E-stopped')
run(node, 1.2, lin=0.5)
t_off = next((t for t, e in events if t > t_on and 'RELEASE' in e), None)
check(not node.brake_engaged and node.estopped and t_off is not None and t_off - t_on <= 2.03,
      f'E-stop brake pulse auto-released after {(t_off or 0) - t_on:.2f} s while STILL E-stopped '
      f'(REV 4 held it energised until reset)')
check(node.current_v == 0.80, 'still at idle after the brake let go')
node.on_estop(Bool(data=False))
run(node, 0.5)
check(not node.estopped and node.current_v == 0.80, 'reset: the cmd_vel sent DURING the E-stop does not act afterwards')

print('== steering: limits with look-ahead, sign, watchdog ==')
wheel = Wheel(node)
node.as5600 = wheel
run(node, 0.1)
check(abs(angle(node)) < 0.1, f'wheel straight at raw 3791 reads {angle(node):+.1f} deg')
for name, ang, sign in (('left', 1.0, 1), ('right', -1.0, -1)):
    run(node, 2.0, lin=0.0, ang=ang)
    a = angle(node)
    check(node.steer_state == 'stop' and 40.0 <= sign * a <= 47.0,
          f'steer {name} held 2 s: stopped at the limit, wheel at {a:+.1f} deg '
          f'(want {sign * 40:+d}..{sign * 47:+d}; {name} reads '
          f'{"positive" if sign > 0 else "negative"})')
    check(abs(wheel.deg - a) < 0.2, f'  encoder agrees with the simulated wheel ({wheel.deg:+.1f})')
run(node, 0.2, lin=0.0, ang=1.0)
check(node.steer_state == 'left' and pins.get(16) == 1 and pins.get(19) == 0,
      'steering back toward centre is allowed (GPIO16=1 left, GPIO19=0)')
run(node, 0.7)
check(node.steer_state == 'stop' and pins.get(16) == 0, 'cmd_vel stale -> steering stopped by the watchdog')
st = json.loads((node.publish_status(), node.status_pub.sent[-1].data)[1])
check(st['steer_angle_deg'] == round(angle(node), 1) and st['steer_fault'] is None
      and st['throttle_v'] == 0.8 and st['as5600_center_raw'] == 3791,
      f'/agv/status: steer_angle_deg {st["steer_angle_deg"]}, throttle_v {st["throttle_v"]}, '
      f'{len(st)} fields')

print('== steering: re-home ==')
node.on_home_cmd(String(data='here'))
run(node, 0.1)
check(abs(angle(node)) < 0.1 and node.CENTER_RAW != 3791,
      f'/agv/steer_home "here": centre now raw {node.CENTER_RAW}, angle {angle(node):+.1f} deg')
wheel.deg = 0.0
node.CENTER_RAW = 3791
run(node, 0.3)

print('== steering: max-run backstop (no encoder) ==')
node.as5600, node.steer_angle = None, None
wheel = None
t0 = clock.t
run(node, 3.9, lin=0.0, ang=1.0)
check(node.steer_state == 'left', 'no encoder: still running at 3.9 s')
run(node, 0.3, lin=0.0, ang=1.0)
check(node.steer_state == 'stop', f'motor cut at {node.STEER_MAX_RUN:.0f} s of continuous running')
events.clear()
run(node, 5.0, lin=0.0, ang=1.0)
restarts = [e for t, e in events if e == '[MOCK] GPIO16 -> 1']
check(node.steer_state == 'stop' and not restarts,
      'command still held 5 s more: motor stays OFF (REV 4 restarted it after one tick)')
run(node, 0.1, lin=0.0, ang=0.0)
run(node, 0.1, lin=0.0, ang=1.0)
check(node.steer_state == 'left', 'after releasing the command, steering works again')
run(node, 0.7)

print('== steering: wrong encoder sign is contained ==')
wheel = Wheel(node, sign_ok=False)
node.as5600 = wheel
run(node, 4.0, lin=0.0, ang=1.0)
check(node.steer_state == 'stop' and abs(wheel.deg) < 110.0,
      f'sign inverted: >90 deg backstop stopped the wheel at {wheel.deg:+.1f} deg physical '
      f'(reads {angle(node):+.1f})')
run(node, 0.7)

print('== closed-loop steering ==')
wheel = Wheel(node)
node.as5600 = wheel
node.CLOSED_LOOP = True
run(node, 0.3)
target = br.math.degrees(br.math.atan2(0.9 * 0.2, 0.5))      # ang 0.2 rad/s at 0.5 m/s
run(node, 2.0, lin=0.5, ang=0.2)
check(abs(angle(node) - target) <= 14.0 and node.steer_fault is None,
      f'bicycle target {target:+.1f} deg, wheel at {angle(node):+.1f} deg, no fault '
      f'(bang-bang, no look-ahead on the target: it hunts)')
run(node, 0.7)
wheel = Wheel(node, sign_ok=False)
node.as5600 = wheel
run(node, 2.0, lin=0.5, ang=0.2)
check('RUNAWAY' in (node.steer_fault or '') and node.steer_state == 'stop',
      f'sign inverted: runaway guard tripped, wheel stopped at {wheel.deg:+.1f} deg')
node.CLOSED_LOOP = False
run(node, 0.7)

print('== gripper ==')
node.on_estop(Bool(data=True)); node.on_estop(Bool(data=False))      # clears the fault
node.on_gripper(String(data='close'))
check(node.grip_state == 'close' and pins.get(12) == 0 and pins.get(13) == 1,
      'close: GPIO12 active LOW, GPIO13 inactive')
run(node, 7.9)
check(node.grip_state == 'close', 'still closing at 7.9 s')
run(node, 0.3)
check(node.grip_state == 'stop' and pins.get(12) == 1, 'gripper auto-stop at 8 s')
node.on_gripper(String(data='sideways'))
check(node.grip_state == 'stop', 'bad gripper command ignored')

print('== shutdown ==')
node.on_brake_cmd(String(data='apply'))
node.shutdown_hw()
check(not node.brake_engaged and pins.get(6) == 1 and node.current_v == 0.80,
      'shutdown leaves the brake RELEASED (de-energised) and throttle idle')

print()
print(f'{len(failures)} FAILED' if failures else 'ALL CHECKS PASSED')
sys.exit(1 if failures else 0)
