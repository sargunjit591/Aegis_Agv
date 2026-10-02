#!/usr/bin/env python3
"""Exercise the operator console in MOCK mode (no lgpio/spidev => no hardware).

Importing the console is safe: the web server only starts under __main__.
Its 50 Hz control thread does start on import, so this test runs in real
time (~20 s) against the simulated wheel (100 deg/s, coasts 0.12 s).

Needs flask.  Run:  python tests/test_console_mock.py
"""
import pathlib
import re
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / 'rpi_os_console'))

import builtins

pins, dac = {}, []          # last level per GPIO, [(t, volts)]
_real_print = builtins.print


def recorder(*args, **kw):
    line = ' '.join(str(a) for a in args)
    m = re.match(r'\[MOCK\] GPIO(\d+).*?-> (\d)', line)
    if m:
        pins[int(m.group(1))] = int(m.group(2))
    m = re.match(r'\[MOCK\] DAC0 -> ([\d.]+) V', line)
    if m:
        dac.append((time.monotonic(), float(m.group(1))))


builtins.print = recorder           # swallow the [MOCK] chatter, keep the data
import agv_console_angle as con     # noqa: E402
builtins.print = _real_print
con.print = recorder

hw = con.hw
failures = []


def check(ok, msg):
    print(('  ok   ' if ok else '  FAIL ') + msg)
    if not ok:
        failures.append(msg)


def wait(cond, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


beat = threading.Event()
beat.set()


def heartbeats():
    while True:
        if beat.is_set():
            con.last_heartbeat = time.monotonic()
        time.sleep(0.1)


threading.Thread(target=heartbeats, daemon=True).start()


def seek(deg):
    ok, msg = hw.seek_angle(deg)
    wait(lambda: hw.seek_target is None, 3.0)
    time.sleep(0.4)                  # let it coast to rest
    return ok, msg


print('== startup ==')
check(con.MOCK, 'console is in MOCK mode')
check(not hw.brake_engaged, 'brake starts RELEASED')
check(hw.current_v == con.THROTTLE_IDLE_V, 'throttle starts at idle 0.80 V')
check(wait(lambda: hw.angle_deg is not None, 1.0), 'control loop is ticking (angle read)')

print('== go-to-angle with look-ahead ==')
for target in (30.0, -20.0, 0.0):
    ok, msg = seek(target)
    check(ok and abs(hw.angle_deg - target) <= 4.0 and hw.steer_fault is None
          and hw.steer_state == 'stop',
          f'seek {target:+.0f} -> landed {hw.angle_deg:+.1f} deg (tol 4), '
          f'motor stopped, fault={hw.steer_fault}')
ok, msg = hw.seek_angle(50)
check(not ok and hw.seek_target is None, f'seek +50 refused: {msg!r}')
ok, msg = hw.seek_angle('abc')
check(not ok, f'seek "abc" refused: {msg!r}')

print('== +/-45 deg limit on manual steering, with look-ahead ==')
for cmd, sign in (('left', 1), ('right', -1)):
    hw.steer(cmd)
    stopped = wait(lambda: hw.steer_state == 'stop', 2.0)
    time.sleep(0.4)
    a = hw.angle_deg
    check(stopped and 40.0 <= sign * a <= 47.0,
          f'hold {cmd}: motor cut by the limit, wheel came to rest at {a:+.1f} deg '
          f'(want {sign * 40:+d}..{sign * 47:+d})')
    check(pins.get(con.STEER_LPWM) == 0 and pins.get(con.STEER_RPWM) == 0,
          f'  both steering pins inactive (GPIO{con.STEER_LPWM}=0, GPIO{con.STEER_RPWM}=0)')
    # Press again, five times. A press that starts a hair INSIDE the limit is
    # allowed, is cut on the next tick and coasts: one bounded overshoot (same
    # as the bridge). Once the wheel rests at or past the limit, every further
    # press is refused outright, so presses cannot walk the wheel away.
    for _ in range(5):
        hw.steer(cmd)
        time.sleep(0.4)
    a = hw.angle_deg
    hw.steer(cmd)
    check(hw.steer_state == 'stop' and 45.0 <= sign * a <= 61.0,
          f'  {cmd} pressed 5 more times: wheel rests at {a:+.1f} deg '
          f'(bounded: limit + one coast), next press refused')
back = hw.angle_deg
hw.steer('left')
time.sleep(0.15)
hw.steer('stop')
time.sleep(0.4)
check(hw.angle_deg > back + 5, f'steering back toward centre is allowed '
                              f'({back:+.1f} -> {hw.angle_deg:+.1f} deg)')

print('== timed steer, set-centre ==')
seek(0.0)
ok, secs = hw.timed('steer', 'right', 0.2)
check(ok and hw.steer_state == 'right' and pins.get(con.STEER_RPWM) == 1,
      f'timed right 0.2 s started (GPIO{con.STEER_RPWM}=1)')
time.sleep(0.7)
check(hw.steer_state == 'stop' and hw.angle_deg < -10,
      f'timed run stopped itself at {hw.angle_deg:+.1f} deg')
hw.steer('left')
ok, msg = hw.home_steering()
check(not ok, f'set-centre refused while steering: {msg!r}')
hw.steer('stop')
time.sleep(0.4)
ok, msg = hw.home_steering()
time.sleep(0.1)
check(ok and abs(hw.angle_deg) < 0.5, f'set-centre: this position is now {hw.angle_deg:+.1f} deg')

print('== steering max-run backstop (encoder absent) ==')
real_read, real_max = hw.read_angle, con.STEER_MAX_RUN_S
hw.read_angle = lambda: None
con.STEER_MAX_RUN_S = 0.5
hw.steer('left')
time.sleep(0.3)
check(hw.steer_state == 'left', 'no encoder: still running at 0.3 s (no angle limit available)')
check(wait(lambda: hw.steer_state == 'stop', 0.6) and 'continuously' in (hw.steer_fault or ''),
      f'motor cut after the max-run time: fault={hw.steer_fault!r}')
hw.read_angle, con.STEER_MAX_RUN_S = real_read, real_max
hw.clear_fault()
hw.home_steering()

print('== throttle slew, direction interlock, brake pulse ==')
dac.clear()
t0 = time.monotonic()
ok, msg = hw.set_voltage(2.0)
check(ok and wait(lambda: abs(hw.current_v - 2.0) < 0.01, 2.0), f'set 2.00 V: {msg}')
rise = dac[-1][0] - t0
steps = [b[1] - a[1] for a, b in zip(dac, dac[1:])]
check(max(steps) <= con.SLEW_STEP_V + 0.011 and rise >= 0.35,
      f'slew limited: biggest DAC step {max(steps):.2f} V (limit {con.SLEW_STEP_V:.2f}), '
      f'0.80->2.00 V took {rise:.2f} s (3 V/s => 0.40 s)')
ok, msg = hw.set_direction('reverse')
check(not ok and hw.direction == 'forward', f'direction change while driving refused: {msg!r}')
ok, msg = hw.set_voltage(9.9)
check(ok and hw.target_v == con.THROTTLE_MAX_V, f'9.9 V clamped: {msg}')
hw.set_speed_percent(0)
check(wait(lambda: hw.brake_engaged, 0.5) and pins.get(con.BRAKE_PIN) == con.BRAKE_ENGAGE_LEVEL,
      f'drive stopped -> brake pulse (GPIO{con.BRAKE_PIN}={con.BRAKE_ENGAGE_LEVEL})')
t_on = time.monotonic()
check(wait(lambda: not hw.brake_engaged, 2.6), 'brake auto-released')
held = time.monotonic() - t_on
check(1.8 <= held <= 2.2 and pins.get(con.BRAKE_PIN) == 1 - con.BRAKE_ENGAGE_LEVEL,
      f'  held {held:.2f} s (limit {con.BRAKE_MAX_ENGAGE_S} s), pin back to released level')
time.sleep(0.5)
check(not hw.brake_engaged and hw.current_v == con.THROTTLE_IDLE_V,
      'brake does not re-fire while stopped; throttle at idle')
ok, msg = hw.set_direction('reverse')
check(ok and pins.get(con.RELAY_DIR_PIN) == 1, f'direction change at idle allowed (GPIO{con.RELAY_DIR_PIN}=1)')
hw.set_direction('forward')
hw.set_brake_mode('apply')
t_on = time.monotonic()
check(hw.brake_engaged and wait(lambda: not hw.brake_engaged, 2.6)
      and time.monotonic() - t_on <= 2.2, 'manual APPLY also auto-releases within 2 s')

print('== E-stop ==')
hw.set_voltage(2.0)
hw.steer('left')
hw.grip('open')
wait(lambda: hw.current_v > 1.5, 2.0)
hw.estop()
t_on = time.monotonic()
check(hw.current_v == con.THROTTLE_IDLE_V and dac[-1][1] == con.THROTTLE_IDLE_V,
      'throttle to idle immediately (no slew)')
check(hw.steer_state == 'stop' and hw.grip_state == 'stop'
      and pins.get(con.STEER_LPWM) == 0 and pins.get(con.GRIP_RPWM) == 1,
      'steering and gripper off (gripper pin back to inactive HIGH)')
check(hw.brake_engaged, 'brake pulse fired')
hw.steer('left'); hw.grip('close')
check(hw.steer_state == 'stop' and hw.grip_state == 'stop'
      and not hw.set_voltage(2.0)[0] and not hw.seek_angle(10)[0]
      and not hw.set_brake_mode('release')[0] and not hw.timed('grip', 'open', 1)[0],
      'steer / grip / throttle / seek / brake-release / timed all refused while E-stopped')
check(wait(lambda: not hw.brake_engaged, 2.6) and hw.estopped
      and time.monotonic() - t_on <= 2.2,
      'E-stop brake pulse auto-released within 2 s while still E-stopped')
hw.estop_reset()
check(not hw.estopped and hw.current_v == con.THROTTLE_IDLE_V, 'reset: still at idle, nothing restarts')

print('== heartbeat watchdog ==')
hw.grip('open')
beat.clear()
t0 = time.monotonic()
check(wait(lambda: hw.grip_state == 'stop', 3.0) and con.watchdog_tripped,
      f'no heartbeat -> ALL STOP after {time.monotonic() - t0:.1f} s '
      f'(timeout {con.WATCHDOG_TIMEOUT_S} s)')
beat.set()
wait(lambda: not hw.brake_engaged, 2.6)

print('== HTTP routes (Flask test client, no server) ==')
c = con.app.test_client()
r = c.get('/')
check(r.status_code == 200 and b'btnHome' in r.data and b'estopBtn' in r.data, 'GET / serves the page')
s = c.get('/api/status').get_json()
check(s['mock'] is True and s['as5600_center_raw'] == 3791 and s['brake'] == 'released'
      and 'steer_rate_dps' in s, f'GET /api/status: centre {s["as5600_center_raw"]}, brake {s["brake"]}')
check(c.post('/api/steer', json={'cmd': 'sideways'}).status_code == 400, 'bad steer cmd -> 400')
check(c.post('/api/steer_to', json={'deg': 99}).get_json()['ok'] is False, 'steer_to 99 refused')
check(c.post('/api/steer_home').get_json()['ok'] is True, 'POST /api/steer_home ok')
check(c.post('/api/estop').get_json()['ok'] and hw.estopped, 'POST /api/estop latches')
check(c.post('/api/speed', json={'pct': 50}).get_json()['ok'] is False, 'speed refused while E-stopped')
c.post('/api/estop_reset')
check(c.post('/api/heartbeat').get_json()['watchdog_tripped'] is True, 'heartbeat reports the earlier watchdog trip')

hw.shutdown()
check(not hw.brake_engaged and hw.current_v == con.THROTTLE_IDLE_V, 'shutdown leaves brake released, throttle idle')

print()
print(f'{len(failures)} FAILED' if failures else 'ALL CHECKS PASSED')
sys.exit(1 if failures else 0)
