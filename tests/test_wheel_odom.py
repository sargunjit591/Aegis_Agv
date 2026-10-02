#!/usr/bin/env python3
"""
Mock harness for WheelOdometry. Pure stdlib:  python tests/test_wheel_odom.py
Exit 0 = every check passed AND every deliberately-wrong model was caught.

Every path is fed as INTEGER cumulative hall counts. The analytic answer is
evaluated at d = final_count * m_per_count, so count quantisation is not an
error source and the tolerances can be tight:

  TIGHT = 1e-9 (m, rad, m/s, rad/s)
      Constant-steer paths are circles; the core's chord step is exact for a
      circle at any step size, so the only error is float round-off
      (~1e-13 over a few hundred steps). 1e-9 is 4 orders above that and
      7 orders below one hall count (11.4 mm), so nothing real hides in it.
  Clothoid position = S * |dkappa/ds| * h^2 / 12   (computed, ~0.35 mm)
      With steering slewing, each step replaces a linearly varying curvature
      by its mean. Heading at step ends is still exact (TIGHT); inside a
      step it is off by at most kappa' u (h-u) / 2, which integrates to
      kappa' h^3 / 12 of position per step, S/h steps. That is the bound.

Expected poses come from circle geometry about the ICR (R = L / tan(steer))
and, for the sweep, from the Fresnel power series - not from re-running an
integrator.
"""

import math
import pathlib
import sys
from itertools import cycle

SRC = pathlib.Path(__file__).resolve().parent.parent / 'ubuntu_ros2'
sys.path.insert(0, str(SRC))

from agv_wheel_odom import WheelOdometry  # noqa: E402

MPC = 0.0114            # example value from the brief; not a calibration
L = 0.9
TIGHT = 1e-9
STEPS = (1, 4, 2, 7, 3)  # irregular counts per update: result must not care


# ---------------- analytic references ----------------
def arc(d, steer):
    """Pose after signed distance d on a circle, from (0, 0, 0)."""
    if steer == 0.0:
        return d, 0.0, 0.0
    r = L / math.tan(steer)             # signed: + = centre on the left
    return r * math.sin(d / r), r * (1.0 - math.cos(d / r)), d / r


def then(a, b):
    """Pose b expressed in the frame at the end of pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def fresnel(a, u):
    """(int_0^u cos(a t^2) dt, int_0^u sin(a t^2) dt) by power series."""
    c = s = 0.0
    for n in range(30):
        c += (-1) ** n * a ** (2 * n) * u ** (4 * n + 1) \
            / (math.factorial(2 * n) * (4 * n + 1))
        s += (-1) ** n * a ** (2 * n + 1) * u ** (4 * n + 3) \
            / (math.factorial(2 * n + 1) * (4 * n + 3))
    return c, s


def clothoid(k0, s_total):
    """End pose for kappa(s) = k0 (1 - 2 s / S): steering sweeps through 0.
    theta(s) = k0 (s - s^2/S) = th_m - a u^2, u = s - S/2, a = k0/S."""
    a, th_m = k0 / s_total, k0 * s_total / 4.0
    c, s = fresnel(a, s_total / 2.0)
    return (2.0 * (math.cos(th_m) * c + math.sin(th_m) * s),
            2.0 * (math.sin(th_m) * c - math.cos(th_m) * s), 0.0)


# ---------------- driving the model under test ----------------
def drive(odo, start, counts, steer, seen=None):
    """Advance `counts` (signed) hall counts at a fixed steering angle.
    seen(signed counts done) is called after every update."""
    sign = 1 if counts >= 0 else -1
    done, steps = 0, cycle(STEPS)
    odo.update(start, steer)
    while done < abs(counts):
        done = min(done + next(steps), abs(counts))
        odo.update(start + sign * done, steer)
        if seen:
            seen(sign * done)
    return start + counts


def pose_err(odo, exp):
    pos = math.hypot(odo.x - exp[0], odo.y - exp[1])
    ang = abs(math.remainder(odo.theta - exp[2], 2.0 * math.pi))
    wrapped = -math.pi <= odo.theta <= math.pi
    return pos, ang, wrapped


def pose_check(name, odo, exp, pos_tol=TIGHT):
    pos, ang, wrapped = pose_err(odo, exp)
    ok = pos <= pos_tol and ang <= TIGHT and wrapped
    return (name, ok, f'pos err {pos:.2e} m (tol {pos_tol:.1e})  '
                      f'yaw err {ang:.2e} rad (tol {TIGHT:.0e})')


def analytic_checks(make):
    """All checks against closed-form answers. make(mpc, L) -> model."""
    out = []
    deg = math.radians

    def const(name, counts, steer):
        # compared at EVERY sample, so an error that cancels by the end of
        # the path is still seen
        o = make(MPC, L)
        worst = [0.0, 0.0, True]

        def seen(done):
            pos, ang, wrapped = pose_err(o, arc(done * MPC, steer))
            worst[:] = max(worst[0], pos), max(worst[1], ang), worst[2] and wrapped
        drive(o, 0, counts, steer, seen)
        ok = worst[0] <= TIGHT and worst[1] <= TIGHT and worst[2]
        out.append((name, ok, f'worst pos err {worst[0]:.2e} m  '
                              f'worst yaw err {worst[1]:.2e} rad (tol {TIGHT:.0e})'))

    const('straight forward 11.4 m', 1000, 0.0)
    const('arc left 30 deg, forward', 273, deg(30))
    const('arc right 20 deg, forward', 400, deg(-20))
    const('arc left 45 deg past 180 deg (yaw wrap)', 316, deg(45))
    const('straight reverse 5.7 m', -500, 0.0)
    const('arc reverse, wheel left 30 deg', -273, deg(30))
    const('arc reverse, wheel right 20 deg', -400, deg(-20))

    # S-curve of two arcs; the wheel swaps side while the count is static
    o = make(MPC, L)
    c = drive(o, 0, 150, deg(30))
    drive(o, c, 200, deg(-25))
    out.append(pose_check('S-curve: arc left then arc right', o,
                          then(arc(150 * MPC, deg(30)),
                               arc(200 * MPC, deg(-25)))))

    # steering slews through zero WHILE moving (and the same in reverse)
    for name, k0, n, h in (('sweep left->right while moving', 1, 350, 5),
                           ('sweep right->left while moving', -1, 350, 5),
                           ('sweep left->right in reverse', 1, -350, -5)):
        k0 *= math.tan(deg(30)) / L
        s_total = n * MPC               # signed
        o = make(MPC, L)
        mid = None
        for cnt in range(0, n + h, h):
            kappa = k0 * (1.0 - 2.0 * cnt * MPC / s_total)
            o.update(cnt, math.atan(L * kappa))
            if cnt == n // 2:
                mid = abs(math.remainder(o.theta - k0 * s_total / 4.0,
                                         2.0 * math.pi))
        tol = abs(s_total) * abs(2.0 * k0 / s_total) * (h * MPC) ** 2 / 12.0
        out.append(pose_check(name, o, clothoid(k0, s_total), tol))
        out.append((name + ': peak yaw at midpoint', mid <= TIGHT,
                    f'yaw err {mid:.2e} rad (tol {TIGHT:.0e})'))

    # velocities: 5 counts every 50 ms on a left arc, forward then reverse
    for name, step in (('v, omega forward', 5), ('v, omega reverse', -5)):
        o = make(MPC, L)
        for i in range(20):
            o.update(i * step, deg(30), i * 0.05)
        v = step * MPC / 0.05
        ev = abs(o.v - v)
        ew = abs(o.omega - v * math.tan(deg(30)) / L)
        out.append((name, ev <= TIGHT and ew <= TIGHT,
                    f'v err {ev:.2e} m/s  omega err {ew:.2e} rad/s '
                    f'(tol {TIGHT:.0e})'))
    return out


def behaviour_checks():
    """Documented non-geometric behaviour of the real core."""
    out = []
    o = WheelOdometry(MPC, L)
    o.update(12345, 0.3, 0.0)
    out.append(('first update only baselines the count',
                (o.x, o.y, o.theta, o.v, o.omega) == (0, 0, 0, 0, 0), ''))

    o = WheelOdometry(MPC, L, max_step_m=0.5)
    o.update(0, 0.0, 0.0)
    o.update(10, 0.0, 0.1)
    o.update(10010, 0.0, 0.2)           # 114 m in one tick: impossible
    held = (abs(o.x - 10 * MPC) <= TIGHT and o.rejected == 1
            and o.v == 0.0 and o.omega == 0.0)
    o.update(10020, 0.0, 0.3)
    out.append(('count jump rejected, then resumes from new baseline',
                held and abs(o.x - 20 * MPC) <= TIGHT and o.rejected == 1,
                f'x {o.x:.4f} m, rejected {o.rejected}'))

    o = WheelOdometry(MPC, L)
    o.update(0, 0.0)
    o.update(10, 0.0)
    o.rebase()
    o.update(500, 0.5)                  # motion while steering was unknown
    o.update(510, 0.0)                  # mean of tan(0.5), tan(0) curvature
    exp = then((10 * MPC, 0.0, 0.0),
               arc(10 * MPC, math.atan(0.5 * math.tan(0.5))))
    out.append(pose_check('rebase() discards the gap, pose continues', o, exp))

    try:
        WheelOdometry(0.0)
        bad = False
    except ValueError:
        bad = True
    out.append(('m_per_count = 0 is refused', bad, ''))

    src = (SRC / 'agv_wheel_odom.py').read_text(encoding='utf-8')
    banned = [w for w in ('cmd_vel', 'agv/brake', 'gripper_cmd', 'agv/estop')
              if w in src]
    out.append(('node source: one publisher (/odom), no actuator topics',
                not banned and src.count('create_publisher') == 1,
                f'found {banned}'))
    return out


# ---------------- deliberately wrong models ----------------
class Wrong:
    """Bicycle-model integrators with one planted error each."""

    def __init__(self, mpc, wheelbase, bug):
        self.mpc, self.L, self.bug = mpc, wheelbase, bug
        self.x = self.y = self.theta = self.v = self.omega = 0.0
        self.c = self.k = self.t = None

    def update(self, count, steer, t=None):
        bug = self.bug
        if bug == 'steer sign flipped':
            steer = -steer
        if bug == 'count sign flipped':
            count = -count
        k = math.tan(steer) / self.L
        if bug == 'sin(steer) instead of tan':
            k = math.sin(steer) / self.L
        if bug == 'steer instead of tan(steer)':
            k = steer / self.L
        if bug == 'track 0.8 m used as wheelbase':
            k = math.tan(steer) / 0.8
        if self.c is None:
            self.c, self.k, self.t = count, k, t
            return
        d = (count - self.c) * self.mpc
        km = 0.5 * (k + self.k)
        if bug == 'latest steer only (no averaging)':
            km = k
        if bug == 'previous steer only':
            km = self.k
        if bug == 'count taken as front-wheel distance':
            km *= math.cos(steer)
            d *= math.cos(steer)
        dth = d * km
        mid = self.theta + 0.5 * dth
        chord = d * (math.sin(0.5 * dth) / (0.5 * dth) if dth else 1.0)
        if bug == 'Euler: step along the old heading':
            mid, chord = self.theta, d
        if bug == 'midpoint heading without chord shortening':
            chord = d
        self.x += chord * math.cos(mid)
        self.y += chord * math.sin(mid)
        self.theta += dth
        if bug != 'yaw not wrapped':
            self.theta = math.remainder(self.theta, 2.0 * math.pi)
        if t is not None and self.t is not None and t > self.t:
            self.v, self.omega = d / (t - self.t), dth / (t - self.t)
            if bug == 'omega sign flipped':
                self.omega = -self.omega
            if bug == 'v is unsigned speed':
                self.v = abs(self.v)
        self.c, self.k, self.t = count, k, t


BUGS = ('steer sign flipped', 'count sign flipped',
        'sin(steer) instead of tan', 'steer instead of tan(steer)',
        'track 0.8 m used as wheelbase', 'latest steer only (no averaging)',
        'previous steer only', 'count taken as front-wheel distance',
        'Euler: step along the old heading',
        'midpoint heading without chord shortening', 'yaw not wrapped',
        'omega sign flipped', 'v is unsigned speed')


def main():
    failed = 0
    for name, ok, detail in (analytic_checks(WheelOdometry)
                             + behaviour_checks()):
        failed += not ok
        print(f'{"PASS" if ok else "FAIL"}  {name}  {detail}'.rstrip())

    # the unbugged reference must pass, or "caught" below means nothing
    ref = [n for n, ok, _ in analytic_checks(lambda m, w: Wrong(m, w, None))
           if not ok]
    failed += bool(ref)
    print(f'{"FAIL" if ref else "PASS"}  mutant scaffold with no bug passes '
          f'all analytic checks')
    for bug in BUGS:
        res = analytic_checks(lambda m, w, b=bug: Wrong(m, w, b))
        tripped = sum(not ok for _, ok, _ in res)
        failed += not tripped
        print(f'{"PASS" if tripped else "FAIL"}  wrong model caught: {bug}  '
              f'({tripped}/{len(res)} checks trip)')

    print(f'\n{"ALL PASSED" if not failed else f"{failed} FAILED"}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
