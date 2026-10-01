#!/usr/bin/env python3
"""
============================================================
BLDC HALL ODOMETRY — COUNT, DIRECTION, CALIBRATION
============================================================
Turns your drive motor's existing hall sensors into a wheel
encoder. No new sensor hardware — you are reading the three
signals the controller already uses for commutation.

WIRING (see the notes that came with this file)
    motor hall HA ──10k──┬── GPIO17 ──┐
                       20k            │
                         └── GND      │
    motor hall HB ──10k──┬── GPIO27   │  all three grounds tie
                       20k            │  to the controller's
                         └── GND      │  hall GND wire
    motor hall HC ──10k──┬── GPIO22 ──┘
                       20k
                         └── GND

  * TAP IN PARALLEL. Never cut the hall cable — the controller
    needs those signals to commutate.
  * The divider is mandatory: halls are 5 V, Pi GPIO is 3.3 V
    and is NOT 5 V tolerant.
  * Do not draw power from the hall +5 V wire.

HOW IT COUNTS
  Three halls give six unique states per electrical cycle. Every
  state transition is a count. Counts per mechanical revolution
  = 6 x pole pairs, so a typical hub motor yields 90+ counts per
  wheel turn — plenty for centimetre-level distance.

  Direction comes from the state SEQUENCE, so it stays correct
  even if the AGV rolls backwards on a slope with no power.

THREE MODES
  --check       verify wiring: shows live hall states and counts
  --calibrate   roll a measured distance, get metres per count
  --odom        live distance readout using a saved calibration

TYPICAL SESSION
  1. python3 hall_odometry.py --check
        Rotate the wheel by hand. All three states must toggle
        and the count must rise. If one never changes, that
        divider or tap is bad.
  2. python3 hall_odometry.py --calibrate --distance 5.0
        Push the AGV exactly 5 m along a tape measure.
  3. python3 hall_odometry.py --odom --m-per-count 0.0114

DEPENDENCIES
  pip install lgpio --break-system-packages
============================================================
"""

import argparse
import math
import signal
import sys
import threading
import time

import lgpio

# ---- default pins (free in the AGV's current map) ----
HALL_A = 17
HALL_B = 27
HALL_C = 22


def find_header_chip():
    """gpiochip0 on Raspberry Pi OS, gpiochip4 on Ubuntu 24.04."""
    for n in range(8):
        try:
            h = lgpio.gpiochip_open(n)
        except Exception:
            continue
        try:
            info = lgpio.gpio_get_chip_info(h)
            if isinstance(info, (list, tuple)) and len(info) >= 4:
                lines, label = info[1], str(info[3])
            else:
                lines, label = getattr(info, 'lines', 0), str(getattr(info, 'label', ''))
            if 'rp1' in label.lower() or lines == 54:
                return n
        except Exception:
            pass
        finally:
            try:
                lgpio.gpiochip_close(h)
            except Exception:
                pass
    return 0


class HallCounter:
    """Counts hall state transitions and decodes direction.

    The six valid states in order form a repeating ring. Moving to
    the next state in the ring is +1, the previous is -1. An
    invalid jump (two states at once) means an edge was missed."""

    # the six valid 3-bit hall states, in commutation order
    RING = [0b001, 0b011, 0b010, 0b110, 0b100, 0b101]

    def __init__(self, h, pins):
        self.h = h
        self.pins = pins
        self.lock = threading.Lock()
        self.count = 0            # signed, direction-aware
        self.transitions = 0      # unsigned, every edge
        self.invalid = 0          # illegal states (000 / 111)
        self.missed = 0           # jumps of more than one ring step
        self.last_state = None
        self.last_edge_t = None
        self.max_rate = 0.0
        self.idx = {s: i for i, s in enumerate(self.RING)}
        self.cbs = []

        for p in pins:
            lgpio.gpio_claim_alert(self.h, p, lgpio.BOTH_EDGES)
        for p in pins:
            self.cbs.append(lgpio.callback(self.h, p, lgpio.BOTH_EDGES,
                                           self._on_edge))
        self.last_state = self.read_state()

    def read_state(self):
        a = lgpio.gpio_read(self.h, self.pins[0])
        b = lgpio.gpio_read(self.h, self.pins[1])
        c = lgpio.gpio_read(self.h, self.pins[2])
        return (a << 2) | (b << 1) | c

    def _on_edge(self, chip, gpio, level, tick):
        s = self.read_state()
        now = time.monotonic()
        with self.lock:
            if s in (0b000, 0b111):
                self.invalid += 1          # never valid on a good 3-hall motor
                return
            if self.last_state is None or self.last_state not in self.idx:
                self.last_state = s
                return
            if s == self.last_state:
                return

            i_old = self.idx[self.last_state]
            i_new = self.idx[s]
            step = (i_new - i_old) % 6
            if step == 1:
                self.count += 1
            elif step == 5:
                self.count -= 1
            else:
                # jumped more than one state — an edge was lost
                self.missed += 1
                self.count += 1 if step <= 3 else -1

            self.transitions += 1
            if self.last_edge_t is not None:
                dt = now - self.last_edge_t
                if dt > 0:
                    r = 1.0 / dt
                    if r > self.max_rate:
                        self.max_rate = r
            self.last_edge_t = now
            self.last_state = s

    def snapshot(self):
        with self.lock:
            return (self.count, self.transitions, self.invalid,
                    self.missed, self.max_rate, self.last_state)

    def zero(self):
        with self.lock:
            self.count = 0
            self.transitions = 0
            self.invalid = 0
            self.missed = 0
            self.max_rate = 0.0

    def cancel(self):
        for cb in self.cbs:
            try:
                cb.cancel()
            except Exception:
                pass


def bits(state):
    if state is None:
        return '---'
    return f'{(state >> 2) & 1}{(state >> 1) & 1}{state & 1}'


def main():
    ap = argparse.ArgumentParser(description='BLDC hall wheel odometry')
    ap.add_argument('--check', action='store_true', help='verify the wiring')
    ap.add_argument('--calibrate', action='store_true',
                    help='roll a measured distance to get metres per count')
    ap.add_argument('--odom', action='store_true', help='live distance readout')
    ap.add_argument('--distance', type=float, default=5.0,
                    help='the measured distance you will roll, in metres')
    ap.add_argument('--m-per-count', type=float, default=None,
                    help='calibration value for --odom')
    ap.add_argument('--pins', type=int, nargs=3, default=[HALL_A, HALL_B, HALL_C],
                    metavar=('A', 'B', 'C'))
    ap.add_argument('--gpiochip', type=int, default=-1)
    a = ap.parse_args()

    if not (a.check or a.calibrate or a.odom):
        ap.print_help()
        return

    chip = a.gpiochip if a.gpiochip >= 0 else find_header_chip()
    h = lgpio.gpiochip_open(chip)
    print(f'[gpiochip{chip}]  halls on GPIO {a.pins[0]}/{a.pins[1]}/{a.pins[2]}')

    hc = HallCounter(h, a.pins)
    stop = threading.Event()

    def on_sig(_s, _f):
        stop.set()
    signal.signal(signal.SIGINT, on_sig)

    try:
        # ---------------- wiring check ----------------
        if a.check:
            print('=' * 62)
            print('  WIRING CHECK — rotate the drive wheel BY HAND')
            print('  All three bits must toggle. Count must rise and fall')
            print('  with direction. Ctrl+C to finish.')
            print('=' * 62)
            seen = set()
            while not stop.is_set():
                cnt, tr, inv, mis, rate, st = hc.snapshot()
                seen.add(st)
                print(f'\r  state {bits(st)}  count {cnt:+7d}  edges {tr:6d}  '
                      f'invalid {inv:4d}  missed {mis:4d}  peak {rate:6.0f} Hz  ',
                      end='', flush=True)
                time.sleep(0.1)
            print()
            print(f'\n  distinct states seen: {len(seen)} '
                  f'({sorted(bits(s) for s in seen if s is not None)})')
            if len(seen) < 6:
                print('  !! Fewer than 6 states — a hall tap or divider is bad,')
                print('     or you did not rotate far enough.')
            if hc.snapshot()[2] > 0:
                print('  !! Invalid states (000/111) seen — check your taps '
                      'and the shared ground.')
            return

        # ---------------- calibration ----------------
        if a.calibrate:
            print('=' * 62)
            print('  CALIBRATION')
            print(f'  You will roll the AGV exactly {a.distance:.2f} m.')
            print('  Mark the start on the floor with tape, measure the end.')
            print('  Push it by hand or drive it — straight line either way.')
            print('=' * 62)
            input('  Position the AGV at the START mark, then press Enter...')
            hc.zero()
            print(f'\n  counting... roll it {a.distance:.2f} m forward, '
                  f'then press Enter')
            done = threading.Event()
            threading.Thread(target=lambda: (input(), done.set()),
                             daemon=True).start()
            while not done.is_set() and not stop.is_set():
                cnt, tr, inv, mis, rate, st = hc.snapshot()
                print(f'\r  count {cnt:+7d}   peak edge rate {rate:6.0f} Hz  ',
                      end='', flush=True)
                time.sleep(0.1)
            cnt, tr, inv, mis, rate, st = hc.snapshot()
            print()
            if abs(cnt) < 10:
                print('\n  Almost no counts — check the wiring with --check first.')
                return
            mpc = a.distance / abs(cnt)
            print()
            print('=' * 62)
            print('  RESULT')
            print('=' * 62)
            print(f'  distance rolled   : {a.distance:.3f} m')
            print(f'  counts            : {cnt:+d}')
            print(f'  METRES PER COUNT  : {mpc:.6f}')
            print(f'  resolution        : {mpc * 1000.0:.2f} mm per count')
            print(f'  peak edge rate    : {rate:.0f} Hz')
            print(f'  missed edges      : {mis}   invalid states: {inv}')
            if mis > abs(cnt) * 0.01:
                print('  !! Over 1% missed edges — Python is dropping counts.')
                print('     Move the counting to a Pico/ESP32 over serial.')
            if rate > 2000:
                print('  !! Peak rate above 2 kHz — too fast for reliable')
                print('     counting in Python. Use a microcontroller.')
            print()
            print('  Use it with:')
            print(f'    python3 hall_odometry.py --odom --m-per-count {mpc:.6f}')
            print('=' * 62)
            return

        # ---------------- live odometry ----------------
        if a.odom:
            if a.m_per_count is None:
                print('  --odom needs --m-per-count (run --calibrate first)')
                return
            print('=' * 62)
            print(f'  LIVE ODOMETRY   {a.m_per_count * 1000:.2f} mm per count')
            print('  Ctrl+C to stop.')
            print('=' * 62)
            hc.zero()
            t0 = time.monotonic()
            last_cnt, last_t = 0, t0
            while not stop.is_set():
                cnt, tr, inv, mis, rate, st = hc.snapshot()
                now = time.monotonic()
                dist = cnt * a.m_per_count
                dt = now - last_t
                speed = ((cnt - last_cnt) * a.m_per_count / dt) if dt > 0.2 else None
                if dt > 0.2:
                    last_cnt, last_t = cnt, now
                sp = f'{speed:+5.2f} m/s' if speed is not None else '  --   '
                print(f'\r  distance {dist:+8.3f} m   {sp}   '
                      f'count {cnt:+7d}   missed {mis:4d}  ', end='', flush=True)
                time.sleep(0.1)
            print()
            return

    finally:
        hc.cancel()
        try:
            lgpio.gpiochip_close(h)
        except Exception:
            pass


if __name__ == '__main__':
    main()
