#!/usr/bin/env python3
"""
============================================================
AS5600 MULTI-TURN UNWRAP + GEAR RATIO CALIBRATION
============================================================
Your AS5600 sits on the wiper motor shaft, which turns several
times across the wheel's +/-45 deg. A single-turn sensor cannot
express that — the reading wraps at 360 and the angle becomes
meaningless.

This tool fixes that in software by UNWRAPPING: it samples fast
enough that the shaft never moves more than 180 deg between
samples, detects each wrap, and accumulates a continuous angle
that keeps counting past 360.

  shaft speed  ~100 deg/s   (your measured 3.6 s / 360 deg)
  sample rate  100 Hz       -> ~1 deg per sample
  wrap limit   180 deg      -> 180x margin. Very safe.

WHAT THIS SCRIPT GIVES YOU
  The GEAR RATIO: encoder degrees per degree of wheel angle.
  Feed that number into the motor bridge and the wheel angle
  becomes real again — limits, closed loop and go-to-angle all
  start working.

HOW TO RUN IT
  Stop the motor bridge and the console first (they hold the
  I2C/GPIO).  Then:

      python3 as5600_calibrate.py

  1. Turn the front wheel BY HAND to full LEFT lock, press Enter
  2. Turn it BY HAND to full RIGHT lock, press Enter
  3. Centre it, press Enter
  It prints the ratio and the settings to paste into the bridge.

  Turning by hand avoids stalling the wiper motor against the
  end stops during calibration.

DEPENDENCIES
  pip install smbus2 --break-system-packages
============================================================
"""

import math
import sys
import threading
import time

import smbus2

I2C_BUS = 1
AS5600_ADDR = 0x36
REG_ANGLE_H = 0x0E
REG_STATUS = 0x0B

SAMPLE_HZ = 100.0
WRAP_WARN_DEG = 150.0     # a single step this big means sampling is too slow


class Unwrapper:
    """Turns the single-turn AS5600 into a continuous multi-turn angle."""

    def __init__(self, bus):
        self.bus = bus
        self.lock = threading.Lock()
        self.accum_deg = 0.0        # continuous, unbounded
        self.last_deg = None
        self.max_step = 0.0
        self.wrap_warnings = 0
        self.samples = 0
        self._stop = threading.Event()

    def read_raw(self):
        hi = self.bus.read_byte_data(AS5600_ADDR, REG_ANGLE_H)
        lo = self.bus.read_byte_data(AS5600_ADDR, REG_ANGLE_H + 1)
        return ((hi << 8) | lo) & 0x0FFF

    def magnet_ok(self):
        s = self.bus.read_byte_data(AS5600_ADDR, REG_STATUS)
        return bool(s & 0b00100000), bool(s & 0b00001000), bool(s & 0b00010000)

    def _tick(self):
        deg = self.read_raw() * (360.0 / 4096.0)
        with self.lock:
            if self.last_deg is not None:
                d = deg - self.last_deg
                # unwrap: the shortest way round is the real motion
                if d > 180.0:
                    d -= 360.0
                elif d < -180.0:
                    d += 360.0
                if abs(d) > self.max_step:
                    self.max_step = abs(d)
                if abs(d) > WRAP_WARN_DEG:
                    self.wrap_warnings += 1
                self.accum_deg += d
            self.last_deg = deg
            self.samples += 1

    def run(self):
        period = 1.0 / SAMPLE_HZ
        nxt = time.monotonic()
        while not self._stop.is_set():
            try:
                self._tick()
            except Exception as e:
                print(f"\n[I2C ERROR] {e}")
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def start(self):
        self.t = threading.Thread(target=self.run, daemon=True)
        self.t.start()

    def stop(self):
        self._stop.set()

    def value(self):
        with self.lock:
            return self.accum_deg


def live_prompt(u, msg):
    """Show the live accumulated angle until the user presses Enter."""
    done = threading.Event()

    def waiter():
        input()
        done.set()

    threading.Thread(target=waiter, daemon=True).start()
    print(f"\n{msg}")
    print("   (turn it BY HAND — press Enter when there)")
    while not done.is_set():
        print(f"\r   encoder: {u.value():+10.1f} deg   "
              f"max step {u.max_step:5.2f} deg   ", end="", flush=True)
        time.sleep(0.1)
    v = u.value()
    print(f"\r   captured: {v:+10.1f} deg" + " " * 30)
    return v


def main():
    bus = smbus2.SMBus(I2C_BUS)
    u = Unwrapper(bus)

    det, strong, weak = u.magnet_ok()
    print("=" * 62)
    print("  AS5600 MULTI-TURN UNWRAP CALIBRATION")
    print(f"  magnet detected: {det}   too strong: {strong}   too weak: {weak}")
    if not det:
        print("  !! NO MAGNET DETECTED — fix that before calibrating.")
        return
    print(f"  sampling at {SAMPLE_HZ:.0f} Hz")
    print("=" * 62)

    u.start()
    time.sleep(0.5)

    print()
    print("  Your steering has NO mechanical end stops — it rotates freely.")
    print("  So we measure the ratio by counting FULL TURNS of the wheel")
    print("  instead of going lock to lock.")
    print()
    print("  Put a tape mark on the steered wheel and a matching mark on")
    print("  the chassis so you can see exactly one revolution.")
    print()

    try:
        turns = float(input("  How many full turns will you rotate it? [2]: ") or "2")
    except ValueError:
        turns = 2.0
    if turns <= 0:
        print("  Needs to be positive.")
        return

    input("  Line up the marks, then press Enter to zero the counter...")
    with u.lock:
        u.accum_deg = 0.0
        u.max_step = 0.0
        u.wrap_warnings = 0

    after = live_prompt(
        u, f"  Now rotate the wheel exactly {turns:.0f} full turn(s) "
           f"in ONE direction.")

    u.stop()
    time.sleep(0.2)

    enc = abs(after)
    wheel_deg = 360.0 * turns
    ratio = enc / wheel_deg if wheel_deg else 0.0

    print()
    print("=" * 62)
    print("  RESULTS")
    print("=" * 62)
    print(f"  wheel rotated      : {wheel_deg:8.1f} deg ({turns:.0f} turns)")
    print(f"  encoder accumulated: {enc:8.1f} deg ({enc / 360.0:.2f} turns)")
    print(f"  RATIO              : {ratio:8.3f} encoder deg per wheel deg")
    print(f"  largest step       : {u.max_step:8.2f} deg (must stay under 180)")
    if u.wrap_warnings:
        print(f"  !! {u.wrap_warnings} oversized steps — sampling too slow, "
              f"the count may be wrong")
    print()

    if ratio < 0.05:
        print("  Encoder barely moved. Check the magnet and the mounting.")
        return

    print("=" * 62)
    if 0.85 <= ratio <= 1.15:
        print("  VERDICT: the encoder is 1:1 WITH THE STEERING PIVOT")
        print("=" * 62)
        print("  Good news — nothing is broken and you need no new hardware.")
        print("  Your steering axis rotates continuously, and a single-turn")
        print("  absolute encoder is exactly the right sensor for that.")
        print("  The 'wrapping' you saw was the wheel genuinely passing 360.")
        print()
        print("  Run the bridge in SINGLE-TURN mode (no unwrapping):")
        print("      -p as5600_unwrap:=false \\")
        print("      -p steer_gear_ratio:=1.0 \\")
        print("      -p as5600_center_raw:=<raw with the wheel straight>")
        print()
        print("  Get that raw value from as5600_test.py with the wheel")
        print("  pointing straight ahead.")
    else:
        print(f"  VERDICT: GEARED — {ratio:.2f} encoder turns per wheel turn")
        print("=" * 62)
        print("  The encoder is upstream of the steering gearing, so a single")
        print("  reading is ambiguous. Run the bridge with UNWRAPPING on:")
        print()
        print("      -p as5600_unwrap:=true \\")
        print(f"      -p steer_gear_ratio:={ratio:.3f} \\")
        print(f"      -p as5600_sample_hz:=100")
        print()
        print("  Home it with the wheel straight before each run:")
        print('      ros2 topic pub --once /agv/steer_home '
              'std_msgs/msg/String "{data: here}"')
    print()
    print("  RESOLUTION")
    print(f"    one AS5600 count = {(360.0 / 4096.0) / max(ratio, 1e-6):.4f} "
          f"wheel deg")
    print()
    print("  !! NO END STOPS — SOFTWARE LIMITS ARE THE ONLY LIMITS !!")
    print("     Nothing mechanical stops the steering, so set")
    print("     steer_max_deg to the range you actually want (45 is")
    print("     sensible) and keep the cmd_vel watchdog short. If the")
    print("     count is ever lost, re-home before driving.")
    print("=" * 62)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(0)
