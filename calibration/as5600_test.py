#!/usr/bin/env python3
"""
AS5600 Magnetic Encoder — Live Angle Test (CALIBRATED)
========================================================
Wiring:
  VCC -> Pi 3.3V (Pin 1)
  GND -> Pi GND  (Pin 6)
  SDA -> Pi GPIO2 (Pin 3)
  SCL -> Pi GPIO3 (Pin 5)
  DIR -> Pi GND  (CW = increasing angle)

Prereq:
  sudo raspi-config  -> enable I2C, reboot
  pip install smbus2 --break-system-packages

CALIBRATION:
  CENTER_RAW below is the raw AS5600 reading measured with the
  steering wheel physically at straight-ahead (dead center).
  Once set, the printed "Angle" is 0.00 deg at that exact
  physical position, positive one direction, negative the other.

  To recalibrate: turn the wheel to true center by hand, run
  this script, read the "raw" value it prints, and put that
  number into CENTER_RAW below.

Keep ferrous metal (steel/iron) away from the sensor -- it
distorts the magnetic field and causes unstable or wrong readings.
"""

import smbus2
import time

I2C_BUS = 1
AS5600_ADDR = 0x36

REG_STATUS = 0x0B       # magnet detection status
REG_ANGLE_H = 0x0E      # filtered/scaled angle, 12-bit (0x0E high, 0x0F low)

# ---------------- CALIBRATION ----------------
CENTER_RAW = 3890   # <-- measured with steering wheel at true center
# ----------------------------------------------

bus = smbus2.SMBus(I2C_BUS)


def read_status():
    status = bus.read_byte_data(AS5600_ADDR, REG_STATUS)
    magnet_detected = bool(status & 0b00100000)
    too_strong = bool(status & 0b00001000)
    too_weak = bool(status & 0b00010000)
    return magnet_detected, too_strong, too_weak


def read_raw():
    high = bus.read_byte_data(AS5600_ADDR, REG_ANGLE_H)
    low = bus.read_byte_data(AS5600_ADDR, REG_ANGLE_H + 1)
    return ((high << 8) | low) & 0x0FFF  # 12-bit value, 0-4095


def centered_angle_deg(raw, center=CENTER_RAW):
    """Angle in degrees relative to `center`, wrapped to [-180, +180).
    Correctly handles the 0/4095 rollover regardless of where
    center sits (including near the edge, like 3890)."""
    diff = (raw - center + 2048) % 4096 - 2048
    return diff * (360.0 / 4096.0)


if __name__ == "__main__":
    print("=" * 55)
    print("AS5600 LIVE ANGLE TEST — CALIBRATED")
    print(f"Center (0 deg) = raw {CENTER_RAW}")
    print("Rotate the magnet in place above the chip. Ctrl+C to stop.")
    print("=" * 55)

    try:
        while True:
            detected, too_strong, too_weak = read_status()
            if not detected:
                print("NO MAGNET DETECTED — check magnet position/distance (0.5-3mm gap)")
            elif too_strong:
                print("Magnet TOO STRONG / TOO CLOSE — move it further away")
            elif too_weak:
                print("Magnet TOO WEAK / TOO FAR — move it closer")
            else:
                raw = read_raw()
                deg = centered_angle_deg(raw)
                print(f"Angle: {deg:+7.2f} deg   (raw: {raw:4d}/4095, center: {CENTER_RAW})")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.close()
