#!/usr/bin/env python3
"""
compass_calibrate.py

Helper script to calibrate the QMC5883L magnetometer used by nmea_server.py.

It collects raw X/Y magnetometer samples while you slowly rotate the board/boat,
then prints suggested COMPASS_X_OFFSET / COMPASS_Y_OFFSET / SCALE values to
paste into nmea_server.py.
"""

import time
import py_qmc5883l

SAMPLES_DURATION = 60.0  # seconds to collect data


def main():
    sensor = py_qmc5883l.QMC5883L()
    print("Calibrating QMC5883L (2D)...")
    print("For the next %d seconds, slowly rotate the board/boat in all headings." % SAMPLES_DURATION)
    print("Try to draw big circles / figure-8s so the sensor sees all directions.")
    time.sleep(2)

    xs, ys = [], []
    end_t = time.time() + SAMPLES_DURATION
    count = 0

    while time.time() < end_t:
        try:
            m = sensor.get_magnet()
            # get_magnet() returns [x, y] on this driver
            if not isinstance(m, (list, tuple)) or len(m) < 2:
                raise ValueError(f"unexpected magnet data: {m!r}")
            mx, my = float(m[0]), float(m[1])
            xs.append(mx)
            ys.append(my)
            count += 1
            if count % 50 == 0:
                print(f"  collected {count} samples...")
        except Exception as e:
            print("read error:", e)
        time.sleep(0.05)

    if not xs:
        print("No samples collected, something is wrong.")
        return

    print(f"\nCollected {len(xs)} samples.")

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    x_off = (x_max + x_min) / 2.0
    y_off = (y_max + y_min) / 2.0

    x_rad = (x_max - x_min) / 2.0
    y_rad = (y_max - y_min) / 2.0
    avg_rad = (x_rad + y_rad) / 2.0 if (x_rad and y_rad) else 1.0

    x_scale = avg_rad / x_rad if x_rad else 1.0
    y_scale = avg_rad / y_rad if y_rad else 1.0

    print("\nRaw ranges:")
    print(f"  X: {x_min} .. {x_max}")
    print(f"  Y: {y_min} .. {y_max}")

    print("\nSuggested nmea_server.py settings:")
    print("  COMPASS_USE_CALIB = True")
    print(f"  COMPASS_X_OFFSET = {x_off:.3f}")
    print(f"  COMPASS_Y_OFFSET = {y_off:.3f}")
    print(f"  COMPASS_X_SCALE  = {x_scale:.6f}")
    print(f"  COMPASS_Y_SCALE  = {y_scale:.6f}")

    print("\nAfter editing nmea_server.py with these values, restart nmea-server.service.")


if __name__ == "__main__":
    main()
