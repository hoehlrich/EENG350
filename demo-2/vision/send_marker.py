#!/usr/bin/env python3
"""EENG350 Demo 2 - Send marker readings to the Arduino by hand, Pi side.

Stands in for demo2.py so the Arduino can be tested without the camera.
With control/beacon/beacon.ino on the Arduino, one reading is enough to
make the robot do a whole run by odometry: turn to face where the marker
was said to be, drive to STOP_DISTANCE_FT short of it, and turn the way
the arrow was said to point. With control/tests/marker_echo/marker_echo.ino
the Arduino just prints what it received.

    python3 send_marker.py 30 1.5 L     marker 30 deg left, 1.5 m away, arrow left
    python3 send_marker.py -10 1.2      10 deg right, 1.2 m, arrow not read
    python3 send_marker.py none         a frame with no marker in it
    python3 send_marker.py              type readings like the above, q to quit
    python3 send_marker.py -r 30 0 1 R  the same reading 30 times a second
                                        until Ctrl+C (a link test: a robot
                                        running beacon.ino will not get
                                        anywhere sensible with a marker
                                        that stays at the same angle)

Angle: degrees from the camera axis, left = positive. Distance: meters
from the camera. Arrow: L, R, or left out. Each reading is sent BURST
times, because beacon.ino wants ARROW_MIN_VOTES readings of the arrow
before it believes it.

The message format is in marker_link.py.

Hardware
    Raspberry Pi GPIO2 (SDA, pin 3) -> Arduino A4 (SDA)
    Raspberry Pi GPIO3 (SCL, pin 5) -> Arduino A5 (SCL)
    Raspberry Pi GND (pin 6)        -> Arduino GND
    I2C must be enabled on the Pi (raspi-config -> Interface Options).
    `i2cdetect -y 1` should show 08. Needs the smbus2 package.
"""

import argparse
import time

from marker_link import (ArduinoLink, pack_message, ARROW_UNKNOWN, ARROW_LEFT,
                         ARROW_RIGHT, ARROW_NAMES, MARKER_MSG_ID)

BURST = 5               # times each typed reading is sent
BURST_GAP_S = 0.02      # time between them

ARROWS = {"L": ARROW_LEFT, "R": ARROW_RIGHT}


def parse_reading(words):
    """Turn ["30", "1.5", "L"] into (visible, angle_deg, distance_m, arrow).
    Raises ValueError if it isn't a reading."""
    if len(words) == 1 and words[0].lower() == "none":
        return False, 0.0, 0.0, ARROW_UNKNOWN
    if len(words) not in (2, 3):
        raise ValueError("expected: angle distance [L|R], or none")
    arrow = ARROW_UNKNOWN
    if len(words) == 3:
        if words[2].upper() not in ARROWS:
            raise ValueError("arrow must be L or R")
        arrow = ARROWS[words[2].upper()]
    return True, float(words[0]), float(words[1]), arrow


def describe(reading):
    visible, angle, distance, arrow = reading
    data = [MARKER_MSG_ID] + list(pack_message(*reading))
    what = "no marker"
    if visible:
        what = f"marker at {angle:+.2f} deg, {distance:.3f} m, arrow {ARROW_NAMES[arrow]}"
    return f"{what}  (bytes {' '.join(str(b) for b in data)})"


def send_burst(link, reading):
    """Send one reading BURST times. Returns how many the Arduino took."""
    taken = 0
    for _ in range(BURST):
        taken += link.send(*reading)
        time.sleep(BURST_GAP_S)
    return taken


def report(taken):
    if taken < BURST:
        print(f"  the Arduino took only {taken} of {BURST}: check the wiring and `i2cdetect -y 1`")


def main():
    parser = argparse.ArgumentParser(
        description="Send marker readings to the Arduino by hand.",
        epilog="With no reading, type them one per line.")
    parser.add_argument("-r", "--rate", type=float, metavar="HZ",
                        help="repeat the reading this many times a second until Ctrl+C")
    parser.add_argument("reading", nargs="*",
                        help="angle_deg distance_m [L|R], or none")
    args = parser.parse_args()

    link = ArduinoLink()
    try:
        if args.reading:
            try:
                reading = parse_reading(args.reading)
            except ValueError as error:
                parser.error(str(error))
            print(describe(reading))
            if args.rate:
                while True:
                    link.send(*reading)
                    time.sleep(1 / args.rate)
            else:
                report(send_burst(link, reading))
        else:
            while True:
                text = input("angle distance [L|R], none, or q: ").strip()
                if text.lower() == "q":
                    break
                try:
                    reading = parse_reading(text.split())
                except ValueError as error:
                    print(" ", error)
                    continue
                print(" ", describe(reading))
                report(send_burst(link, reading))
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        link.close()


if __name__ == "__main__":
    main()
