# EENG 350 SEED Lab - Demo 2: send marker readings to the Arduino by hand
#
# Stands in for the camera so the Arduino can be tested on its own. Uses
# make_message() and send() from demo2.py, so the bytes are exactly what
# demo2.py sends.
#
#   python3 send_marker.py 30 1.5 L     marker 30 deg left, 1.5 m away, arrow left
#   python3 send_marker.py -10 1.2      10 deg right, 1.2 m, arrow not read
#   python3 send_marker.py none         a frame with no marker in it
#   python3 send_marker.py              type readings like the above, q to quit
#
# Angle: degrees from the camera axis, left = positive. Distance: meters.
# Each reading is sent 5 times, because beacon.ino wants a few arrow
# readings before it believes the arrow.
#
# With control/beacon on the Arduino, one reading is enough for a whole run
# by odometry. With control/tests/marker_echo it just prints what it got.

import sys
import time

from demo2 import (make_message, send, open_bus, MARKER_MSG_ID,
                   ARROW_UNKNOWN, ARROW_LEFT, ARROW_RIGHT)

BURST = 5           # times each reading is sent
GAP = 0.02          # seconds between them


def parse(words):
    # ["30", "1.5", "L"] -> (visible, angle, distance, arrow), or None if bad
    if len(words) == 1 and words[0].lower() == "none":
        return False, 0.0, 0.0, ARROW_UNKNOWN
    if len(words) not in (2, 3):
        return None
    arrow = ARROW_UNKNOWN
    if len(words) == 3:
        if words[2].upper() == "L":
            arrow = ARROW_LEFT
        elif words[2].upper() == "R":
            arrow = ARROW_RIGHT
        else:
            return None
    try:
        return True, float(words[0]), float(words[1]), arrow
    except ValueError:
        return None


def send_reading(bus, reading):
    data = make_message(*reading)
    print("  bytes:", MARKER_MSG_ID, *data)
    taken = 0
    for _ in range(BURST):
        taken += send(bus, data)
        time.sleep(GAP)
    if taken < BURST:
        print(f"  Arduino took only {taken} of {BURST}: check wiring and `i2cdetect -y 1`")


def main():
    bus = open_bus()
    if len(sys.argv) > 1:
        reading = parse(sys.argv[1:])
        if reading is None:
            print("Usage: send_marker.py angle_deg distance_m [L|R]   or   none")
            return
        send_reading(bus, reading)
        return
    try:
        while True:
            text = input("angle distance [L|R], none, or q: ").strip()
            if text.lower() == "q":
                break
            reading = parse(text.split())
            if reading is None:
                print("  expected: angle distance [L|R], or none")
                continue
            send_reading(bus, reading)
    except (KeyboardInterrupt, EOFError):
        print()


if __name__ == "__main__":
    main()
