# EENG 350 SEED Lab - Demo 2 (computer vision / Pi side)
#
# Every camera frame: look for ArUco marker 0 (DICT_6X6_50, 2 in wide) and
# the arrow beside it, then send the Arduino one 8 byte message over I2C.
# The Arduino (control/beacon/beacon.ino) does all the driving. It never
# replies.
#
# The message (must match struct MarkerMsg in beacon.ino), little-endian:
#
#     byte 0     message ID    0x01
#     byte 1     visible       1 if marker 0 is in this frame, else 0
#     byte 2-3   angle_cdeg    int16, camera axis to marker, 0.01 deg, + = left
#     byte 4-5   distance_mm   uint16, camera to marker along the floor, mm
#     byte 6     arrow         0 unknown, 1 left, 2 right
#     byte 7     checksum      low byte of the sum of bytes 1 to 6
#
# When the marker isn't visible, bytes 2 to 6 are 0. The Arduino drops any
# message with the wrong length, ID or checksum.
#
# Arrow: the beacon has a green arrow on its LEFT (points left) or a red
# arrow on its RIGHT (points right). So we check the patch of image just
# left of the marker for green and just right of it for red.
#
# Hardware:
#   USB camera  -> any USB port on the Pi
#   LCD shield  -> Pi I2C, address 0x20 (optional)
#   Arduino     -> Pi I2C (GPIO2 SDA -> A4, GPIO3 SCL -> A5, GND -> GND), 0x08
#
# Needs camera_calibration.npz (from Demo 1) in this folder.
# For a demo run use ../trial1.sh or ../trial2.sh, which also upload the
# Arduino sketch. To run only this: python3 demo2.py
# Quit: q in the camera window, or Ctrl+C
#
# send_marker.py and tests/synthetic_test.py import the functions below,
# so this file is the only copy of the message format and the detection.

import cv2
from cv2 import aruco
import numpy as np
import math
import os
import struct
import time
import threading
import queue
import subprocess

# ----------------------------------------------------
# SETTINGS
# ----------------------------------------------------
MARKER_ID = 0
MARKER_SIZE = 0.0508                # marker black square width [m] (2 in)
FRAME_WIDTH = 1280                  # same resolution as the calibration
FRAME_HEIGHT = 720

ARDUINO_ADDRESS = 0x08
MARKER_MSG_ID = 0x01
ARROW_UNKNOWN = 0
ARROW_LEFT = 1
ARROW_RIGHT = 2

# Where the arrow sits, in marker widths from the marker center
ARROW_NEAR = 0.65                   # inner edge of the patch we check
ARROW_FAR = 1.55                    # outer edge
ARROW_HALF_HEIGHT = 0.30            # patch goes this far above and below center

# Colors in OpenCV HSV (hue 0-179). Tune these in the demo room.
GREEN_LOW = (35, 80, 60)
GREEN_HIGH = (90, 255, 255)
RED_LOW_1 = (0, 80, 60)             # red wraps around hue 0, so two ranges
RED_HIGH_1 = (10, 255, 255)
RED_LOW_2 = (170, 80, 60)
RED_HIGH_2 = (179, 255, 255)

# TEMPORARY, for the practice beacons printed at home: the printer made the
# left (green) arrow blue and the right (red) arrow pink. True looks for blue
# on the left and pink on the right instead. SET BACK TO False for the real
# beacons.
HOME_PRINTER_COLORS = True
if HOME_PRINTER_COLORS:
    GREEN_LOW = (85, 60, 60)        # "green" = left arrow: blue/cyan, hue 85-130
    GREEN_HIGH = (130, 255, 255)
    RED_LOW_1 = (0, 40, 60)         # "red" = right arrow: pink/magenta, hue 135-179
    RED_HIGH_1 = (5, 255, 255)      # and 0-5 (light pink wraps around hue 0).
    RED_LOW_2 = (135, 40, 60)       # Lower saturation limit (40), as pink is pale
    RED_HIGH_2 = (179, 255, 255)

MIN_FILL = 0.12                     # fraction of the patch that must be the color
MAX_FILL = 0.70                     # more than this is a wall/shirt, not an arrow

SHOW_WINDOW = "DISPLAY" in os.environ   # no window when run over SSH or at boot

# CONTINUOUS SEARCH (continuous-search branch): a short fixed exposure, so the
# marker stays sharp while the robot turns. In the camera's units, 100 us:
# 25 = 2.5 ms. It must match SEARCH_PHI_DOT in beacon.ino (see there). Too
# short and the picture is too dark to find the marker at 6 ft: watch the
# brightness printed every 2 s (the average pixel, 0-255) and the camera
# window. The camera is put back on automatic exposure when this exits.
# None = automatic exposure, as on main.
EXPOSURE_100US = 25


def set_exposure(value):
    # value: exposure in 100 us units, or None for automatic exposure
    if value is None:
        args = ["-c", "auto_exposure=3"]
    else:
        args = ["-c", "auto_exposure=1", "-c", "exposure_time_absolute=%d" % value]
    try:
        subprocess.run(["v4l2-ctl", "-d", "/dev/video0"] + args, check=True)
    except (OSError, subprocess.CalledProcessError) as e:
        print("Could not set the exposure:", e)

# ----------------------------------------------------
# SETUP
# ----------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())

# Marker corners in 3D (marker's own frame), same order ArUco gives them:
# top-left, top-right, bottom-right, bottom-left
h = MARKER_SIZE / 2
marker_points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
                         dtype=np.float64)


def open_bus():
    # I2C to the Arduino. If there is no bus (testing on a laptop) keep going.
    try:
        from smbus2 import SMBus
        return SMBus(1)
    except Exception as e:
        print("No I2C bus, messages will not be sent:", e)
        return None

# ----------------------------------------------------
# LCD (optional, runs in its own thread so it never slows the camera)
# ----------------------------------------------------
lcd_queue = queue.Queue(maxsize=1)

def lcd_thread():
    try:
        import board
        import busio
        import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
        i2c = busio.I2C(board.SCL, board.SDA)
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2)
        lcd.color = [0, 0, 100]
        lcd.clear()
    except Exception as e:
        print("No LCD:", e)
        return
    while True:
        text = lcd_queue.get()
        try:
            lcd.clear()
            lcd.message = text
        except OSError:
            pass                    # I2C hiccup, try again next time


# ----------------------------------------------------
# FUNCTIONS
# ----------------------------------------------------
def find_marker(gray, camera_matrix, dist_coeffs):
    # Returns (angle_deg, distance_m, corners) for marker 0, or None
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    ids = ids.flatten()
    for i in range(len(ids)):
        if ids[i] == MARKER_ID:
            c = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(marker_points, c, camera_matrix,
                                          dist_coeffs,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
            x, y, z = tvec.flatten()        # x right, y down, z out of the lens
            angle = math.degrees(math.atan2(-x, z))   # marker on the left = +
            distance = math.sqrt(x * x + z * z)       # along the floor (no y)
            return angle, distance, c
    return None


def color_fraction(hsv, x1, x2, y1, y2, color):
    # Fraction of the box [x1:x2, y1:y2] that is green or red
    x1 = max(0, int(x1))
    x2 = min(hsv.shape[1], int(x2))
    y1 = max(0, int(y1))
    y2 = min(hsv.shape[0], int(y2))
    if x2 <= x1 or y2 <= y1:            # box is off the edge of the picture
        return 0.0
    patch = hsv[y1:y2, x1:x2]
    if color == "green":
        mask = cv2.inRange(patch, GREEN_LOW, GREEN_HIGH)
    else:
        mask = cv2.inRange(patch, RED_LOW_1, RED_HIGH_1) | \
               cv2.inRange(patch, RED_LOW_2, RED_HIGH_2)
    return cv2.countNonZero(mask) / mask.size


def find_arrow(frame, c):
    # c = marker corners (TL, TR, BR, BL). Returns (arrow, boxes, green, red)
    cx, cy = c.mean(axis=0)
    width = (np.linalg.norm(c[1] - c[0]) + np.linalg.norm(c[2] - c[3])) / 2
    height = (np.linalg.norm(c[3] - c[0]) + np.linalg.norm(c[2] - c[1])) / 2
    y1 = cy - ARROW_HALF_HEIGHT * height
    y2 = cy + ARROW_HALF_HEIGHT * height
    left_box = (cx - ARROW_FAR * width, cx - ARROW_NEAR * width, y1, y2)
    right_box = (cx + ARROW_NEAR * width, cx + ARROW_FAR * width, y1, y2)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green = color_fraction(hsv, *left_box, "green")
    red = color_fraction(hsv, *right_box, "red")
    is_left = MIN_FILL <= green <= MAX_FILL
    is_right = MIN_FILL <= red <= MAX_FILL

    if is_left and not is_right:
        arrow = ARROW_LEFT
    elif is_right and not is_left:
        arrow = ARROW_RIGHT
    else:
        arrow = ARROW_UNKNOWN           # neither or both: don't guess
    return arrow, left_box, right_box, green, red


def make_message(visible, angle_deg, distance_m, arrow):
    # Bytes 1 to 7 of the message (byte 0, the ID, is sent as the I2C command byte)
    if not visible:
        angle_deg, distance_m, arrow = 0, 0, ARROW_UNKNOWN
    angle_cdeg = max(-32768, min(32767, round(angle_deg * 100)))
    distance_mm = max(0, min(65535, round(distance_m * 1000)))
    payload = struct.pack("<BhHB", 1 if visible else 0, angle_cdeg,
                          distance_mm, arrow)
    return list(payload) + [sum(payload) & 0xFF]


def send(bus, data):
    # ID byte + 7 data bytes = the 8 bytes beacon.ino expects
    if bus is None:
        return False
    try:
        bus.write_i2c_block_data(ARDUINO_ADDRESS, MARKER_MSG_ID, data)
        return True
    except OSError:                     # Arduino resetting or unplugged
        return False

# ----------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------
def main():
    calibration = np.load(os.path.join(HERE, "camera_calibration.npz"))
    camera_matrix = calibration["camera_matrix"]
    dist_coeffs = calibration["dist_coeffs"]
    bus = open_bus()
    threading.Thread(target=lcd_thread, daemon=True).start()

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    camera.set(cv2.CAP_PROP_FPS, 30)
    set_exposure(EXPOSURE_100US)
    print("exposure:", "automatic" if EXPOSURE_100US is None else "%.1f ms fixed" % (EXPOSURE_100US / 10))
    settle_until = time.time() + 0.5
    while time.time() < settle_until:   # let auto exposure settle
        camera.read()

    arrow_names = {ARROW_UNKNOWN: "?", ARROW_LEFT: "L", ARROW_RIGHT: "R"}
    frames = 0
    sent = 0
    brightness = 0.0                    # sum of the average pixel of each frame
    last_print = time.time()
    last_lcd = 0

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("Camera read failed")
                time.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            result = find_marker(gray, camera_matrix, dist_coeffs)

            if result is None:
                visible, angle, distance, arrow = False, 0.0, 0.0, ARROW_UNKNOWN
            else:
                angle, distance, corners = result
                arrow, left_box, right_box, green, red = find_arrow(frame, corners)
                visible = True

            if send(bus, make_message(visible, angle, distance, arrow)):
                sent += 1
            frames += 1
            brightness += gray.mean()

            # Terminal, every 2 s
            now = time.time()
            if now - last_print >= 2:
                fps = frames / (now - last_print)
                light = brightness / max(frames, 1)
                if visible:
                    print(f"{fps:4.1f} fps  brightness {light:3.0f}  sent {sent}/{frames}  "
                          f"angle {angle:6.2f} deg  dist {distance:5.3f} m  "
                          f"arrow {arrow_names[arrow]}")
                else:
                    print(f"{fps:4.1f} fps  brightness {light:3.0f}  sent {sent}/{frames}  no marker")
                frames = 0
                sent = 0
                brightness = 0.0
                last_print = now

            # LCD, twice a second
            if now - last_lcd >= 0.5 and lcd_queue.empty():
                if visible:
                    text = f"A {angle:6.2f} {arrow_names[arrow]}\nD {distance:5.3f} m"
                else:
                    text = "No marker"
                lcd_queue.put(text)
                last_lcd = now

            # Camera window, for tuning
            if SHOW_WINDOW:
                if visible:
                    aruco.drawDetectedMarkers(frame, [corners.astype(np.float32).reshape(1, 4, 2)])
                    for box, frac, color in ((left_box, green, (0, 255, 0)),
                                             (right_box, red, (0, 0, 255))):
                        x1, x2, y1, y2 = [int(v) for v in box]
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, f"{frac:.2f}", (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    cv2.putText(frame, f"{angle:.2f} deg  {distance:.3f} m  "
                                f"arrow {arrow_names[arrow]}", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                cv2.imshow("Demo 2", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass

    send(bus, make_message(False, 0, 0, ARROW_UNKNOWN))   # leave the Arduino on "no marker"
    camera.release()
    if EXPOSURE_100US is not None:
        set_exposure(None)              # leave the camera on automatic exposure
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
