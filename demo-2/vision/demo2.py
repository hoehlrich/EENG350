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

# ----------------------------------------------------
# SETTINGS
# ----------------------------------------------------
MARKER_ID = 0
MARKER_SIZE = 0.0508                # marker black square width [m] (2 in)
FRAME_WIDTH = 1280                  # same resolution as the calibration
FRAME_HEIGHT = 720
CAMERA_FPS = 30                     # frame rate to ask the camera for

# Tracking: once the marker has been seen, the next frame is searched only
# near where it was (much faster than the whole picture). If it isn't there,
# the whole picture is searched in the same frame, so nothing is missed.
# The window reaches this many marker widths past the marker on each side,
# and at least TRACK_MIN_PX pixels (the marker moves about 55 px per frame
# when the robot turns at 1.5 rad/s and the camera gives 30 frames/s).
TRACK_MARGIN = 1.5
TRACK_MIN_PX = 150

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
MIN_FILL = 0.12                     # fraction of the patch that must be the color
MAX_FILL = 0.70                     # more than this is a wall/shirt, not an arrow

SHOW_WINDOW = "DISPLAY" in os.environ   # no window when run over SSH or at boot

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
def track_window(corners, gray):
    # Part of the picture to search next frame: the marker's box plus a margin.
    # Returns (x1, y1, x2, y2).
    width = np.ptp(corners[:, 0])
    margin = max(TRACK_MARGIN * width, TRACK_MIN_PX)
    x1 = max(0, int(corners[:, 0].min() - margin))
    y1 = max(0, int(corners[:, 1].min() - margin))
    x2 = min(gray.shape[1], int(corners[:, 0].max() + margin))
    y2 = min(gray.shape[0], int(corners[:, 1].max() + margin))
    return x1, y1, x2, y2


def find_marker(gray, camera_matrix, dist_coeffs, window=None):
    # Returns (angle_deg, distance_m, corners) for marker 0, or None.
    # window = (x1, y1, x2, y2): look there first, then in the whole picture.
    if window is not None:
        x1, y1, x2, y2 = window
        result = find_marker_in(gray[y1:y2, x1:x2], (x1, y1),
                                camera_matrix, dist_coeffs)
        if result is not None:
            return result
    return find_marker_in(gray, (0, 0), camera_matrix, dist_coeffs)


def find_marker_in(gray, offset, camera_matrix, dist_coeffs):
    # Looks for marker 0 in gray, which starts at pixel offset = (x, y) of
    # the full picture. Corners come back in full-picture pixels.
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    ids = ids.flatten()
    for i in range(len(ids)):
        if ids[i] == MARKER_ID:
            c = corners[i][0].astype(np.float64) + offset
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


def color_fraction(frame, x1, x2, y1, y2, color):
    # Fraction of the box [x1:x2, y1:y2] that is green or red
    x1 = max(0, int(x1))
    x2 = min(frame.shape[1], int(x2))
    y1 = max(0, int(y1))
    y2 = min(frame.shape[0], int(y2))
    if x2 <= x1 or y2 <= y1:            # box is off the edge of the picture
        return 0.0
    # Only the box is converted to HSV, not the whole picture (faster)
    patch = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
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

    green = color_fraction(frame, *left_box, "green")
    red = color_fraction(frame, *right_box, "red")
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
# CAMERA THREAD
# ----------------------------------------------------
class CameraReader:
    # Reads the camera in its own thread, so the next frame is captured and
    # decoded while the main loop works on this one. Only the newest frame
    # is kept: the main loop never works on an old picture.

    def __init__(self, camera):
        self.camera = camera
        self.frame = None
        self.number = 0                 # counts frames read so far
        self.new_frame = threading.Condition()
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while self.running:
            ok, frame = self.camera.read()
            if not self.running:
                break
            if not ok:
                print("Camera read failed")
                time.sleep(0.1)
                continue
            with self.new_frame:
                self.frame = frame
                self.number += 1
                self.new_frame.notify()

    def read(self, last_number):
        # Waits for a frame newer than last_number. Returns (number, frame),
        # or (last_number, None) if none came within 1 s.
        with self.new_frame:
            if not self.new_frame.wait_for(lambda: self.number != last_number,
                                           timeout=1.0):
                return last_number, None
            return self.number, self.frame

    def stop(self):
        # Let the thread finish its read before the camera is released
        self.running = False
        self.thread.join(timeout=1.0)


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
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # don't queue up old frames
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    # Automatic exposure. The camera keeps its settings between programs, so
    # this undoes any fixed exposure another program left behind.
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)          # 3 = automatic (V4L2)
    print(f"Camera: {camera.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
          f"{camera.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} at "
          f"{camera.get(cv2.CAP_PROP_FPS):.0f} fps")
    settle_until = time.time() + 0.5
    while time.time() < settle_until:   # let auto exposure settle
        camera.read()
    reader = CameraReader(camera)

    arrow_names = {ARROW_UNKNOWN: "?", ARROW_LEFT: "L", ARROW_RIGHT: "R"}
    frame_number = 0
    window = None                       # where to look first (tracking)
    frames = 0
    sent = 0
    busy = 0.0                          # time spent working on frames [s]
    last_print = time.time()
    last_lcd = 0

    try:
        while True:
            frame_number, frame = reader.read(frame_number)
            if frame is None:
                continue
            start = time.time()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            result = find_marker(gray, camera_matrix, dist_coeffs, window)

            if result is None:
                visible, angle, distance, arrow = False, 0.0, 0.0, ARROW_UNKNOWN
                window = None
            else:
                angle, distance, corners = result
                arrow, left_box, right_box, green, red = find_arrow(frame, corners)
                visible = True
                window = track_window(corners, gray)

            if send(bus, make_message(visible, angle, distance, arrow)):
                sent += 1
            frames += 1
            busy += time.time() - start

            # Terminal, every 2 s. "work" is the time per frame spent finding
            # the marker. If it is well under 1000/fps, the camera is what
            # limits the frame rate, not this code.
            now = time.time()
            if now - last_print >= 2:
                fps = frames / (now - last_print)
                work = 1000 * busy / max(frames, 1)
                if visible:
                    print(f"{fps:4.1f} fps  work {work:4.1f} ms  sent {sent}/{frames}  "
                          f"angle {angle:6.2f} deg  dist {distance:5.3f} m  "
                          f"arrow {arrow_names[arrow]}")
                else:
                    print(f"{fps:4.1f} fps  work {work:4.1f} ms  sent {sent}/{frames}  "
                          f"no marker")
                frames = 0
                sent = 0
                busy = 0.0
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
    reader.stop()
    camera.release()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
