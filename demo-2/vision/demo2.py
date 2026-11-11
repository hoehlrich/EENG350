# EENG 350 SEED Lab - Demo 2: find the beacon and its arrow (Pi / computer vision)
#
# For every camera frame this looks for ArUco marker 0 (DICT_6X6_50, 2 in)
# and sends the Arduino one message (marker_link.py) saying
#
#     visible     is marker 0 in this frame
#     angle       camera axis to the marker, degrees, left = positive
#     distance    camera to the marker, measured along the floor, meters
#     arrow       which way the arrow beside the marker points
#
# The Arduino does everything else (control/beacon/beacon.ino). Nothing
# comes back from it.
#
# Angle and distance are Demo 1's method: solvePnP on the 4 marker corners
# with the camera calibration gives the marker center (X, Y, Z) in camera
# coordinates (X right, Y down, Z out of the lens), then
#
#     angle = atan2(-X, Z)        distance = sqrt(X^2 + Z^2)
#
# The arrow is found by color. The beacons in the handout have a green
# arrow on the left of the marker pointing left, or a red arrow on the
# right pointing right. So the picture just left of the marker is checked
# for green and the picture just right of it for red.
#
# Run:  python3 demo2.py      Quit: q in the camera window, or Ctrl+C
# With a screen attached it shows the camera view with the marker and the
# two arrow areas outlined, for tuning the settings below. Without one
# (SSH, or started at boot by demo2.service) it only sends.
#
# Needs camera_calibration.npz in this directory (same file as Demo 1).

import math
import os
import queue
import signal
import threading
import time

import cv2
from cv2 import aruco
import numpy as np

from marker_link import (ArduinoLink, ARROW_UNKNOWN, ARROW_LEFT, ARROW_RIGHT,
                         ARROW_NAMES)


# ----------------------------------------------------
# SETTINGS
# ----------------------------------------------------

MARKER_ID = 0
MARKER_SIZE_M = 0.0508          # 2 in black square (sets the distance)
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720   # same as the calibration

# Exposure time, in the camera driver's units (usually 0.1 ms). None = auto.
# A short fixed exposure keeps the marker sharp while the robot is turning,
# which lets the Arduino search faster (SEARCH_PHI_DOT). Too short and the
# picture is too dark to detect anything.
EXPOSURE = None

# Where the arrow is, measured in marker widths from the marker center.
# In the handout's pictures (tests/beacon_left.png, beacon_right.png) the
# arrow starts 0.72 widths out, ends 1.47 widths out, and is 0.52 tall.
ARROW_NEAR = 0.65               # inner edge of the area that is checked
ARROW_FAR = 1.55                # outer edge
ARROW_HALF_HEIGHT = 0.30        # the area reaches this far above and below center
PATCH_PIXELS = 32               # each area is resampled to this many pixels per marker width

# Colors, as OpenCV HSV: hue 0-179, saturation and value 0-255.
# Tune these under the demo room's lights (see the camera window).
GREEN_HUE = (35, 90)            # hue from, to
RED_HUE_BELOW, RED_HUE_ABOVE = 10, 170   # red wraps around the ends of the hue scale
MIN_SATURATION = 80             # less than this is white, gray or black
MIN_VALUE = 60                  # less than this is too dark to tell
# How much of an area must be the color to count as an arrow. The arrow
# fills about 0.35 of its area. Much more than that is not an arrow but
# something big and colored behind the beacon (a red wall, a green shirt).
ARROW_MIN_FILL = 0.12
ARROW_MAX_FILL = 0.70

USE_LCD = True                  # show status on the I2C LCD if there is one
LCD_PERIOD_S = 0.5              # the LCD is slow, so update it this often at most
PRINT_PERIOD_S = 2.0            # print the frame rate and newest reading this often

HERE = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(HERE, "camera_calibration.npz")

# ArUco detector with sub-pixel corners (needed for sub-degree accuracy)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
detector = aruco.ArucoDetector(
    aruco.getPredefinedDictionary(aruco.DICT_6X6_50), parameters)

# Marker corners in the marker's own frame (meters, center at 0,0), in
# ArUco's order: top-left, top-right, bottom-right, bottom-left
half = MARKER_SIZE_M / 2
MARKER_POINTS = np.array([[-half,  half, 0], [half,  half, 0],
                          [half, -half, 0], [-half, -half, 0]])


# ----------------------------------------------------
# MARKER
# ----------------------------------------------------

def find_marker(gray, camera_matrix, dist_coeffs):
    """Return (angle_deg, distance_m, corners) for marker 0, or None."""
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:                     # no markers at all in this frame
        return None
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id == MARKER_ID:      # ignore any other marker IDs
            marker_corners = corners[i][0].astype(np.float64)
            # 3D position of the marker from its 4 corner pixels + calibration
            # (lens distortion is removed here)
            ok, rvec, tvec = cv2.solvePnP(MARKER_POINTS, marker_corners,
                                          camera_matrix, dist_coeffs,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
            x, y, z = tvec.flatten()    # X right, Y down, Z out of the lens
            # minus sign: marker on the camera's left gives a positive angle.
            # Height (Y) is left out of the distance: the robot moves along
            # the floor, and the camera and marker need not be level.
            return (math.degrees(math.atan2(-x, z)), math.hypot(x, z),
                    marker_corners)
    return None


# ----------------------------------------------------
# ARROW
# ----------------------------------------------------

def side_area(corners, side):
    """Corners, in the image, of the area beside the marker where an arrow
    would be. side = -1 for the marker's left, +1 for its right.

    The marker's own corners give its center and two vectors: one marker
    width to its right, and one marker height down. Any point near the
    marker is center + a*across + b*down, so the area follows the marker
    when it is far away, off to one side, or tilted. Returned as
    [inner top, outer top, outer bottom, inner bottom].
    """
    top_left, top_right, bottom_right, bottom_left = corners
    center = corners.mean(axis=0)
    across = ((top_right + bottom_right) - (top_left + bottom_left)) / 2
    down = ((bottom_left + bottom_right) - (top_left + top_right)) / 2
    return np.array([
        center + side * ARROW_NEAR * across - ARROW_HALF_HEIGHT * down,
        center + side * ARROW_FAR * across - ARROW_HALF_HEIGHT * down,
        center + side * ARROW_FAR * across + ARROW_HALF_HEIGHT * down,
        center + side * ARROW_NEAR * across + ARROW_HALF_HEIGHT * down])


def color_fill(frame, area, color):
    """Return the fraction (0 to 1) of an area that is "green" or "red"."""
    # Cut the area out of the frame as a small upright rectangle
    width = int((ARROW_FAR - ARROW_NEAR) * PATCH_PIXELS)
    height = int(2 * ARROW_HALF_HEIGHT * PATCH_PIXELS)
    upright = np.float32([[0, 0], [width, 0], [width, height]])
    transform = cv2.getAffineTransform(np.float32(area[:3]), upright)
    patch = cv2.warpAffine(frame, transform, (width, height))

    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    colorful = (saturation >= MIN_SATURATION) & (value >= MIN_VALUE)
    if color == "green":
        right_hue = (hue >= GREEN_HUE[0]) & (hue <= GREEN_HUE[1])
    else:
        right_hue = (hue <= RED_HUE_BELOW) | (hue >= RED_HUE_ABOVE)
    return float(np.mean(colorful & right_hue))


def find_arrow(frame, corners):
    """Return (arrow, green_fill, red_fill, left_area, right_area).

    arrow is ARROW_LEFT if there is a green arrow on the marker's left,
    ARROW_RIGHT if there is a red one on its right, and ARROW_UNKNOWN if
    there is neither or both. The Arduino adds up many frames, so "unknown"
    costs nothing and is better than a guess. The rest is for the camera
    window.
    """
    left_area = side_area(corners, -1)
    right_area = side_area(corners, +1)
    green = color_fill(frame, left_area, "green")
    red = color_fill(frame, right_area, "red")
    green_arrow = ARROW_MIN_FILL <= green <= ARROW_MAX_FILL
    red_arrow = ARROW_MIN_FILL <= red <= ARROW_MAX_FILL
    if green_arrow and not red_arrow:
        arrow = ARROW_LEFT
    elif red_arrow and not green_arrow:
        arrow = ARROW_RIGHT
    else:
        arrow = ARROW_UNKNOWN
    return arrow, green, red, left_area, right_area


# ----------------------------------------------------
# LCD thread (the LCD is slow, so the camera loop never waits on it)
# ----------------------------------------------------

lcd_queue = queue.Queue()
GREEN = [0, 100, 0]
RED = [100, 0, 0]


def lcd_thread():
    # The LCD is optional in this demo. At power-up the I2C bus can take a
    # moment to appear, so try for a while, then carry on without it.
    lcd = None
    for attempt in range(10):
        try:
            import board
            import busio
            import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
            lcd = character_lcd.Character_LCD_RGB_I2C(
                busio.I2C(board.SCL, board.SDA), 16, 2, address=0x20)
            lcd.clear()
            break
        except ImportError as error:    # LCD library not installed: waiting won't help
            print("No LCD library:", error)
            lcd = None
            break
        except Exception as error:      # no bus yet, or no LCD
            print("Waiting for LCD:", error)
            time.sleep(1)
    if lcd is None:
        print("No LCD, carrying on without it")
    while True:
        item = lcd_queue.get()          # wait for new text from main()
        while not lcd_queue.empty() and item is not None:
            item = lcd_queue.get()      # skip to the newest text
        if item is None:                # None = program is ending
            break
        if lcd is None:
            continue
        text, color = item
        try:
            lcd.color = color
            lcd.cursor_position(0, 0)
            lcd.message = text
        except OSError:                 # a failed write: the next one tries again
            pass
    if lcd is not None:
        lcd.clear()
        lcd.color = [0, 0, 0]


def show_on_lcd(line1, line2, color):
    # Pad each line to 16 chars so old text gets overwritten
    lcd_queue.put((line1.ljust(16) + "\n" + line2.ljust(16), color))


# ----------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------

def open_camera():
    """Open the USB camera, retrying until it gives a frame (at power-up
    it can take a few seconds to be ready)."""
    while True:
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # Keep only the newest frame, so the Arduino never gets an old one
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if EXPOSURE is not None:
            camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual on V4L2 cameras
            camera.set(cv2.CAP_PROP_EXPOSURE, EXPOSURE)
        if camera.read()[0]:
            return camera
        print("Waiting for camera...")
        camera.release()
        time.sleep(1)


def stop(signum, frame):
    raise KeyboardInterrupt     # turn "systemctl stop" into a clean exit


def main():
    # Ctrl+C / systemctl stop -> leave the loop cleanly
    signal.signal(signal.SIGTERM, stop)

    # Camera calibration: K (focal length + axis pixel) and lens distortion
    calibration = np.load(CALIBRATION_FILE)
    camera_matrix = calibration["camera_matrix"]
    dist_coeffs = calibration["dist_coeffs"]

    if USE_LCD:
        threading.Thread(target=lcd_thread).start()
        show_on_lcd("Starting...", "", RED)
    link = ArduinoLink()
    camera = open_camera()

    axis_x = int(round(camera_matrix[0, 2]))    # camera axis column
    # Only open the camera window if there is a screen
    show_window = "DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ
    last_lcd = 0.0          # when the LCD was last updated
    last_print = time.monotonic()
    frames = 0              # frames since the last print
    sent = 0                # messages the Arduino took since the last print

    try:
        while True:
            ret, frame = camera.read()
            if not ret:                 # skip a bad frame
                continue

            # ---- find the marker and its arrow, tell the Arduino ----

            result = find_marker(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                                 camera_matrix, dist_coeffs)
            if result is None:
                sent += link.send(False)
                status = ("No marker", "", RED)
            else:
                angle, distance, corners = result
                arrow, green, red, left_area, right_area = find_arrow(frame, corners)
                sent += link.send(True, angle, distance, arrow)
                status = ("Angle:" + format(angle, "+6.1f") + " deg",
                          format(distance, ".2f") + " m  " + ARROW_NAMES[arrow],
                          GREEN)
            frames += 1
            now = time.monotonic()

            # ---- status: LCD, terminal ----

            if USE_LCD and now - last_lcd >= LCD_PERIOD_S:
                show_on_lcd(*status)
                last_lcd = now
            if now - last_print >= PRINT_PERIOD_S:
                print(format(frames / (now - last_print), ".0f"), "frames/s,",
                      sent, "of", frames, "sent |", status[0], status[1])
                last_print = now
                frames = 0
                sent = 0

            # ---- camera window (only when a screen is attached) ----

            if show_window:
                # Yellow line = camera axis (0 degrees)
                cv2.line(frame, (axis_x, 0), (axis_x, frame.shape[0]),
                         (0, 255, 255), 2)
                if result is not None:
                    # Marker in blue, the two arrow areas in their colors,
                    # each labeled with how much of it is that color
                    cv2.polylines(frame, [corners.astype(int)], True,
                                  (255, 0, 0), 2)
                    for area, fill, color in ((left_area, green, (0, 200, 0)),
                                              (right_area, red, (0, 0, 255))):
                        cv2.polylines(frame, [area.astype(int)], True, color, 2)
                        x, y = area.min(axis=0).astype(int)
                        cv2.putText(frame, format(fill, ".2f"), (x, y - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.putText(frame, status[0] + "  " + status[1],
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
                cv2.imshow("Demo 2", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass

    # ---- clean up ----
    camera.release()
    if show_window:
        cv2.destroyAllWindows()
    link.close()
    lcd_queue.put(None)     # tells the LCD thread to clear and stop


if __name__ == "__main__":
    main()
