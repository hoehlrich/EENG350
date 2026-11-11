# EENG 350 SEED Lab - Demo 1, Task 1: marker angle (Pi / computer vision)
#
# Finds ArUco marker 0 (DICT_6X6_50, 5 cm) and shows on the LCD the signed
# angle between the camera axis and the marker. Left of the axis = positive.
#
#     angle = atan2(-X, Z)
#
# where (X, Z) is the marker centre in camera coordinates (X right, Z out of
# the lens), found with solvePnP from the 4 marker corners and the camera
# calibration in camera_calibration.npz (made by calibrate_camera.py).
#
# Run:  python3 demo1.py      Quit: q in the camera window, or Ctrl+C
# Works with no screen too (e.g. started over SSH): then only the LCD shows.

import math
import os
import queue
import signal
import statistics
import threading
import time

import cv2
from cv2 import aruco
import numpy as np


# ----------------------------------------------------
# SETTINGS
# ----------------------------------------------------

MARKER_ID = 0
MARKER_SIZE_M = 0.05            # 5 cm black square (sets distance, not angle)
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720   # same as the calibration
SMOOTHING_FRAMES = 5            # LCD shows the median of the last 5 angles
HOLD_TIME_S = 1.0               # keep the last angle up through short dropouts

# Camera calibration: K (focal length + axis pixel) and lens distortion
HERE = os.path.dirname(os.path.abspath(__file__))
calibration = np.load(os.path.join(HERE, "camera_calibration.npz"))
camera_matrix = calibration["camera_matrix"]
dist_coeffs = calibration["dist_coeffs"]

# ArUco detector with sub-pixel corners (needed for sub-degree accuracy)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
detector = aruco.ArucoDetector(
    aruco.getPredefinedDictionary(aruco.DICT_6X6_50), parameters)

# Marker corners in the marker's own frame (metres, centre at 0,0), in
# ArUco's order: top-left, top-right, bottom-right, bottom-left
half = MARKER_SIZE_M / 2
MARKER_POINTS = np.array([[-half,  half, 0], [half,  half, 0],
                          [half, -half, 0], [-half, -half, 0]])


def find_marker_angle(gray):
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
            # minus sign: marker on the camera's left gives a positive angle
            return (math.degrees(math.atan2(-x, z)),
                    math.sqrt(x * x + y * y + z * z), marker_corners)
    return None


# ----------------------------------------------------
# LCD thread (the LCD is slow, so the camera loop never waits on it)
# ----------------------------------------------------

lcd_queue = queue.Queue()
GREEN = [0, 100, 0]
RED = [100, 0, 0]


def lcd_thread():
    import board
    import busio
    import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
    # At power-up the I2C bus can take a moment to appear, so keep trying
    while True:
        try:
            lcd = character_lcd.Character_LCD_RGB_I2C(
                busio.I2C(board.SCL, board.SDA), 16, 2, address=0x20)
            lcd.clear()
            break
        except (OSError, ValueError) as error:
            print("Waiting for LCD:", error)
            time.sleep(1)
    while True:
        item = lcd_queue.get()          # wait for new text from main()
        while not lcd_queue.empty() and item is not None:
            item = lcd_queue.get()      # skip to the newest text
        if item is None:                # None = program is ending
            break
        text, color = item
        lcd.color = color
        lcd.cursor_position(0, 0)
        lcd.message = text
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
    threading.Thread(target=lcd_thread).start()
    show_on_lcd("Starting...", "", RED)
    camera = open_camera()

    axis_x = int(round(camera_matrix[0, 2]))    # camera axis column
    # Only open the camera window if there is a screen (over SSH there
    # isn't, and the LCD alone is enough for the demo)
    show_window = "DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ
    recent_angles = []      # last few angles, for the median
    last_seen = None        # time marker 0 was last seen
    shown = None            # LCD text on screen now (only send changes)

    try:
        while True:
            ret, frame = camera.read()
            if not ret:                 # skip a bad frame
                continue

            # ---- find the marker and its angle ----

            result = find_marker_angle(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            now = time.monotonic()
            if result is not None:
                angle, distance, corners = result
                last_seen = now
                recent_angles = (recent_angles + [angle])[-SMOOTHING_FRAMES:]
                # Outline the marker in blue on the camera view
                cv2.polylines(frame, [corners.astype(int)], True,
                              (255, 0, 0), 2)

            # ---- choose the LCD text ----
            # Seen in the last second: show the median angle (steady digits)
            if last_seen is not None and now - last_seen < HOLD_TIME_S:
                angle_text = format(statistics.median(recent_angles), "+6.2f")
                lcd_text = ("Marker 0 found", "Angle:" + angle_text + " deg",
                            GREEN)
            else:                       # gone for over a second
                recent_angles = []
                lcd_text = ("No marker", "Angle: ---", RED)

            # Only write to the LCD when the text changes (it's slow)
            if lcd_text != shown:
                show_on_lcd(*lcd_text)
                shown = lcd_text
                print(lcd_text[0], "|", lcd_text[1])

            # ---- camera window (only when a screen is attached) ----
            if show_window:
                # Yellow line = camera axis (0 degrees)
                cv2.line(frame, (axis_x, 0), (axis_x, frame.shape[0]),
                         (0, 255, 255), 3)
                cv2.putText(frame, lcd_text[0] + "  " + lcd_text[1],
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
                cv2.imshow("Demo 1", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass

    # ---- clean up ----
    camera.release()
    if show_window:
        cv2.destroyAllWindows()
    lcd_queue.put(None)     # tells the LCD thread to clear and stop


if __name__ == "__main__":
    main()
