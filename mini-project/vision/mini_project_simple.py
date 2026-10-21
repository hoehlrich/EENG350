# EENG 350 SEED Lab - Mini Project (computer vision / Pi side)
#
# The camera looks for an ArUco marker (DICT_6X6_50). Whichever quadrant of
# the image the marker is in sets the goal position for the two wheels:
#
#     Quadrant   Left wheel   Right wheel
#     NE         0            0
#     NW         0            1
#     SW         1            1
#     SE         1            0
#
# The goal is sent to the Arduino over I2C and shown on the LCD.
#
# Hardware:
#   USB camera  -> any USB port on the Pi
#   LCD shield  -> Pi I2C (SDA/SCL), address 0x20
#   Arduino     -> Pi I2C (SDA/SCL), address 0x08, common ground
#
# Press q in the camera window to quit.

import cv2
from cv2 import aruco
from time import sleep
import threading
import queue

import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus


# ----------------------------------------------------
# SETTINGS
# ----------------------------------------------------

ARDUINO_ADDRESS = 0x08
GOAL_OFFSET = 0

# Goal [left, right] for each quadrant
QUADRANT_GOALS = {
    "NE": [0, 0],
    "NW": [0, 1],
    "SW": [1, 1],
    "SE": [1, 0],
}

# If the marker is this many pixels from a center line, don't change the
# goal (stops the wheels flipping back and forth when it sits on a line)
DEADBAND = 15

# Marker has to be in the new quadrant this many frames in a row before
# the goal changes (ignores one-frame glitches)
STABLE_FRAMES = 3

# Ignore detections smaller than this many pixels^2 (background noise)
MIN_MARKER_AREA = 1500


# ----------------------------------------------------
# LCD SETUP (runs in its own thread)
# ----------------------------------------------------
# The LCD is slow to write. If the main loop wrote to it directly, the
# camera would freeze every time the LCD updated. So the main loop puts
# the new text in a queue and this thread does the slow writing.

lcd_queue = queue.Queue()


def lcd_thread():
    i2c = busio.I2C(board.SCL, board.SDA)
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, 16, 2, address=0x20)
    lcd.clear()
    lcd.message = "Goal Position:\n0 0"
    on_screen = "0 0"

    while True:
        text = lcd_queue.get()      # waits here until there is new text
        if text is None:            # None means the program is ending
            break

        # Only skip ahead to the newest text if several are waiting
        while not lcd_queue.empty():
            text = lcd_queue.get()
            if text is None:
                break
        if text is None:
            break

        # Only rewrite the characters that changed (much faster than
        # clearing and rewriting the whole screen)
        for col in range(len(text)):
            if text[col] != on_screen[col]:
                lcd.cursor_position(col, 1)
                lcd.message = text[col]
        on_screen = text

    lcd.clear()
    lcd.message = "Program ended"


threading.Thread(target=lcd_thread).start()


# ----------------------------------------------------
# I2C SETUP (Arduino)
# ----------------------------------------------------

bus = SMBus(1)


def send_goal(goal):
    try:
        bus.write_i2c_block_data(ARDUINO_ADDRESS, GOAL_OFFSET, goal)
        print("Sent to Arduino:", goal)
    except OSError:
        # Arduino not connected / not answering - keep running anyway
        print("Arduino not responding, goal would be:", goal)


# ----------------------------------------------------
# ARUCO SETUP
# ----------------------------------------------------

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()
parameters.minMarkerPerimeterRate = 0.03


# ----------------------------------------------------
# CAMERA SETUP
# ----------------------------------------------------

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
sleep(0.5)


# ----------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------

goal = [0, 0]           # both wheels start with 0 facing up
send_goal(goal)

new_quadrant = None     # quadrant we are counting frames for
frame_count = 0

while True:

    ret, frame = camera.read()
    if not ret:
        print("Could not read camera.")
        continue

    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(
        gray,
        aruco_dict,
        parameters=parameters
    )

    # ------------------------------------------------
    # FIND THE QUADRANT OF THE BIGGEST MARKER
    # ------------------------------------------------

    quadrant = None
    marker_center = None

    if ids is not None:

        # Pick the biggest marker (closest to the camera)
        biggest_area = 0
        for marker in corners:
            area = cv2.contourArea(marker[0])
            if area > biggest_area:
                biggest_area = area
                biggest = marker[0]

        if biggest_area >= MIN_MARKER_AREA:

            # Center of the marker = average of its 4 corners
            x = int(biggest[:, 0].mean())
            y = int(biggest[:, 1].mean())
            marker_center = (x, y)

            # y counts DOWN from the top of the image, so small y = North
            near_line = (abs(x - center_x) < DEADBAND or
                         abs(y - center_y) < DEADBAND)

            if not near_line:
                if y < center_y:
                    quadrant = "N"
                else:
                    quadrant = "S"
                if x > center_x:
                    quadrant += "E"
                else:
                    quadrant += "W"

            # Draw the marker outline and its center
            cv2.polylines(frame, [biggest.astype(int)], True, (255, 0, 0), 2)
            cv2.circle(frame, marker_center, 6, (0, 0, 255), -1)

    # ------------------------------------------------
    # UPDATE THE GOAL (only after STABLE_FRAMES frames)
    # ------------------------------------------------

    if quadrant is None:
        # No marker, or it's on a line: keep the old goal
        new_quadrant = None
        frame_count = 0
    else:
        if quadrant == new_quadrant:
            frame_count += 1
        else:
            new_quadrant = quadrant
            frame_count = 1

        if frame_count >= STABLE_FRAMES:
            if QUADRANT_GOALS[quadrant] != goal:
                goal = QUADRANT_GOALS[quadrant]
                print("Marker in", quadrant)
                send_goal(goal)
                lcd_queue.put(str(goal[0]) + " " + str(goal[1]))

    # ------------------------------------------------
    # DRAW THE QUADRANT LINES AND GOAL
    # ------------------------------------------------

    cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 255), 2)
    cv2.line(frame, (0, center_y), (width, center_y), (0, 255, 255), 2)

    cv2.putText(frame, "NW", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2)
    cv2.putText(frame, "NE", (center_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2)
    cv2.putText(frame, "SW", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2)
    cv2.putText(frame, "SE", (center_x + 10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    if quadrant is not None:
        marker_text = "Marker: " + quadrant
    elif marker_center is not None:
        marker_text = "Marker: on line"
    else:
        marker_text = "Marker: none"
    goal_text = "Goal Position: " + str(goal[0]) + " " + str(goal[1])

    cv2.putText(frame, marker_text, (10, center_y - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, goal_text, (10, center_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Mini Project", frame)

    # ------------------------------------------------
    # EXIT WITH Q
    # ------------------------------------------------

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break


# ----------------------------------------------------
# CLEANUP
# ----------------------------------------------------

camera.release()
cv2.destroyAllWindows()
lcd_queue.put(None)     # tells the LCD thread to finish
bus.close()
