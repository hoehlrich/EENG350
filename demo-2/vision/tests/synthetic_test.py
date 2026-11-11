#!/usr/bin/env python3
"""EENG350 Demo 2 - Check demo2.py's marker and arrow detection without the camera.

Draws camera frames of the two beacons from the handout (beacon_left.png and
beacon_right.png in this folder) as a made-up camera would see them, from a
range of distances and angles, with the beacon turned partly away, on a
cluttered background. Each frame goes through find_marker() and
find_arrow() from demo2.py, and the answers are compared with where the
beacon was drawn.

    python3 synthetic_test.py           run every case, print a table
    python3 synthetic_test.py --save    also save each frame to frames/ to look at

This checks the geometry, the signs, and that the arrow areas are in the
right place. It can't check the real camera: its focus, blur, and how it
sees the printed colors under the room lights. The camera here is
1280x720 with a focal length of 1000 pixels and no lens distortion. It is
not the robot's camera, so camera_calibration.npz is not needed.

Needs opencv-contrib-python and numpy. Runs on any computer.
"""

import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))       # demo2.py is one folder up
import demo2                                    # noqa: E402
from marker_link import ARROW_UNKNOWN, ARROW_LEFT, ARROW_RIGHT, ARROW_NAMES   # noqa: E402

# The made-up camera
WIDTH, HEIGHT = demo2.FRAME_WIDTH, demo2.FRAME_HEIGHT
FOCAL = 1000.0
CAMERA_MATRIX = np.array([[FOCAL, 0, WIDTH / 2], [0, FOCAL, HEIGHT / 2], [0, 0, 1]])
NO_DISTORTION = np.zeros(5)

# The beacon pictures: (file, arrow, marker center x in the picture). Both
# are 660 pixels tall and so is the marker, so 660 pixels = one marker width.
MARKER_PIXELS = 660
BEACONS = [("beacon_left.png", ARROW_LEFT, 979), ("beacon_right.png", ARROW_RIGHT, 331)]
MARGIN = 150            # white paper around the picture [picture pixels]

# What is tried. Distances are camera to marker; the camera is 0.2 m from
# the marker when the robot stops 1 ft away, and 1.83 m is 6 ft.
DISTANCES_M = [0.2, 0.3, 0.6, 0.9, 1.2, 1.5, 1.83]
ANGLES_DEG = [-20, 0, 20]       # marker left (+) or right (-) of the camera axis
YAWS_DEG = [-30, 0, 30]         # beacon turned about a vertical axis
MARKER_BELOW_M = 0.03           # marker center this far below the camera axis

ANGLE_TOLERANCE_DEG = 0.5
DISTANCE_TOLERANCE = 0.05       # fraction


def background():
    """A frame with no beacon: gray wall, brownish floor, two colored
    objects that are not arrows, and noise."""
    rng = np.random.default_rng(0)
    frame = np.full((HEIGHT, WIDTH, 3), (150, 150, 150), np.float32)
    frame[HEIGHT * 2 // 3:] = (70, 110, 160)                # floor (BGR): orange-brown
    cv2.rectangle(frame, (30, 30), (130, 110), (40, 40, 220), -1)       # red thing
    cv2.rectangle(frame, (1150, 30), (1250, 110), (60, 170, 40), -1)    # green thing
    return frame + rng.normal(0, 4, frame.shape)


def draw_frame(picture, marker_x, distance_m, angle_deg, yaw_deg):
    """Return the frame the camera would see with the beacon picture placed
    so its marker center is distance_m away (along the floor), angle_deg to
    the left of the camera axis, and turned yaw_deg."""
    pixel = demo2.MARKER_SIZE_M / MARKER_PIXELS             # meters per picture pixel
    paper = cv2.copyMakeBorder(picture, MARGIN, MARGIN, MARGIN, MARGIN,
                               cv2.BORDER_CONSTANT, value=(255, 255, 255))
    height, width = paper.shape[:2]
    center_u, center_v = marker_x + MARGIN, MARKER_PIXELS / 2 + MARGIN

    # Camera coordinates: X right, Y down, Z out of the lens
    angle, yaw = math.radians(angle_deg), math.radians(yaw_deg)
    center = np.array([-distance_m * math.sin(angle), MARKER_BELOW_M,
                       distance_m * math.cos(angle)])
    across = np.array([math.cos(yaw), 0, math.sin(yaw)])    # along the paper, to its right
    down = np.array([0, 1, 0])

    # Where the paper's four corners land in the frame
    paper_corners = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    frame_corners = []
    for u, v in paper_corners:
        point = center + (u - center_u) * pixel * across + (v - center_v) * pixel * down
        frame_corners.append((CAMERA_MATRIX @ point / point[2])[:2])
    frame_corners = np.float32(frame_corners)

    # Shrink the paper to about its size in the frame first, so the warp
    # doesn't skip pixels, then warp it into place
    scale = min(1.0, 2 * FOCAL * pixel / distance_m)
    small = cv2.resize(paper, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    warp = cv2.getPerspectiveTransform(np.float32(paper_corners * scale), frame_corners)
    frame = background()
    drawn = cv2.warpPerspective(small.astype(np.float32), warp, (WIDTH, HEIGHT))
    covered = cv2.warpPerspective(np.ones(small.shape[:2], np.float32), warp, (WIDTH, HEIGHT))
    frame = frame * (1 - covered[:, :, None]) + drawn * covered[:, :, None]

    # Dimmer than a computer drawing, and a little out of focus
    frame = cv2.GaussianBlur(frame * 0.8, (0, 0), 0.8)
    return np.clip(frame, 0, 255).astype(np.uint8)


def main():
    save = "--save" in sys.argv
    if save:
        os.makedirs(os.path.join(HERE, "frames"), exist_ok=True)

    print("beacon  dist(m)  angle  yaw | angle err  dist err  green   red  arrow | result")
    failures = 0
    cases = 0
    for name, true_arrow, marker_x in BEACONS:
        picture = cv2.imread(os.path.join(HERE, name))
        for distance in DISTANCES_M:
            for angle in ANGLES_DEG:
                for yaw in YAWS_DEG:
                    frame = draw_frame(picture, marker_x, distance, angle, yaw)
                    if save:
                        cv2.imwrite(os.path.join(HERE, "frames", "%s_%.2f_%+d_%+d.png" % (
                            ARROW_NAMES[true_arrow], distance, angle, yaw)), frame)
                    cases += 1
                    label = "%-6s %7.2f  %+5d  %+3d |" % (
                        ARROW_NAMES[true_arrow], distance, angle, yaw)

                    result = demo2.find_marker(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                                               CAMERA_MATRIX, NO_DISTORTION)
                    if result is None:
                        failures += 1
                        print(label, "marker not found".center(43), "| FAIL")
                        continue
                    found_angle, found_distance, corners = result
                    arrow, green, red, left_area, right_area = demo2.find_arrow(frame, corners)

                    # Close up and off to one side, the arrow can be outside
                    # the picture. Then there is nothing to read.
                    area = left_area if true_arrow == ARROW_LEFT else right_area
                    in_frame = (area.min() >= 0 and area[:, 0].max() < WIDTH
                                and area[:, 1].max() < HEIGHT)

                    problems = []
                    if abs(found_angle - angle) > ANGLE_TOLERANCE_DEG:
                        problems.append("angle")
                    if abs(found_distance / distance - 1) > DISTANCE_TOLERANCE:
                        problems.append("distance")
                    if in_frame and arrow != true_arrow:
                        problems.append("arrow")
                    if not in_frame and arrow not in (true_arrow, ARROW_UNKNOWN):
                        problems.append("arrow")
                    failures += bool(problems)
                    verdict = "ok" if in_frame else "ok (arrow out of the picture)"
                    print(label, "%+8.2f  %+7.1f%%  %5.2f %5.2f  %-5s | %s" % (
                        found_angle - angle, 100 * (found_distance / distance - 1),
                        green, red, ARROW_NAMES[arrow],
                        "FAIL: " + ", ".join(problems) if problems else verdict))

    print("%d of %d cases passed" % (cases - failures, cases))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
