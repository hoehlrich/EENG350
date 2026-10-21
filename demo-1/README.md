# EENG350 Demo 1

The robot must be able to:

1. Detect ArUco marker 0 (6×6 dictionary) 1 to 5 ft away and show on the LCD
   the signed angle from the camera axis to the marker. The angle is positive
   when the marker is to the left.
2. Drive straight a given distance (1 to 10 ft) and stop.
3. Turn by a given angle, then drive a given distance.

For tasks 2 and 3, the angle and distance are set in the code before upload.
After upload, nothing else can be sent to the robot. It must start within 10 s
and finish within 60 s. The score is the error at the robot's center of
rotation (target 0.5 in, and 0.75° for task 1).

## Organization

- `control/`: Arduino sketch for tasks 2 and 3, identification and calibration tools, and Simulink/MATLAB models (see [`control/README.md`](control/README.md))
- `vision/`: Pi code for task 1 (to be added)
- `resources/`: Demo 1 instructions and the SteeringControl handout

## Quick start for a demo run (tasks 2 and 3)

1. In `control/drive/drive.ino`, set `TARGET_ANGLE_DEG` (positive = left) and `TARGET_DISTANCE_FT`. The demo gives the angle in radians: degrees = radians × 180/π.
2. Put the robot on the start mark and upload. It waits `START_DELAY_S` (9 s), turns, drives, and stops.
3. To run the same targets again without re-uploading, press the Arduino reset button.
