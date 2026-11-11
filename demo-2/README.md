# EENG350 Demo 2

The robot starts 4 to 6 ft from a beacon, facing a random direction. The
beacon is ArUco marker 0 (6×6 dictionary, 2 in wide) with an arrow beside it:
a green arrow on its left pointing left, or a red arrow on its right pointing
right. There are two kinds of trial:

1. Turn to find the beacon, drive to within 1.5 ft of it, and stop without
   touching it.
2. The same, then turn 90° the way the arrow points.

Distances are measured from the robot's center of rotation. The robot must
start moving within 30 s of the go signal and nothing may be plugged into it
or sent to it during a trial. Trials are scored on time and on the number of
failures. The full rules are in `resources/Demo2Instructions.pdf`.

## How it works

The Pi looks and the Arduino drives. For every camera frame the Pi sends the
Arduino one message saying whether it can see the marker, the angle and
distance to it, and which way the arrow points. Nothing is sent back. The
Arduino turns each reading into a heading and a distance to aim for,
measured by its own wheel encoders, and runs Demo 1's controller toward
them:

| Stage | What the robot does | Ends when |
|-------|---------------------|-----------|
| wait | Nothing, for `START_DELAY_S` after reset | Time is up |
| search | Turns in place | A message says the marker is visible |
| face | Turns toward the marker | The marker is within `FACE_TOL_DEG` of straight ahead |
| approach | Drives at the marker, steering as new readings arrive | It is `STOP_DISTANCE_FT` from the marker |
| turn | Trial type 2 only: turns 90° the way the arrow points | The turn has settled |
| done | Motors off | |

## Organization

- `vision/`: Pi code. Finds the marker and the arrow and sends the messages (see [`vision/README.md`](vision/README.md))
- `control/`: Arduino sketch, test tools, a PC simulation that runs the sketch, and the Simulink model (see [`control/README.md`](control/README.md))
- `resources/`: Demo 2 instructions

## The message

The Pi is the I2C master and the Arduino is a slave at address `0x08`, wired
as in the mini project:

| Pi | Arduino |
|----|---------|
| GPIO2 SDA (header pin 3) | A4 (SDA) |
| GPIO3 SCL (header pin 5) | A5 (SCL) |
| GND (header pin 6) | GND |

One 8 byte message per camera frame, multi-byte values little-endian:

| Byte | Field | Meaning |
|------|-------|---------|
| 0 | message ID | `0x01` |
| 1 | `visible` | 1 if marker 0 is in this frame, else 0 |
| 2–3 | `angle_cdeg` | int16, camera axis to marker in 0.01°, positive = marker to the left |
| 4–5 | `distance_mm` | uint16, camera to marker along the floor, mm |
| 6 | `arrow` | 0 unknown, 1 left, 2 right |
| 7 | checksum | low byte of the sum of bytes 1 to 6 |

When the marker is not visible, bytes 2 to 6 are 0. The Arduino drops any
message with the wrong length, ID or checksum, and it only ever acts on a
new message, so if the Pi stops sending, the robot finishes on what it
already knows or keeps searching.

The format is written down in three places, which must change together:

- `vision/marker_link.py`: `MESSAGE_FORMAT` and `pack_message()`
- `control/beacon/beacon.ino`: `struct MarkerMsg`
- `control/tests/marker_echo/marker_echo.ino`: the same struct

For a different kind of message, give it a new ID in byte 0. A sketch that
doesn't know the ID ignores it.

## Quick start for a demo run

Once, on the Pi: put `camera_calibration.npz` (the Demo 1 file, it is not in
this repository) in `vision/`, and install `vision/demo2.service` so
`demo2.py` starts at boot (instructions at the top of that file).

For each trial:

1. Power the Pi and give it time to boot. If the LCD is attached it shows
   `No marker`, or the angle once the camera can see the beacon.
2. In `control/beacon/beacon.ino`, set `TURN_AT_BEACON` to `false` for trial
   type 1 or `true` for type 2, and upload.
3. Unplug the USB cable and put the robot on its start mark.
4. Press the Arduino's reset button. That is the go signal. The robot
   starts moving `START_DELAY_S` (1 s) later.

To run the same trial type again, just press reset.

## Before the first run on the robot

Everything here has been checked in simulation only (`control/README.md`,
"What has been tested"). These need the robot, in this order:

1. Check the link and the readings with `control/tests/marker_echo`.
2. Measure `CAMERA_OFFSET_M` and choose `STOP_DISTANCE_FT` so the front of
   the robot stays clear of the beacon.
3. Check that the camera finds the marker at 6 ft and reads both arrows
   under the room's lights (`vision/README.md`, "Tuning").
4. Check that the marker is found while the robot turns at `SEARCH_PHI_DOT`.
5. Raise the speeds. They are Demo 1's slow, proven ones, and time is scored.

The steps are spelled out in `control/README.md`, "Bringing it up on the
robot".
