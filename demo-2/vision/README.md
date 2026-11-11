# Vision (Demo 2)

`demo2.py` runs on the Raspberry Pi. For every camera frame it looks for
ArUco marker 0 (`DICT_6X6_50`, 2 in) and the arrow beside it, and sends the
Arduino one 8 byte I2C message: is the marker visible, the angle and
distance to it, and which way the arrow points. The format is in
[`../README.md`](../README.md#the-message) and must match `struct MarkerMsg`
in `control/beacon/beacon.ino`.

For a demo run, don't start it by hand: use `../trial1.sh` or
`../trial2.sh`, which also upload the Arduino sketch.

## Files

| Path | What it is |
|------|------------|
| `demo2.py` | **The whole vision side.** Camera → marker and arrow → message to the Arduino. Settings at the top |
| `send_marker.py` | Sends readings you type instead of camera readings, to test the Arduino on its own. Uses `demo2.py`'s message code |
| `tests/synthetic_test.py` | Runs `demo2.py`'s detection on drawn camera frames. Needs no camera or Pi |
| `tests/beacon_left.png`, `tests/beacon_right.png` | The two beacons from the handout. Used by the test, and good for printing practice beacons (print the marker 2 in wide) |

## How it works

- **Marker.** `solvePnP` on the 4 marker corners with the camera calibration
  gives the marker center `(x, y, z)` in camera coordinates. The angle is
  `atan2(-x, z)` (positive when the marker is on the left) and the distance
  is `sqrt(x² + z²)`, along the floor, so camera height doesn't matter.
- **Arrow.** The beacon has a green arrow left of the marker (points left)
  or a red arrow right of it (points right). Using the marker's width in the
  picture, the script checks a box just left of the marker for green and a
  box just right of it for red. Between `MIN_FILL` and `MAX_FILL` of a box
  counts as an arrow. Neither or both is sent as unknown.
- **Speed.** The camera is read in its own thread, so the next frame is
  captured and decoded while this one is processed, and only the newest
  frame is ever used. Once the marker has been seen, the next frame is
  searched only near where it was (`TRACK_MARGIN`, `TRACK_MIN_PX`), and the
  whole picture only if it isn't there. Only the two arrow boxes are
  converted to HSV. On a PC this makes a frame 1.5 times cheaper while
  searching and 2.5 times cheaper once the marker is found, with the same
  answers as before. Every 2 s the script prints the frame rate and `work`,
  the time spent per frame. If `work` is well under 1000 / fps, the camera
  is what limits the frame rate, not the code. On the robot it is 15
  frames/s with about 19 ms of work, so the camera is the limit. The camera
  is always set to automatic exposure.
- **No smoothing.** Every frame is sent as measured. The Arduino uses the
  newest angle and distance and counts arrow readings as votes.
- **LCD** (optional) shows angle, distance and arrow twice a second from its
  own thread. If it isn't attached the script carries on without it.

## Requirements

- Python 3 with `opencv-contrib-python`, `numpy` and `smbus2`
- `adafruit-circuitpython-charlcd` for the LCD (optional)
- `camera_calibration.npz` in this folder (the Demo 1 file, not in the repo),
  at the same 1280x720 resolution
- I2C enabled on the Pi. `i2cdetect -y 1` should show `08`

## Tuning

In the demo room, run `python3 demo2.py` with a screen attached and hold
each beacon in front of the camera at 1 to 6 ft. The window shows the marker
and the two arrow boxes, each labeled with how much of it is green or red.

| What you see | What to change |
|--------------|----------------|
| Marker not outlined at 6 ft | Focus and lighting first. The marker is only about 30 pixels wide there |
| A box isn't over the arrow | `ARROW_NEAR`, `ARROW_FAR`, `ARROW_HALF_HEIGHT` |
| The number over the arrow is below 0.2 | Widen `GREEN_LOW`/`GREEN_HIGH` or the red ranges, or lower their saturation (middle number) |
| The number on the empty side is above 0.1 | Something behind the beacon is that color: narrow the ranges or raise saturation |
| Marker missed during the search | The camera only detects it when the robot is still: lengthen `SEARCH_PAUSE_S` or shrink `SEARCH_STEP_DEG` in `beacon.ino` |

## Testing without the robot

```
python3 tests/synthetic_test.py
```

Draws each beacon as a camera would see it from 0.2 to 1.83 m, 20° to
either side, turned up to 30°, on a background with other red and green
things in it, and checks the angle (within 0.5°), distance (within 5%) and
arrow. All 126 cases pass. It says nothing about the real camera's focus,
blur or the room's lights, which is what the tuning is for.

To test the Arduino without the camera:

```
python3 send_marker.py 30 1.5 L     # marker 30 deg left, 1.5 m away, arrow left
python3 send_marker.py none         # a frame with no marker
python3 send_marker.py              # type readings one per line
```
