# Vision (Demo 2)

`demo2.py` runs on the Raspberry Pi. For every camera frame it looks for
ArUco marker 0 (`DICT_6X6_50`, 2 in) and the arrow beside it, and sends the
Arduino one message: is the marker visible, the angle and distance to it, and
which way the arrow points. The message format is in
[`../README.md`](../README.md#the-message).

## Layout

| Path | What it is |
|------|------------|
| `demo2.py` | **The demo script.** Camera → marker and arrow → message to the Arduino. Settings at the top |
| `marker_link.py` | The message: its format, `pack_message()`, and `ArduinoLink`, which sends it over I2C |
| `send_marker.py` | Sends readings typed by hand instead of from the camera, to test the Arduino on its own |
| `demo2.service` | systemd unit that starts `demo2.py` when the Pi boots. Install instructions at the top |
| `tests/synthetic_test.py` | Runs the marker and arrow detection on drawn camera frames. Needs no camera or Pi |
| `tests/beacon_left.png`, `tests/beacon_right.png` | The two beacons, taken from the handout. Used by the test, and good for printing practice beacons (print the marker 2 in wide) |

## How it works

- **Marker.** Same as Demo 1: `solvePnP` on the 4 marker corners with the
  camera calibration gives the marker center `(X, Y, Z)` in camera
  coordinates. The angle is `atan2(-X, Z)`, positive when the marker is to
  the left. The distance is `sqrt(X² + Z²)`, the distance along the floor,
  so it doesn't matter if the camera and marker are at different heights.
- **Arrow.** Found by color and side. The marker's corners give its center,
  width and tilt in the image, and from those the script works out where the
  area just left of the marker and the area just right of it are. It counts
  what fraction of the left area is green and what fraction of the right
  area is red. Between `ARROW_MIN_FILL` and `ARROW_MAX_FILL` counts as an
  arrow (the arrow fills about 0.35 of its area). Green on the left means
  the arrow points left, red on the right means right, and neither or both
  is reported as unknown.
- **No smoothing.** Every frame is sent as measured. The Arduino combines
  them: it uses the newest angle and distance, and adds up the arrow
  readings as votes.
- **The LCD** is optional. If it is attached it shows the angle, distance
  and arrow twice a second, from its own thread as in Demo 1. If it isn't,
  the script gives up on it after 10 s and carries on.

This relies on the beacons looking like the handout's: a green arrow on the
left or a red one on the right. If the real ones differ, `find_arrow()` is
the only function that has to change.

## Requirements

- Python 3 with `opencv-contrib-python` (for `cv2.aruco`), `numpy` and `smbus2`
- `adafruit-circuitpython-charlcd` and `board`/`busio` for the LCD (optional)
- `camera_calibration.npz` in this directory. It is the Demo 1 file and is
  not in the repository. Use the same 1280x720 resolution as the
  calibration.
- I2C enabled on the Pi (`raspi-config` → Interface Options). `i2cdetect -y 1`
  should show `08` (the Arduino).

## Run

```
python3 demo2.py
```

Press `q` in the camera window or Ctrl+C to quit. Every 2 s it prints the
frame rate, how many messages the Arduino accepted, and the newest reading.
With a screen attached it shows the camera view with the marker outlined in
blue and the two arrow areas outlined in green and red, each labeled with
the fraction of it that is that color. Without a screen (over SSH, or
started at boot) it only sends.

For the demo it has to start by itself: see `demo2.service`. Stop the
service before running the script by hand, because only one program can use
the camera.

## Tuning

Do this in the demo room, with a screen attached, holding each beacon in
front of the camera at 1 to 6 ft:

| What you see | What to change |
|--------------|----------------|
| Marker not outlined at 6 ft | Focus and lighting first. The marker is only about 30 pixels wide there |
| An arrow area isn't over the arrow | `ARROW_NEAR`, `ARROW_FAR`, `ARROW_HALF_HEIGHT` (in marker widths from the marker center) |
| The number over the arrow is below 0.2 | The color limits are too tight for these lights: widen `GREEN_HUE` or the red limits, or lower `MIN_SATURATION` |
| The number on the side with no arrow is above 0.1 | Something behind the beacon counts as that color: tighten the hue limits or raise `MIN_SATURATION` |
| Marker lost while the robot turns | Set `EXPOSURE` to a short fixed time, or slow the search (`SEARCH_PHI_DOT` in `beacon.ino`), or make it stop and look (`SEARCH_STEP_DEG`) |

## Testing without the robot

```
python3 tests/synthetic_test.py
```

draws each beacon as a camera would see it from 0.2 to 1.83 m (the camera is
0.2 m from the marker when the robot stops, and 1.83 m is 6 ft), 20° to
either side, and turned up to 30°, on a background with other red and green
things in it. It checks the angle (within 0.5°), the distance (within 5%)
and the arrow. `--save` writes the frames to `tests/frames/` to look at.

It passes all 126 cases. That shows the geometry, the signs and the arrow
areas are right. It says nothing about the real camera's focus, motion blur,
or how the printed colors look under the room's lights, which is what the
tuning above is for.

To test the Arduino without the camera, `send_marker.py` sends whatever
reading you type:

```
python3 send_marker.py 30 1.5 L     # marker 30 deg left, 1.5 m away, arrow left
python3 send_marker.py              # type readings one per line
```
