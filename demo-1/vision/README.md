# Vision (Demo 1, Task 1)

`demo1.py` runs on the Raspberry Pi. It finds ArUco marker 0 (`DICT_6X6_50`, 5 cm) in the camera image and shows the signed angle between the camera axis and the marker on the I2C character LCD. A marker to the left of the axis gives a positive angle.

## How it works

- `solvePnP` on the 4 marker corners, using the camera calibration, gives the marker centre `(X, Z)` in camera coordinates.
- The angle is `atan2(-X, Z)`.
- The LCD shows the median of the last 5 angles. The last angle stays up for 1 s if the marker drops out briefly.
- The LCD runs in its own thread so the camera loop never waits on it.

## Requirements

- Python 3 with `opencv-contrib-python` (for `cv2.aruco`) and `numpy`
- `adafruit-circuitpython-charlcd` and `board`/`busio` (Pi only)
- `camera_calibration.npz` in this directory, made by `calibrate_camera.py`. Use the same 1280x720 resolution as the calibration.

## Run

```
python3 demo1.py
```

Press `q` in the camera window or Ctrl+C to quit. With no screen (for example over SSH), only the LCD shows output.
