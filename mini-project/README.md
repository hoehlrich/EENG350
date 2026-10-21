# EENG350 Mini Project

A camera on the Raspberry Pi acts as the user interface for a two wheel system.
The quadrant of the image containing an ArUco marker sets the goal position of
each wheel, the Pi sends that goal to the Arduino, and the Arduino holds each
wheel at its goal with integral control.

| Quadrant | Left Wheel | Right Wheel |
|----------|------------|-------------|
| NE       | 0          | 0           |
| NW       | 0          | 1           |
| SW       | 1          | 1           |
| SE       | 1          | 0           |

Each wheel has two tape marks, 0 and 1, 180 degrees apart. Both wheels start
with 0 facing up.

## Organization

- `vision/`: Pi code for marker detection, LCD display, and communication with the Arduino (see [`vision/README.md`](vision/README.md))
- `control/`: Arduino position controller, I2C test tools, and Simulink/MATLAB models and step response scripts (see [`control/README.md`](control/README.md))
- `resources/`: assignment handout and reference material

The Pi talks to the Arduino over I2C (Arduino at address `0x08`). The message
format is in [`control/README.md`](control/README.md#i2c-protocol).
