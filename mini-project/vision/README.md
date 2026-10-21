# Computer Vision Subsystem

Runs on the Raspberry Pi.

## Goals

- Detect the marker in the camera image and determine which quadrant (NE, NW,
  SW, SE) it is in
- Show a live camera view with the marker position indicated
- Map the quadrant to goal positions for the left and right wheels
- Send the goal positions to the Arduino
- Display `Goal Position: L R` on the LCD, updated from a separate thread so
  the LCD does not slow the main loop

## Arduino interface

Send goals over I2C to the Arduino at address `0x08` on bus 1:

```python
from smbus2 import SMBus
bus = SMBus(1)
bus.write_i2c_block_data(0x08, 0, [left, right])   # each goal 0 or 1
```

The Arduino doesn't reply.

The full protocol is in [`control/README.md`](../control/README.md#i2c-protocol).
`control/tests/i2c_send_goals.py` sends goals without the vision code.
