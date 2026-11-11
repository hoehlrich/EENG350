# EENG 350 SEED Lab - Demo 2: the Pi -> Arduino message
#
# The one place on the Pi that knows what the message looks like. demo2.py
# sends one of these for every camera frame, send_marker.py sends them by
# hand. The Arduino never replies.
#
# Message (I2C, Arduino at address 0x08, multi-byte values little-endian):
#
#     byte 0     MARKER_MSG_ID (0x01)
#     byte 1     visible       1 if marker 0 is in this frame, else 0
#     byte 2-3   angle_cdeg    int16, camera axis to marker, 0.01 deg, + = left
#     byte 4-5   distance_mm   uint16, camera to marker along the floor, mm
#     byte 6     arrow         0 unknown, 1 left, 2 right
#     byte 7     checksum      low byte of the sum of bytes 1 to 6
#
# To change the message: edit MESSAGE_FORMAT and pack_message() here, and
# struct MarkerMsg in control/beacon/beacon.ino (and the copy in
# control/tests/marker_echo/marker_echo.ino) to match. The Arduino drops
# any message whose length, ID or checksum is wrong, so a mismatch shows up
# as a robot that never sees the marker, not as one that drives off.

import struct

I2C_BUS = 1
ARDUINO_ADDRESS = 0x08
MARKER_MSG_ID = 0x01

ARROW_UNKNOWN = 0
ARROW_LEFT = 1
ARROW_RIGHT = 2
ARROW_NAMES = {ARROW_UNKNOWN: "?", ARROW_LEFT: "left", ARROW_RIGHT: "right"}

# Bytes 1 to 6: visible (B), angle_cdeg (h), distance_mm (H), arrow (B).
# "<" = little-endian with no padding, the same as the Arduino's packed struct.
MESSAGE_FORMAT = "<BhHB"


def pack_message(visible, angle_deg=0.0, distance_m=0.0, arrow=ARROW_UNKNOWN):
    """Return bytes 1 to 7 of the message (everything after the ID byte)."""
    if not visible:
        angle_deg, distance_m, arrow = 0.0, 0.0, ARROW_UNKNOWN
    # Clip to what fits in 16 bits: +-327.67 deg and 0 to 65.535 m
    angle_cdeg = max(-32768, min(32767, round(angle_deg * 100)))
    distance_mm = max(0, min(65535, round(distance_m * 1000)))
    payload = struct.pack(MESSAGE_FORMAT, 1 if visible else 0,
                          angle_cdeg, distance_mm, arrow)
    return payload + bytes([sum(payload) & 0xFF])


class ArduinoLink:
    """Sends marker messages to the Arduino.

    If there is no I2C bus (smbus2 missing, or not on a Pi) it says so once
    and carries on without sending, so the vision code can be tried on any
    computer with a camera.
    """

    def __init__(self):
        try:
            from smbus2 import SMBus
            self.bus = SMBus(I2C_BUS)
        except (ImportError, OSError) as error:
            print("No I2C bus, nothing will be sent to the Arduino:", error)
            self.bus = None

    def send(self, visible, angle_deg=0.0, distance_m=0.0, arrow=ARROW_UNKNOWN):
        """Send one message. Returns False if the Arduino did not take it."""
        if self.bus is None:
            return False
        data = pack_message(visible, angle_deg, distance_m, arrow)
        try:
            self.bus.write_i2c_block_data(ARDUINO_ADDRESS, MARKER_MSG_ID, list(data))
            return True
        except OSError:
            # Arduino unplugged, resetting or busy: the next frame tries again
            return False

    def close(self):
        if self.bus is not None:
            self.bus.close()
