/*
 * EENG350 Demo 2 - I2C link test, Arduino side.
 *
 * Checks the Pi -> Arduino link without the motors or the controller.
 * Every message the Pi writes is printed over Serial as raw bytes, followed
 * by what beacon.ino would make of it, or why it would drop it.
 *
 * Run vision/demo2.py or vision/send_marker.py on the Pi and watch the
 * serial monitor at 115200 baud. `i2cdetect -y 1` on the Pi should show 08.
 * If messages arrive faster than they can be printed, some are skipped and
 * the next line says how many.
 *
 * I2C protocol (same as beacon.ino, see ../../../README.md)
 *     byte 0     MARKER_MSG_ID (0x01)
 *     byte 1     visible       1 if marker 0 is in this frame, else 0
 *     byte 2-3   angle_cdeg    int16, camera axis to marker, 0.01 deg, + = left
 *     byte 4-5   distance_mm   uint16, camera to marker along the floor, mm
 *     byte 6     arrow         0 unknown, 1 left, 2 right
 *     byte 7     checksum      low byte of the sum of bytes 1 to 6
 *   The Arduino sends nothing back.
 *
 * Hardware
 *   Raspberry Pi GPIO2 (SDA, pin 3)  -> Arduino A4 (SDA)
 *   Raspberry Pi GPIO3 (SCL, pin 5)  -> Arduino A5 (SCL)
 *   Raspberry Pi GND (pin 6)         -> Arduino GND
 */

#include <Wire.h>

const uint8_t I2C_ADDRESS   = 0x08;
const uint8_t MARKER_MSG_ID = 0x01;
const uint8_t MAX_MSG = 16;          // longest message kept for printing

// Keep the same as in beacon.ino
struct __attribute__((packed)) MarkerMsg {
    uint8_t  visible;       // 1 if marker 0 is in this frame
    int16_t  angle_cdeg;    // camera axis to marker [0.01 deg], positive = left
    uint16_t distance_mm;   // camera to marker along the floor [mm]
    uint8_t  arrow;         // 0 unknown, 1 left, 2 right
};
const uint8_t MSG_LENGTH = sizeof(MarkerMsg) + 2;   // + ID byte + checksum

// Filled by receiveEvent, printed by loop
volatile uint8_t msg[MAX_MSG];
volatile uint8_t msg_len = 0;
volatile bool new_msg = false;
volatile unsigned int skipped = 0;   // messages overwritten before they were printed

// Called when the Pi writes. Stores the bytes for loop() to print.
void receiveEvent(int howMany) {
    uint8_t n = 0;

    if (new_msg) skipped++;
    while (Wire.available()) {
        uint8_t c = Wire.read();
        if (n < MAX_MSG) {
            msg[n] = c;
            n++;
        }
    }
    msg_len = n;
    new_msg = true;
}

void setup() {
    // I2C slave with the internal 5 V pull-ups turned off (the Pi has its own)
    Wire.begin(I2C_ADDRESS);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.onReceive(receiveEvent);

    Serial.begin(115200);
    Serial.println(F("Ready! Waiting for I2C messages at address 0x08"));
}

void loop() {
    uint8_t copy[MAX_MSG];
    uint8_t n;
    unsigned int missed;

    if (!new_msg) return;

    // Copy the message with interrupts off so it can't change mid-print
    noInterrupts();
    n = msg_len;
    for (uint8_t i = 0; i < n; i++) {
        copy[i] = msg[i];
    }
    missed = skipped;
    skipped = 0;
    new_msg = false;
    interrupts();

    if (missed > 0) {
        Serial.print(F("("));
        Serial.print(missed);
        Serial.println(F(" not printed)"));
    }
    Serial.print(n);
    Serial.print(F(" bytes:"));
    for (uint8_t i = 0; i < n; i++) {
        Serial.print(F(" "));
        Serial.print(copy[i]);
    }
    Serial.print(F("  -> "));

    // The same three checks as beacon.ino
    if (n != MSG_LENGTH) {
        Serial.print(F("dropped: wrong length, expected "));
        Serial.println(MSG_LENGTH);
        return;
    }
    if (copy[0] != MARKER_MSG_ID) {
        Serial.println(F("dropped: wrong message ID"));
        return;
    }
    uint8_t sum = 0;
    for (uint8_t i = 1; i < MSG_LENGTH - 1; i++) sum += copy[i];
    if (sum != copy[MSG_LENGTH - 1]) {
        Serial.print(F("dropped: checksum should be "));
        Serial.println(sum);
        return;
    }

    MarkerMsg m;
    memcpy(&m, copy + 1, sizeof(MarkerMsg));
    if (!m.visible) {
        Serial.println(F("no marker"));
        return;
    }
    Serial.print(F("marker at "));
    Serial.print(m.angle_cdeg / 100.0, 2);
    Serial.print(F(" deg, "));
    Serial.print(m.distance_mm / 1000.0, 3);
    Serial.print(F(" m, arrow "));
    Serial.println(m.arrow == 1 ? F("left") : m.arrow == 2 ? F("right") : F("unknown"));
}
