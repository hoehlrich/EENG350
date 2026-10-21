/*
 * EENG350 Mini Project - I2C link test, Arduino side.
 *
 * Checks the Pi -> Arduino link without the motors or the controller.
 * Every message the Pi writes is printed over Serial as raw bytes, followed
 * by whether position_control.ino would accept it.
 *
 * Run control/tests/i2c_send_goals.py on the Pi and watch the serial
 * monitor at 115200 baud. `i2cdetect -y 1` on the Pi should show 08.
 *
 * I2C protocol (same as position_control.ino)
 *   Pi -> Arduino, 3 bytes: [0, left_goal, right_goal], goals 0 or 1.
 *   The Arduino sends nothing back.
 *
 * Hardware
 *   Raspberry Pi GPIO2 (SDA, pin 3)  -> Arduino A4 (SDA)
 *   Raspberry Pi GPIO3 (SCL, pin 5)  -> Arduino A5 (SCL)
 *   Raspberry Pi GND (pin 6)         -> Arduino GND
 */

#include <Wire.h>

const uint8_t I2C_ADDRESS = 0x08;
const uint8_t MAX_MSG = 16;          // longest message kept for printing

// Filled by receiveEvent, printed by loop
volatile uint8_t msg[MAX_MSG];
volatile uint8_t msg_len = 0;
volatile bool new_msg = false;

// Called when the Pi writes. Stores the bytes for loop() to print.
void receiveEvent(int howMany) {
    uint8_t n = 0;

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
    Serial.println("Ready! Waiting for I2C messages at address 0x08");
}

void loop() {
    uint8_t copy[MAX_MSG];
    uint8_t n;

    if (new_msg) {
        // Copy the message with interrupts off so it can't change mid-print
        noInterrupts();
        n = msg_len;
        for (uint8_t i = 0; i < n; i++) {
            copy[i] = msg[i];
        }
        new_msg = false;
        interrupts();

        Serial.print("Received ");
        Serial.print(n);
        Serial.print(" bytes:");
        for (uint8_t i = 0; i < n; i++) {
            Serial.print(" ");
            Serial.print(copy[i]);
        }

        if (n == 3 && copy[0] == 0 && copy[1] <= 1 && copy[2] <= 1) {
            Serial.print("  -> Goal Position: ");
            Serial.print(copy[1]);
            Serial.print(" ");
            Serial.println(copy[2]);
        } else {
            Serial.println("  -> ignored (not a valid goal message)");
        }
    }
}
