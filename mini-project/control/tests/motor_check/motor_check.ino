/*
 * EENG350 Mini Project - Motor and encoder wiring check.
 *
 * Run this before position_control.ino, and whenever a wheel runs away or
 * the encoders don't count. It finds out how the robot is actually wired
 * instead of trusting the pin numbers in the code.
 *
 * What it does: drives each motor on its own at +TEST_VOLTAGE for
 * TEST_TIME_MS while watching all four encoder pins (2, 3, 5, 6) and
 * counting how many times each one changes. The pins that change belong to
 * that motor's encoder: the one on 2 or 3 is channel A, the one on 5 or 6 is
 * channel B. Then it decodes the direction from those two pins and compares
 * everything with ENC_A, ENC_B and MOTOR_DIR below, which must match
 * position_control.ino. Lift the wheels so they spin freely. Open the serial
 * monitor at 115200 baud; press reset to run it again.
 *
 * Example of a good result:
 *   Motor 1 (left, pins 7/9):  edges  pin2 258  pin3 0  pin5 0  pin6 258
 *     -> encoder A = pin 2, B = pin 6, 258 counts forward. OK
 *
 * The pins are polled rather than using interrupts, so pins 5 and 6 can be
 * watched too. At test speed each pin changes about once per millisecond,
 * and the loop checks all four pins many times in that time.
 *
 * Hardware (same as position_control.ino)
 *   Pololu dual MC33926 motor driver shield: pin 4 enable, pins 7/8 motor
 *   1/2 direction, pins 9/10 motor 1/2 PWM.
 *   Encoder A (yellow) -> pin 2 or 3, B (white) -> pin 5 or 6
 *   Encoder Vcc (blue) -> 5 V, encoder GND (green) -> GND
 */

// ---- Motor driver pins, index 0 = left (motor 1), 1 = right (motor 2) ----
const uint8_t ENABLE_PIN = 4;
const uint8_t DIR_PIN[2] = {7, 8};
const uint8_t PWM_PIN[2] = {9, 10};

// ---- What position_control.ino expects (keep these the same) ----
const uint8_t ENC_A[2] = {2, 3};
const uint8_t ENC_B[2] = {6, 5};
const int MOTOR_DIR[2] = {-1, -1};       // -1 reverses a motor's direction pin

// ---- Test settings ----
const float Battery_Voltage = 7.0;       // read the actual value off the voltage monitor
const float TEST_VOLTAGE = 2.5;          // enough to overcome friction (V)
const unsigned long TEST_TIME_MS = 300;  // how long each motor is driven
const unsigned long SETTLE_MS = 700;     // let the wheel stop before the next test
const long MIN_EDGES = 50;               // fewer changes than this counts as "no signal"

// Encoder pins to watch: two possible A pins and two possible B pins
const uint8_t WATCH[4] = {2, 3, 5, 6};

// Drives motor m forward on its own and reports what the encoder pins did
void testMotor(uint8_t m) {
    long edges[4] = {0, 0, 0, 0};
    bool last[4];
    long count[2][2] = {{0, 0}, {0, 0}};   // decoded counts for A in {2,3} x B in {5,6}

    for (uint8_t p = 0; p < 4; p++) {
        last[p] = digitalRead(WATCH[p]);
    }

    // Positive voltage: direction pin HIGH, unless MOTOR_DIR reverses it
    digitalWrite(DIR_PIN[m], MOTOR_DIR[m] > 0 ? HIGH : LOW);
    analogWrite(PWM_PIN[m], 255 * TEST_VOLTAGE / Battery_Voltage);

    unsigned long start = millis();
    while (millis() - start < TEST_TIME_MS) {
        bool now[4];
        for (uint8_t p = 0; p < 4; p++) {
            now[p] = digitalRead(WATCH[p]);
        }
        // On a change of either A pin, decode the direction against both B pins
        // the same way position_control.ino does: A != B means forward.
        for (uint8_t a = 0; a < 2; a++) {
            if (now[a] != last[a]) {
                for (uint8_t b = 0; b < 2; b++) {
                    count[a][b] += (now[a] != now[2 + b]) ? 1 : -1;
                }
            }
        }
        for (uint8_t p = 0; p < 4; p++) {
            if (now[p] != last[p]) {
                edges[p]++;
                last[p] = now[p];
            }
        }
    }

    analogWrite(PWM_PIN[m], 0);
    delay(SETTLE_MS);

    // Report the raw edge counts
    Serial.print(m == 0 ? "Motor 1 (left, pins 7/9):  " : "Motor 2 (right, pins 8/10): ");
    Serial.print("edges");
    for (uint8_t p = 0; p < 4; p++) {
        Serial.print("  pin");
        Serial.print(WATCH[p]);
        Serial.print(" ");
        Serial.print(edges[p]);
    }
    Serial.println("");
    Serial.print("  -> ");

    // The busiest A pin and B pin are this motor's encoder
    uint8_t a = (edges[1] > edges[0]) ? 1 : 0;
    uint8_t b = (edges[3] > edges[2]) ? 1 : 0;
    bool haveA = edges[a] >= MIN_EDGES;
    bool haveB = edges[2 + b] >= MIN_EDGES;

    if (!haveA && !haveB) {
        Serial.println("NO SIGNAL on any encoder pin. If the wheel spun, the encoder has no power "
                       "(blue -> 5 V, green -> GND) or its yellow/white wires aren't on pins 2/3/5/6.");
        return;
    }
    if (!haveA) {
        Serial.print("B found on pin ");
        Serial.print(WATCH[2 + b]);
        Serial.println(" but no A signal on pin 2 or 3. Check this encoder's yellow (A) wire - it must be on pin 2 or 3.");
        return;
    }
    if (!haveB) {
        Serial.print("A found on pin ");
        Serial.print(WATCH[a]);
        Serial.println(" but no B signal on pin 5 or 6. Check this encoder's white (B) wire - it must be on pin 5 or 6.");
        return;
    }

    long counts = count[a][b];
    Serial.print("encoder A = pin ");
    Serial.print(WATCH[a]);
    Serial.print(", B = pin ");
    Serial.print(WATCH[2 + b]);
    Serial.print(", ");
    Serial.print(labs(counts));
    Serial.print(counts >= 0 ? " counts forward. " : " counts BACKWARD. ");

    bool pinsOk = (WATCH[a] == ENC_A[m] && WATCH[2 + b] == ENC_B[m]);
    if (pinsOk && counts > 0) {
        Serial.println("OK");
        return;
    }
    if (!pinsOk) {
        Serial.print("Pins don't match the code: set ENC_A[");
        Serial.print(m);
        Serial.print("] = ");
        Serial.print(WATCH[a]);
        Serial.print(" and ENC_B[");
        Serial.print(m);
        Serial.print("] = ");
        Serial.print(WATCH[2 + b]);
        Serial.print(". ");
    }
    if (counts < 0) {
        Serial.print("Flip MOTOR_DIR[");
        Serial.print(m);
        Serial.print("] to ");
        Serial.print(-MOTOR_DIR[m]);
        Serial.print(".");
    }
    Serial.println("");
}

void setup() {
    // Motor driver
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);
    for (uint8_t i = 0; i < 2; i++) {
        pinMode(DIR_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
        analogWrite(PWM_PIN[i], 0);
    }

    // Encoder pins
    for (uint8_t p = 0; p < 4; p++) {
        pinMode(WATCH[p], INPUT_PULLUP);
    }

    Serial.begin(115200);
    Serial.println("Motor/encoder check. Lift the wheels. Starting in 2 s...");
    delay(2000);

    testMotor(0);
    testMotor(1);
    Serial.println("Done. Press reset to run again.");
}

void loop() {
}
