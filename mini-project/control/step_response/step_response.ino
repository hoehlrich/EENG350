/*
 * EENG350 Mini Project - Open loop motor step response, both wheels.
 *
 * Measures each motor's DC gain K and sigma for the transfer function
 *
 *     velocity(s) / voltage(s) = K*sigma / (s + sigma)
 *
 * used by the Simulink models in control/matlab. Put the results in
 * motor_params.m.
 *
 * Voltage applied to both motors:
 *   0 V            for 0 <= t < 1 s
 *   Step_Voltage   for 1 <= t < 3 s
 *   0 V            after 3 s
 *
 * Output: five tab-separated columns every 10 ms for the first 3 s:
 *
 *     time (s)   left voltage (V)   left velocity (rad/s)
 *                right voltage (V)  right velocity (rad/s)
 *
 * The data starts after "Ready!" and ends with "Finished", for
 * control/matlab/readArduinoData.m. Then use run_openloop.m to fit K and
 * sigma and compare the simulation to the data. Lift the wheels so they
 * spin freely.
 *
 * Hardware
 *   Pololu dual MC33926 motor driver shield: pin 4 enable, pins 7/8 motor
 *   1/2 voltage sign, pins 9/10 motor 1/2 PWM.
 *   Motor 1 = LEFT wheel.  Encoder A (yellow) -> pin 2, B (white) -> pin 6
 *   Motor 2 = RIGHT wheel. Encoder A (yellow) -> pin 3, B (white) -> pin 5
 *   Encoder Vcc (blue) -> 5 V, encoder GND (green) -> GND
 *
 * Sign convention: a positive voltage must give positive counts. MOTOR_DIR
 * flips a motor's direction pin, same values as position_control.ino.
 */

// ---- Motor driver pins, index 0 = left (motor 1), 1 = right (motor 2) ----
const uint8_t ENABLE_PIN = 4;
const uint8_t SIGN_PIN[2] = { 7, 8 };
const uint8_t PWM_PIN[2] = { 9, 10 };

// ---- Encoder pins ----
const uint8_t ENC_A[2] = {2, 3};
const uint8_t ENC_B[2] = {6, 5};

const float COUNTS_PER_REV = 3200;  // 64 counts per motor rev * 50:1 gearbox

// -1 reverses a motor's direction pin so positive voltage gives positive
// counts. Same values as position_control.ino.
const int MOTOR_DIR[2] = {-1, -1};  // {left, right}

// ---- Experiment settings ----
const float Battery_Voltage = 7.0;  // read the actual value off the voltage monitor
const float Step_Voltage = 3.0;     // mid-range step voltage (V)

// analogWrite value that gives Step_Voltage on average
const unsigned int Step_PWM = 255 * Step_Voltage / Battery_Voltage;

// ---- Timing ----
unsigned long desired_Ts_ms = 10;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

float last_pos[2] = { 0, 0 };  // wheel positions on the previous pass (rad)
bool finished = false;         // true once "Finished" has been printed

// ---- Encoders ----
// Same method as position_control.ino: the channel A interrupt counts by
// twos and myEnc() adds the missing +/-1 from channel B.
volatile long counter[2] = { 0, 0 };
volatile bool lastA[2], lastB[2];

// Shared by both encoder interrupts
void updateEncoder(uint8_t i) {
    bool a = digitalRead(ENC_A[i]);
    bool b = digitalRead(ENC_B[i]);

    if (a != b) {
        counter[i] += 2;
    } else {
        counter[i] -= 2;
    }

    lastA[i] = a;
    lastB[i] = b;
}

// Runs on every edge of the left encoder channel A
void handleA1() {
    updateEncoder(0);
}

// Runs on every edge of the right encoder channel A
void handleA2() {
    updateEncoder(1);
}

// Returns the current position of wheel i (0 = left, 1 = right) in counts
long myEnc(uint8_t i) {
    long count;
    bool a, b, snapA, snapB;

    // Take a consistent snapshot of the 4 byte counter
    noInterrupts();
    a = digitalRead(ENC_A[i]);
    b = digitalRead(ENC_B[i]);
    count = counter[i];
    snapA = lastA[i];
    snapB = lastB[i];
    interrupts();

    // Only B can have changed since the last interrupt
    if (a == snapA && b != snapB) {
        if (snapA != snapB) {
            count += 1;
        } else {
            count -= 1;
        }
    }

    return count;
}

void setup() {
    // Motor driver, positive voltage on both motors
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);
    for (uint8_t i = 0; i < 2; i++) {
        pinMode(SIGN_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
        digitalWrite(SIGN_PIN[i], MOTOR_DIR[i] > 0 ? HIGH : LOW);
        analogWrite(PWM_PIN[i], 0);
    }

    // Encoders
    for (uint8_t i = 0; i < 2; i++) {
        pinMode(ENC_A[i], INPUT_PULLUP);
        pinMode(ENC_B[i], INPUT_PULLUP);
        lastA[i] = digitalRead(ENC_A[i]);
        lastB[i] = digitalRead(ENC_B[i]);
    }
    attachInterrupt(digitalPinToInterrupt(ENC_A[0]), handleA1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A[1]), handleA2, CHANGE);

    Serial.begin(115200);
    Serial.println("Ready!");
    last_time_ms = millis();
    start_time_ms = last_time_ms;
}

void loop() {
    float pos_rad;
    float velocity[2];
    unsigned int PWM;
    float Voltage;

    // timestamp in seconds
    current_time = (float)(last_time_ms - start_time_ms) / 1000;

    // Velocity = change in position since the last pass / sample time
    for (uint8_t i = 0; i < 2; i++) {
        pos_rad = 2 * PI * (float)myEnc(i) / COUNTS_PER_REV;
        velocity[i] = (pos_rad - last_pos[i]) / ((float)desired_Ts_ms / 1000);
        last_pos[i] = pos_rad;
    }

    // Step input: Step_Voltage between 1 and 3 seconds, zero otherwise
    if (current_time >= 1 && current_time < 3) {
        PWM = Step_PWM;
    } else {
        PWM = 0;
    }
    analogWrite(PWM_PIN[0], PWM);
    analogWrite(PWM_PIN[1], PWM);

    // Average voltage of the PWM signal applied to the motors
    Voltage = Battery_Voltage * PWM / 255;

    // Only display the first 3 seconds of data
    if (current_time < 3) {
        Serial.print(current_time);
        Serial.print("\t");
        Serial.print(Voltage);
        Serial.print("\t");
        Serial.print(velocity[0]);
        Serial.print("\t");
        Serial.print(Voltage);
        Serial.print("\t");
        Serial.print(velocity[1]);
        Serial.println("");
    } else if (!finished) {
        Serial.println("Finished");
        finished = true;
    }

    while (millis() < last_time_ms + desired_Ts_ms) {
        // wait until desired time passes to go top of the loop
    }
    last_time_ms = millis();
}
