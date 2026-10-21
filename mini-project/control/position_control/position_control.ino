/*
 * EENG350 Mini Project - Two wheel position control.
 *
 * Holds each wheel at a goal position set by the Raspberry Pi over I2C.
 * Each wheel has two tape marks, 0 and 1, 180 degrees apart, and 0 faces up
 * when the Arduino is reset. A goal of 0 means 0 rad and a goal of 1 means
 * pi rad, measured from the position at reset. Returning to 0 unwinds back
 * to 0 rad rather than continuing on to 2*pi.
 *
 * Every 10 ms, for each wheel:
 *     pos_error      = desired_pos - actual_pos
 *     integral_error = integral_error + pos_error*dt
 *     desired_speed  = Kp_pos*pos_error + Ki_pos*integral_error
 *     Voltage        = Kp_vel*(desired_speed - actual_speed)
 *
 * Deadband: within DEADBAND_RAD of the goal the voltage is set to 0 and the
 * integral is frozen. Otherwise a voltage too small to overcome friction
 * sits on the motor and it whines. The Simulink model has no deadband.
 *
 * I2C protocol (Arduino is the slave at address 0x08)
 *   Pi -> Arduino, 3 bytes: [0, left_goal, right_goal]. Each goal is 0 or 1.
 *     From Python: bus.write_i2c_block_data(0x08, 0, [left, right])
 *     Any other message is ignored. The Arduino sends nothing back.
 *
 * Modes
 *   Normal (STEP_TEST not defined): goals come from the Pi
 *
 * Note: opening the USB serial port resets the Uno, which also resets the
 * encoder counts. Set both wheels to 0 before opening the serial monitor.
 *
 * Hardware
 *   Pololu dual MC33926 motor driver shield: pin 4 enable, pins 7/8 motor
 *   1/2 direction, pins 9/10 motor 1/2 PWM.
 *   Motor 1 = LEFT wheel.  Encoder A (yellow) -> pin 2, B (white) -> pin 6
 *   Motor 2 = RIGHT wheel. Encoder A (yellow) -> pin 3, B (white) -> pin 5
 *   Encoder Vcc (blue) -> 5 V, encoder GND (green) -> GND
 *   Raspberry Pi GPIO2 (SDA, pin 3)  -> Arduino A4 (SDA)
 *   Raspberry Pi GPIO3 (SCL, pin 5)  -> Arduino A5 (SCL)
 *   Raspberry Pi GND (pin 6)         -> Arduino GND
 */

#include <Wire.h>

// Uncomment to run the quadrant table as a step test instead of listening to the Pi
// #define STEP_TEST

// ------------------------------ settings ------------------------------------
const float Kp_vel = 3.2;
const float Kp_pos = 5.0;
const float Ki_pos = 2.0;
const float MAX_SPEED = 6.0;
const float Battery_Voltage = 7.0;
const float DEADBAND_RAD = 0.02;

const float COUNTS_PER_REV = 1600.0;
const int MOTOR_DIR[2] = {-1, -1};

const unsigned long desired_Ts_ms = 10;
const uint8_t I2C_ADDRESS = 0x08;

#ifdef STEP_TEST
const int SUITE_GOALS[][2] = {
    {0, 0},   // NE (start)
    {0, 1},   // NW: right 0 -> 1
    {1, 1},   // SW: left  0 -> 1
    {1, 0},   // SE: right 1 -> 0
    {0, 0},   // NE: left  1 -> 0
};
const int SUITE_STAGES = sizeof(SUITE_GOALS) / sizeof(SUITE_GOALS[0]);
const float STEP_TIME = 1.0;
const float STAGE_TIME = 3.0;
const float RUN_TIME = STEP_TIME + (SUITE_STAGES - 1) * STAGE_TIME;
bool finished = false;
#else
const unsigned int PRINT_EVERY = 10;
unsigned int pass_count = 0;
#endif
// ----------------------------------------------------------------------------

const int EN_PIN       = 4;
const int DIR_PIN[2]   = {7, 8};
const int PWM_PIN[2]   = {9, 10};
const int ENC_A_PIN[2] = {2, 3};
const int ENC_B_PIN[2] = {6, 5};

volatile long count[2] = {0, 0};       // encoder counts, updated in the interrupts

// Runs on every edge of channel A: if B differs from A the wheel turns forward.
void encoder1ISR() {
    if (digitalRead(ENC_A_PIN[0]) != digitalRead(ENC_B_PIN[0])) count[0]++;
    else                                                        count[0]--;
}
void encoder2ISR() {
    if (digitalRead(ENC_A_PIN[1]) != digitalRead(ENC_B_PIN[1])) count[1]++;
    else                                                        count[1]--;
}

long readCount(int i) {
    noInterrupts();
    long c = count[i];
    interrupts();
    return c;
}

// Controller state
int goal[2] = {0, 0};                  // goal tape mark for each wheel, 0 or 1
float prev_pos_rad[2] = {0, 0};        // wheel positions at the previous sample [rad]
float integral_error[2] = {0, 0};      // integral of the position error [rad*s]
unsigned long next_ms;                 // when the next sample is due
unsigned long start_us, last_us;       // start time and previous sample [us]

volatile uint8_t i2c_goal[2] = {0, 0};

// Called when the Pi writes. Accepts only [0, left, right] with goals 0 or 1.
void receiveEvent(int howMany) {
    uint8_t msg[3];
    int n = 0;
    while (Wire.available()) {             // read everything so the buffer is empty
        uint8_t c = Wire.read();
        if (n < 3) msg[n] = c;
        n++;
    }
    if (n == 3 && msg[0] == 0 && msg[1] <= 1 && msg[2] <= 1) {
        i2c_goal[0] = msg[1];
        i2c_goal[1] = msg[2];
    }
}

// The sign of the voltage goes to the DIR pin, its size to the PWM duty.
void applyVoltage(int i, float Voltage) {
    bool forward = (Voltage > 0);
    if (MOTOR_DIR[i] < 0) forward = !forward;
    digitalWrite(DIR_PIN[i], forward ? HIGH : LOW);
    float PWM = 255.0 * fabs(Voltage) / Battery_Voltage;   // 255 = full battery voltage
    analogWrite(PWM_PIN[i], min(PWM, 255.0));
}

void setup() {
    Serial.begin(115200);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
    for (int i = 0; i < 2; i++) {
        pinMode(DIR_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
        pinMode(ENC_A_PIN[i], INPUT_PULLUP);
        pinMode(ENC_B_PIN[i], INPUT_PULLUP);
        analogWrite(PWM_PIN[i], 0);
    }

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[0]), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[1]), encoder2ISR, CHANGE);

#ifndef STEP_TEST
    Wire.begin(I2C_ADDRESS);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.onReceive(receiveEvent);
#endif

    Serial.println("Ready!");
#ifndef STEP_TEST
    Serial.println("time\tgoalL\tgoalR\tposL\tposR\tVL\tVR");
#endif
    start_us = micros();
    last_us  = start_us;
    next_ms  = millis() + desired_Ts_ms;
}

void loop() {
    while (millis() < next_ms) { }  // wait for the next sample time
    next_ms += desired_Ts_ms;

    unsigned long now_us = micros();
    float dt = (now_us - last_us) * 1e-6;                // time since last sample [s]
    float current_time = (now_us - start_us) * 1e-6;     // time since start [s]
    last_us = now_us;

    // Goals for this sample
#ifdef STEP_TEST
    // Stage 0 until STEP_TIME, then the next stage every STAGE_TIME seconds
    int stage = 0;
    if (current_time >= STEP_TIME) stage = 1 + (int)((current_time - STEP_TIME) / STAGE_TIME);
    if (stage >= SUITE_STAGES) stage = SUITE_STAGES - 1;   // hold the last pair
    goal[0] = SUITE_GOALS[stage][0];
    goal[1] = SUITE_GOALS[stage][1];
#else
    noInterrupts();
    goal[0] = i2c_goal[0];
    goal[1] = i2c_goal[1];
    interrupts();
#endif

    float pos_rad[2];
    float Voltage[2];

    for (int i = 0; i < 2; i++) {
        // speed = change in wheel position [rad] / sample time [s]
        pos_rad[i] = 2 * PI * (float)readCount(i) / COUNTS_PER_REV;
        float actual_speed = (pos_rad[i] - prev_pos_rad[i]) / dt;
        prev_pos_rad[i] = pos_rad[i];

        // Outer loop: PI position controller gives the desired speed
        float desired_pos = goal[i] * PI;
        float pos_error = desired_pos - pos_rad[i];
        integral_error[i] += pos_error * dt;
        float desired_speed = Kp_pos * pos_error + Ki_pos * integral_error[i];

        bool saturated = false;
        if (desired_speed >  MAX_SPEED) { desired_speed =  MAX_SPEED; saturated = true; }
        if (desired_speed < -MAX_SPEED) { desired_speed = -MAX_SPEED; saturated = true; }

        // Inner loop: proportional velocity controller
        float error = desired_speed - actual_speed;
        Voltage[i] = Kp_vel * error;

        // the motor cannot get more than the battery voltage
        if (Voltage[i] >  Battery_Voltage) { Voltage[i] =  Battery_Voltage; saturated = true; }
        if (Voltage[i] < -Battery_Voltage) { Voltage[i] = -Battery_Voltage; saturated = true; }

        // Anti-windup: the motor can't respond to a bigger command
        if (saturated) integral_error[i] -= pos_error * dt;

        // Deadband: close enough to the goal
        if (fabs(pos_error) < DEADBAND_RAD) {
            Voltage[i] = 0;
            if (!saturated) integral_error[i] -= pos_error * dt;
        }

        applyVoltage(i, Voltage[i]);
    }

#ifdef STEP_TEST
    if (current_time < RUN_TIME) {
        Serial.print(current_time, 3);
        for (int i = 0; i < 2; i++) {
            Serial.print("\t");
            Serial.print(Voltage[i], 3);
            Serial.print("\t");
            Serial.print(pos_rad[i], 3);
        }
        Serial.print("\t");
        Serial.print(goal[0]);
        Serial.print("\t");
        Serial.print(goal[1]);
        Serial.println("");
    } else if (!finished) {
        Serial.println("Finished");
        finished = true;
    }
#else
    // Status line for debugging, 10 times a second
    if (++pass_count >= PRINT_EVERY) {
        pass_count = 0;
        Serial.print(current_time, 2);
        Serial.print("\t");
        Serial.print(goal[0]);
        Serial.print("\t");
        Serial.print(goal[1]);
        Serial.print("\t");
        Serial.print(pos_rad[0], 3);
        Serial.print("\t");
        Serial.print(pos_rad[1], 3);
        Serial.print("\t");
        Serial.print(Voltage[0], 2);
        Serial.print("\t");
        Serial.print(Voltage[1], 2);
        Serial.println("");
    }
#endif
}
