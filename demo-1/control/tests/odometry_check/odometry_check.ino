/*
 * EENG350 Demo 1 - Odometry check and calibration.
 *
 * Checks the signs and scale of rho (distance) and phi (heading) without
 * any wheel slip, by moving the robot by hand. Motors stay off unless you
 * ask for a pulse. Open the serial monitor at 115200 baud. Five times a
 * second it prints
 *
 *     countL  countR  rho (in)  phi (deg)
 *
 * countL and countR are the encoder counts, which go up when that wheel
 * rolls forward.
 *
 * Serial commands (type and press Enter):
 *   z   zero the counts
 *   m   pulse each motor forward on its own at PULSE_VOLTAGE for PULSE_MS
 *       (wheels up or on the floor), left first. Prints how far each
 *       wheel's count moved. Only the pulsed wheel should move, and its
 *       count should go up. Down: flip that wheel's MOTOR_SIGN. The other
 *       wheel moved instead: the motor pins are swapped (DIR_PIN, PWM_PIN).
 *   f   static friction, driving: raises the voltage on both wheels until
 *       the robot starts to move, and prints that voltage
 *   t   static friction, turning: the same with the wheels in opposite
 *       directions. Use about the smaller of f and t as V_STATIC in
 *       drive.ino. Try each a few times on the demo floor.
 *
 * 1. Signs. Push the robot straight forward a few inches: both counts
 *    must go up (flip ENC_SIGN for a wheel whose count goes down), rho
 *    must grow and phi stay near 0. Turn the robot left (CCW from above)
 *    by hand: phi must grow. Then send m to check the motors.
 *
 * 2. Wheel radius. Send z, push the robot straight along a tape measure
 *    exactly D (say 120 in), keeping the center hole on the tape. Then
 *        WHEEL_RADIUS_M (new) = WHEEL_RADIUS_M * D / rho
 *
 * 3. Wheel base. Send z, spin the robot in place about the center hole by
 *    N full turns (N = 5 to average out where you stop). Then
 *        WHEEL_BASE_M (new) = WHEEL_BASE_M * phi / (N * 360)
 *    Do step 2 first, since phi also scales with WHEEL_RADIUS_M.
 *
 * Put the new values in drive.ino, robot_step_response.ino, this sketch,
 * and control/matlab/robot_params.m.
 */

// ------------------------------ settings ------------------------------------
const float WHEEL_RADIUS_M = 0.0725;      // same as drive.ino
const float WHEEL_BASE_M   = 0.36;

const float COUNTS_PER_REV = 1600.0;
// One sign per wheel for each part, {left, right}:
//   MOTOR_SIGN: flip it if a positive voltage turns that wheel backward
//   ENC_SIGN:   flip it if that wheel's count goes down when the robot rolls forward
const int MOTOR_SIGN[2] = {-1, 1};
const int ENC_SIGN[2]   = {1, -1};

const float Battery_Voltage = 7.5;
const float PULSE_VOLTAGE = 4.0;         // V on each wheel
const unsigned long PULSE_MS = 500;
const unsigned long PRINT_MS = 200;
// ----------------------------------------------------------------------------

const int EN_PIN       = 4;
const int DIR_PIN[2]   = {8, 7};    // left motor is on M2, right on M1
const int PWM_PIN[2]   = {10, 9};
const int ENC_A_PIN[2] = {2, 3};
const int ENC_B_PIN[2] = {6, 5};

volatile long count[2] = {0, 0};

// Same encoder code as drive.ino
void encoder1ISR() {
    if (digitalRead(ENC_A_PIN[0]) != digitalRead(ENC_B_PIN[0])) count[0] += ENC_SIGN[0];
    else                                                        count[0] -= ENC_SIGN[0];
}
void encoder2ISR() {
    if (digitalRead(ENC_A_PIN[1]) != digitalRead(ENC_B_PIN[1])) count[1] += ENC_SIGN[1];
    else                                                        count[1] -= ENC_SIGN[1];
}

long readCount(int i) {
    noInterrupts();
    long c = count[i];
    interrupts();
    return c;
}

void applyVoltage(int i, float Voltage) {
    digitalWrite(DIR_PIN[i], (MOTOR_SIGN[i] * Voltage > 0) ? HIGH : LOW);
    float PWM = 255.0 * fabs(Voltage) / Battery_Voltage;
    analogWrite(PWM_PIN[i], min(PWM, 255.0));
}

// The driver pulls its status flag LOW on a fault (overcurrent, short,
// overheating, battery too low). Some faults latch the outputs off, which
// looks like the shield lost power. Toggling the enable pin clears it.
// Faults that keep coming back mean the cause is still there (usually a low
// or sagging battery), so after MAX_FAULTS the motors are left off.
const int NSF_PIN = 12;
const unsigned int MAX_FAULTS = 5;
unsigned int fault_count = 0;

void checkDriverFault() {
    if (fault_count >= MAX_FAULTS) return;          // given up, motors stay off
    if (digitalRead(NSF_PIN) == HIGH) return;
    fault_count++;
    digitalWrite(EN_PIN, LOW);
    Serial.print("# driver fault ");
    Serial.print(fault_count);
    Serial.print(" at t = ");
    Serial.print(millis() / 1000.0, 2);
    if (fault_count >= MAX_FAULTS) {
        Serial.println(" s, too many: motors left off, check the battery");
        return;
    }
    delayMicroseconds(10);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(" s, cleared");
}

unsigned long last_print = 0;

// Drives motor m forward on its own, then reports how far each wheel's
// count moved. Only wheel m should move, and its count should go up.
void pulseMotor(int m) {
    long start[2] = {readCount(0), readCount(1)};
    applyVoltage(m, PULSE_VOLTAGE);
    delay(PULSE_MS);
    applyVoltage(m, 0);
    delay(300);   // let it coast to a stop before reading
    long moved[2] = {readCount(0) - start[0], readCount(1) - start[1]};
    int other = 1 - m;

    Serial.print(m == 0 ? "left motor:  " : "right motor: ");
    Serial.print("left count ");
    Serial.print(moved[0]);
    Serial.print(", right count ");
    Serial.print(moved[1]);
    Serial.print("  ");
    if (labs(moved[other]) > labs(moved[m]))  Serial.println("WRONG WHEEL moved: motor pins are swapped");
    else if (labs(moved[m]) < 20)             Serial.println("(barely moved)");
    else if (moved[m] > 0)                    Serial.println("ok");
    else                                      Serial.println("went backward: flip its MOTOR_SIGN");
}

// Raises the wheel voltage in small steps until either wheel has turned
// MOVE_COUNTS, then prints the voltage. turn = false drives both wheels
// forward, turn = true drives the right wheel forward and the left back.
void findStaticFriction(bool turn) {
    const float STEP_V = 0.05;
    const unsigned long STEP_MS = 100;
    const long MOVE_COUNTS = 8;
    long start[2] = {readCount(0), readCount(1)};
    float V = 0;
    bool moved = false;
    while (!moved && V < Battery_Voltage / 2) {
        V += STEP_V;
        applyVoltage(0, turn ? -V : V);
        applyVoltage(1, V);
        delay(STEP_MS);
        for (int i = 0; i < 2; i++) {
            if (labs(readCount(i) - start[i]) >= MOVE_COUNTS) moved = true;
        }
    }
    applyVoltage(0, 0);
    applyVoltage(1, 0);
    Serial.print(turn ? "turning" : "driving");
    Serial.print(" static friction: ");
    Serial.print(V, 2);
    Serial.println(" V");
}

void setup() {
    Serial.begin(115200);

    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
    pinMode(NSF_PIN, INPUT_PULLUP);
    for (int i = 0; i < 2; i++) {
        pinMode(DIR_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
        pinMode(ENC_A_PIN[i], INPUT_PULLUP);
        pinMode(ENC_B_PIN[i], INPUT_PULLUP);
        analogWrite(PWM_PIN[i], 0);
    }

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[0]), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[1]), encoder2ISR, CHANGE);

    // What this upload was built with, to check against the file
    Serial.print("MOTOR_SIGN ");
    Serial.print(MOTOR_SIGN[0]);
    Serial.print(" ");
    Serial.print(MOTOR_SIGN[1]);
    Serial.print("  ENC_SIGN ");
    Serial.print(ENC_SIGN[0]);
    Serial.print(" ");
    Serial.println(ENC_SIGN[1]);
    Serial.println("Odometry check. z = zero, m = forward pulse, f/t = static friction");
    Serial.println("countL\tcountR\trho(in)\tphi(deg)");
}

void loop() {
    checkDriverFault();
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'z') {
            noInterrupts();
            count[0] = 0;
            count[1] = 0;
            interrupts();
            Serial.println("zeroed");
        } else if (c == 'm') {
            for (int m = 0; m < 2; m++) pulseMotor(m);
        } else if (c == 'f') {
            findStaticFriction(false);
        } else if (c == 't') {
            findStaticFriction(true);
        }
    }

    if (millis() - last_print >= PRINT_MS) {
        last_print = millis();
        long cL = readCount(0);
        long cR = readCount(1);
        float thetaL = 2 * PI * (float)cL / COUNTS_PER_REV;
        float thetaR = 2 * PI * (float)cR / COUNTS_PER_REV;
        float rho = WHEEL_RADIUS_M * (thetaL + thetaR) / 2;
        float phi = WHEEL_RADIUS_M * (thetaR - thetaL) / WHEEL_BASE_M;

        Serial.print(cL);
        Serial.print("\t");
        Serial.print(cR);
        Serial.print("\t");
        Serial.print(rho / 0.0254, 2);
        Serial.print("\t");
        Serial.print(phi * 180 / PI, 2);
        Serial.println("");
    }
}
