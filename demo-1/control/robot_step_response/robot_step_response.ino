/*
 * EENG350 Demo 1 - Open loop step response of the whole robot.
 *
 * Identifies the two transfer functions of the SteeringControl handout,
 *
 *     G_rho(s) = rho_dot(s)/Vbar(s) = K_rho*sigma_rho/(s + sigma_rho)
 *     G_phi(s) = phi_dot(s)/dVa(s)  = K_phi*sigma_phi/(s + sigma_phi)
 *
 * where Vbar = VR + VL drives the robot forward and dVa = VR - VL turns it
 * left. Run it with the robot on the floor, once as is (forward step) and
 * once with TEST_ROTATION defined (turn in place):
 *
 *   forward:  Vbar = STEP_VBAR, dVa = 0 -> VL = VR = STEP_VBAR/2
 *   rotation: Vbar = 0, dVa = STEP_DVA -> VR = STEP_DVA/2, VL = -STEP_DVA/2
 *
 * applied between 1 and 3 s, 0 V otherwise. Give it 1 to 2 ft of room
 * forward.
 *
 * Output: five tab-separated columns every 10 ms for the first 3 s:
 *
 *     time (s)   Vbar (V)   dVa (V)   rho_dot (m/s)   phi_dot (rad/s)
 *
 * after "Ready!" and ending with "Finished". Record with
 *     readArduinoData("rho_step_data.mat", 5)    forward
 *     readArduinoData("phi_step_data.mat", 5)    with TEST_ROTATION
 * and fit K and sigma with control/matlab/run_identify.m. The output that
 * isn't being stepped (phi_dot in the forward test, rho_dot in the
 * rotation test) shows how well the two are decoupled.
 *
 * Pins, encoders, MOTOR_SIGN, ENC_SIGN and geometry are the same as
 * drive/drive.ino; keep them in sync.
 */

// Uncomment for the rotation step instead of the forward step
// #define TEST_ROTATION

// ------------------------------ settings ------------------------------------
const float STEP_VBAR = 2.5;             // V, sum of the wheel voltages (forward test)
const float STEP_DVA  = 2.5;             // V, difference (rotation test)

const float WHEEL_RADIUS_M = 0.0725;      // same as drive.ino
const float WHEEL_BASE_M   = 0.36;

const float COUNTS_PER_REV = 1600.0;
// One sign per wheel for each part, {left, right}:
//   MOTOR_SIGN: flip it if a positive voltage turns that wheel backward
//   ENC_SIGN:   flip it if that wheel's count goes down when the robot rolls forward
const int MOTOR_SIGN[2] = {-1, 1};
const int ENC_SIGN[2]   = {1, -1};

const float Battery_Voltage = 7.0;
const unsigned long desired_Ts_ms = 10;
// ----------------------------------------------------------------------------

#ifdef TEST_ROTATION
const float Vbar_step = 0;
const float dVa_step  = STEP_DVA;
#else
const float Vbar_step = STEP_VBAR;
const float dVa_step  = 0;
#endif

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

// Wheel angle [rad], positive when the wheel drives the robot forward
float wheelAngle(int i) {
    return 2 * PI * (float)readCount(i) / COUNTS_PER_REV;
}

void applyVoltage(int i, float Voltage) {
    digitalWrite(DIR_PIN[i], (MOTOR_SIGN[i] * Voltage > 0) ? HIGH : LOW);
    float PWM = 255.0 * fabs(Voltage) / Battery_Voltage;
    analogWrite(PWM_PIN[i], min(PWM, 255.0));
}

float prev_rho = 0, prev_phi = 0;
bool finished = false;
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

unsigned long next_ms;
unsigned long start_us, last_us;

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

    Serial.println("Ready!");
    start_us = micros();
    last_us  = start_us;
    next_ms  = millis() + desired_Ts_ms;
}

void loop() {
    while (millis() < next_ms) { }
    next_ms += desired_Ts_ms;

    unsigned long now_us = micros();
    float dt = (now_us - last_us) * 1e-6;
    float current_time = (now_us - start_us) * 1e-6;
    last_us = now_us;
    checkDriverFault();

    // rho_dot and phi_dot from the change in rho and phi (handout section 3)
    float thetaL = wheelAngle(0);
    float thetaR = wheelAngle(1);
    float rho = WHEEL_RADIUS_M * (thetaL + thetaR) / 2;
    float phi = WHEEL_RADIUS_M * (thetaR - thetaL) / WHEEL_BASE_M;
    float rho_dot = (rho - prev_rho) / dt;
    float phi_dot = (phi - prev_phi) / dt;
    prev_rho = rho;
    prev_phi = phi;

    // Step between 1 and 3 s
    float Vbar = 0, dVa = 0;
    if (current_time >= 1 && current_time < 3) {
        Vbar = Vbar_step;
        dVa  = dVa_step;
    }
    applyVoltage(0, (Vbar - dVa) / 2);   // left
    applyVoltage(1, (Vbar + dVa) / 2);   // right

    if (current_time < 3) {
        Serial.print(current_time, 3);
        Serial.print("\t");
        Serial.print(Vbar, 2);
        Serial.print("\t");
        Serial.print(dVa, 2);
        Serial.print("\t");
        Serial.print(rho_dot, 4);
        Serial.print("\t");
        Serial.print(phi_dot, 4);
        Serial.println("");
    } else if (!finished) {
        Serial.println("Finished");
        finished = true;
    }
}
