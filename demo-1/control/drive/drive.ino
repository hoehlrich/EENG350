/*
 * EENG350 Demo 1 - Turn by an angle, then drive straight a distance.
 *
 * Set TARGET_ANGLE_DEG and TARGET_DISTANCE_FT below and upload. After
 * START_DELAY_S the robot turns in place by the angle (positive = left,
 * counterclockwise seen from above), waits until it has settled, then
 * drives forward the distance while holding that heading, and stops.
 *
 * Hardware (same as the mini project)
 *   Pololu dual MC33926 motor driver shield: pin 4 enable, pins 7/8 motor
 *   1/2 direction, pins 9/10 motor 1/2 PWM.
 *   LEFT wheel:  motor on M2 (pins 8/10), encoder A -> pin 2, B -> pin 6
 *   RIGHT wheel: motor on M1 (pins 7/9),  encoder A -> pin 3, B -> pin 5
 *   Encoder Vcc (blue) -> 5 V, encoder GND (green) -> GND
 *   The counts go up when that wheel rolls forward (ENC_SIGN), and a
 *   positive voltage turns that wheel forward (MOTOR_SIGN).
 */

// ------------------------------ the run -------------------------------------
const float TARGET_ANGLE_DEG   = -180.0;  // turn first, positive = left (CCW)
const float TARGET_DISTANCE_FT = 2.0;    // then drive this far forward
const float START_DELAY_S      = 1.0;    // must start within 10 s of download

// ------------------------------ robot ---------------------------------------
// MEASURE both with tests/odometry_check, then trim from driven runs
// (control/README.md, "Calibration"). Same values in robot_params.m.
const float WHEEL_RADIUS_M = 0.0725;      // r
const float WHEEL_BASE_M   = 0.35;       // d, between the wheel contact points

const float COUNTS_PER_REV = 1600.0;
// One sign per wheel for each part, {left, right}:
//   MOTOR_SIGN: flip it if a positive voltage turns that wheel backward
//   ENC_SIGN:   flip it if that wheel's count goes down when the robot rolls forward
const int MOTOR_SIGN[2] = {-1, 1};
const int ENC_SIGN[2]   = {1, -1};

// ------------------------------ controller ----------------------------------
const float K_rho = 0.0825;              // (m/s) per V of Vbar
const float K_phi = 0.471;               // (rad/s) per V of dVa
const float V_STATIC = 0.5;              // V, static friction (odometry_check f/t commands)

const float Kp_rho_dot = 48.0;           // V per m/s
const float Kp_phi_dot = 8.5;            // V per rad/s
const float Kp_rho = 3.0;                // (m/s) per m
const float Ki_rho = 2.0;                // (m/s) per m*s
const float Kp_phi = 4.0;                // (rad/s) per rad
const float Ki_phi = 2.0;                // (rad/s) per rad*s
const float RHO_DOT_LIMIT = 0.15;        // limit on rho_dot_d [m/s]
const float PHI_DOT_LIMIT = 0.75;        // limit on phi_dot_d [rad/s]

// Reference profile, slow for testing: about 4 in/s, 10 ft in about 32 s
const float MAX_RHO_DOT = 0.1;           // m/s
const float RHO_ACCEL   = 0.1;           // m/s^2
const float MAX_PHI_DOT = 0.5;           // rad/s
const float PHI_ACCEL   = 1.0;           // rad/s^2

// Hard cap on each wheel's voltage, so a sign or calibration mistake can't
// send the robot off at full speed. Slow runs need under 2 V.
const float MAX_VOLTAGE = 3.0;           // V

// Stopping
const float RHO_TOL = 0.003;             // m (about 0.1 in)
const float PHI_TOL = 0.003;             // rad (about 0.17 deg)
const float SETTLE_HOLD_S    = 0.3;
const float SETTLE_TIMEOUT_S = 3.0;

const float Battery_Voltage = 7.0;
const unsigned long desired_Ts_ms = 10;
// ----------------------------------------------------------------------------

struct Profile {
    float pos;   // reference position
    float vel;   // reference speed (feedforward)
};
enum Stage { WAIT, TURN, DRIVE, DONE };

const float TARGET_ANGLE_RAD  = TARGET_ANGLE_DEG * PI / 180.0;
const float TARGET_DISTANCE_M = TARGET_DISTANCE_FT * 0.3048;

const int EN_PIN       = 4;
const int DIR_PIN[2]   = {8, 7};    // left motor is on M2, right on M1
const int PWM_PIN[2]   = {10, 9};
const int ENC_A_PIN[2] = {2, 3};
const int ENC_B_PIN[2] = {6, 5};

volatile long count[2] = {0, 0};       // encoder counts, updated in the interrupts

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

// Positive voltage turns the wheel forward. The sign goes to the DIR pin
// (through MOTOR_SIGN), the size to the PWM duty.
void applyVoltage(int i, float Voltage) {
    digitalWrite(DIR_PIN[i], (MOTOR_SIGN[i] * Voltage > 0) ? HIGH : LOW);
    float PWM = 255.0 * fabs(Voltage) / Battery_Voltage;   // 255 = full battery voltage
    analogWrite(PWM_PIN[i], min(PWM, 255.0));
}

// Trapezoidal reference: pos moves toward target with speed at most vmax
// and acceleration at most amax, slowing down in time to stop on it.
void stepProfile(Profile &p, float target, float vmax, float amax, float dt) {
    float remaining = target - p.pos;
    float dir = (remaining >= 0) ? 1.0 : -1.0;
    // fastest speed from which it can still stop at the target
    float v_want = dir * min(vmax, sqrt(2 * amax * fabs(remaining)));
    p.vel += constrain(v_want - p.vel, -amax * dt, amax * dt);
    p.pos += p.vel * dt;
    // stop exactly on the target once it gets there
    if ((target - p.pos) * dir <= 0) {
        p.pos = target;
        p.vel = 0;
    }
}

bool profileDone(const Profile &p, float target) {
    return p.pos == target && p.vel == 0;
}

// ------------------------------ state ---------------------------------------
Stage stage = WAIT;

Profile rho_prof = {0, 0}, phi_prof = {0, 0};
float rho_target = 0, phi_target = 0;   // where the profiles are heading now
float integral_rho = 0, integral_phi = 0;
float prev_rho = 0, prev_phi = 0;

float profile_done_time = -1;   // time the stage's profile finished, -1 if not yet
float band_since = -1;          // time the robot entered the deadband, -1 if outside
bool finished = false;          // true once "Finished" has been printed

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

void startStage(Stage s) {
    stage = s;
    profile_done_time = -1;
    band_since = -1;
    if (s == TURN)  phi_target = TARGET_ANGLE_RAD;
    if (s == DRIVE) rho_target = TARGET_DISTANCE_M;
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
    Serial.println("Ready!");
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
    checkDriverFault();

    // Odometry
    float thetaL = wheelAngle(0);
    float thetaR = wheelAngle(1);
    float rho = WHEEL_RADIUS_M * (thetaL + thetaR) / 2;
    float phi = WHEEL_RADIUS_M * (thetaR - thetaL) / WHEEL_BASE_M;
    float rho_dot = (rho - prev_rho) / dt;
    float phi_dot = (phi - prev_phi) / dt;
    prev_rho = rho;
    prev_phi = phi;

    float VL = 0, VR = 0;

    if (stage == WAIT) {
        if (current_time >= START_DELAY_S) startStage(TURN);
    }

    if (stage == TURN || stage == DRIVE) {
        // 1. Reference
        stepProfile(rho_prof, rho_target, MAX_RHO_DOT, RHO_ACCEL, dt);
        stepProfile(phi_prof, phi_target, MAX_PHI_DOT, PHI_ACCEL, dt);
        bool rho_done = profileDone(rho_prof, rho_target);
        bool phi_done = profileDone(phi_prof, phi_target);
        bool profiles_done = rho_done && phi_done;
        if (profiles_done && profile_done_time < 0) profile_done_time = current_time;

        // 2. Outer loops: PI on position plus the reference speed
        float e_rho = rho_prof.pos - rho;
        float e_phi = phi_prof.pos - phi;
        if (rho_done) integral_rho += e_rho * dt;
        if (phi_done) integral_phi += e_phi * dt;
        float rho_dot_d = rho_prof.vel + Kp_rho * e_rho + Ki_rho * integral_rho;
        float phi_dot_d = phi_prof.vel + Kp_phi * e_phi + Ki_phi * integral_phi;

        bool freeze_rho = false, freeze_phi = false;
        if (fabs(rho_dot_d) > RHO_DOT_LIMIT) {
            rho_dot_d = constrain(rho_dot_d, -RHO_DOT_LIMIT, RHO_DOT_LIMIT);
            freeze_rho = true;
        }
        if (fabs(phi_dot_d) > PHI_DOT_LIMIT) {
            phi_dot_d = constrain(phi_dot_d, -PHI_DOT_LIMIT, PHI_DOT_LIMIT);
            freeze_phi = true;
        }

        // 3. Inner loops: model feedforward plus proportional on speed
        float Vbar = rho_dot_d / K_rho + Kp_rho_dot * (rho_dot_d - rho_dot);
        float dVa  = phi_dot_d / K_phi + Kp_phi_dot * (phi_dot_d - phi_dot);

        // 4. Wheel voltages plus friction, scaled together if either is too big
        VR = (Vbar + dVa) / 2;
        VL = (Vbar - dVa) / 2;
        if (VR != 0) VR += (VR > 0) ? V_STATIC : -V_STATIC;
        if (VL != 0) VL += (VL > 0) ? V_STATIC : -V_STATIC;
        float biggest = max(fabs(VL), fabs(VR));
        if (biggest > MAX_VOLTAGE) {
            VL *= MAX_VOLTAGE / biggest;
            VR *= MAX_VOLTAGE / biggest;
            freeze_rho = true;
            freeze_phi = true;
        }

        // Deadband: on target, so stop pushing and stop integrating
        bool in_band = profiles_done && fabs(e_rho) < RHO_TOL && fabs(e_phi) < PHI_TOL;
        if (in_band) {
            VL = 0;
            VR = 0;
            freeze_rho = true;
            freeze_phi = true;
            if (band_since < 0) band_since = current_time;
        } else {
            band_since = -1;
        }

        // Anti-windup: undo this pass's integration
        if (freeze_rho && rho_done) integral_rho -= e_rho * dt;
        if (freeze_phi && phi_done) integral_phi -= e_phi * dt;

        // Next stage once settled, or after the timeout if friction holds it off target
        bool settled   = in_band && current_time - band_since >= SETTLE_HOLD_S;
        bool timed_out = profile_done_time >= 0 && current_time - profile_done_time >= SETTLE_TIMEOUT_S;
        if (settled || timed_out) {
            if (stage == TURN) {
                startStage(DRIVE);
            } else {
                startStage(DONE);
                VL = 0;
                VR = 0;
            }
        }
    }

    applyVoltage(0, VL);
    applyVoltage(1, VR);

    if (stage == DONE) {
        if (!finished) {
            Serial.println("Finished");
            finished = true;
        }
        return;
    }

    Serial.print(current_time, 3);
    Serial.print("\t");
    Serial.print(rho_prof.pos, 4);
    Serial.print("\t");
    Serial.print(rho, 4);
    Serial.print("\t");
    Serial.print(phi_prof.pos, 4);
    Serial.print("\t");
    Serial.print(phi, 4);
    Serial.print("\t");
    Serial.print(VL, 2);
    Serial.print("\t");
    Serial.print(VR, 2);
    Serial.println("");
}
