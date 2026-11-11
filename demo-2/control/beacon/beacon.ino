/*
 * EENG350 Demo 2 - Find the beacon, drive up to it, turn the way its arrow points.
 *
 * After START_DELAY_S the robot turns in place until the Pi reports ArUco
 * marker 0, turns to face it, drives until the center of rotation is
 * STOP_DISTANCE_FT from it, and (if TURN_AT_BEACON) turns 90 degrees left or
 * right, whichever way the Pi says the arrow beside the marker points.
 *
 * The motion controller is Demo 1's drive.ino: distance rho and heading phi
 * each follow a trapezoidal reference with an outer PI position loop around
 * an inner speed loop. What is new is where the targets come from. Demo 1
 * had two fixed targets. Here every camera reading is turned into a heading
 * target and a distance target in the encoders' own coordinates, so the
 * robot keeps correcting while it moves and carries on by odometry if the
 * marker drops out of view.
 *
 * I2C protocol (Arduino is the slave at address 0x08, see ../../README.md)
 *   The Pi writes one 8 byte message per camera frame. The Arduino never
 *   replies. All multi-byte values are little-endian.
 *     byte 0     MARKER_MSG_ID (0x01)
 *     byte 1     visible       1 if marker 0 is in this frame, else 0
 *     byte 2-3   angle_cdeg    int16, camera axis to marker, 0.01 deg, + = left
 *     byte 4-5   distance_mm   uint16, camera to marker along the floor, mm
 *     byte 6     arrow         0 unknown, 1 left, 2 right
 *     byte 7     checksum      low byte of the sum of bytes 1 to 6
 *   Bytes 1 to 6 are struct MarkerMsg below. To change the message, change
 *   that struct and MESSAGE_FORMAT in vision/demo2.py together.
 *
 * Hardware (same as Demo 1, plus the I2C link from the mini project)
 *   Pololu dual MC33926 motor driver shield: pin 4 enable, pins 7/8 motor
 *   1/2 direction, pins 9/10 motor 1/2 PWM, pin 12 status flag.
 *   LEFT wheel:  motor on M2 (pins 8/10), encoder A -> pin 2, B -> pin 6
 *   RIGHT wheel: motor on M1 (pins 7/9),  encoder A -> pin 3, B -> pin 5
 *   Encoder Vcc (blue) -> 5 V, encoder GND (green) -> GND
 *   Raspberry Pi GPIO2 (SDA, pin 3) -> Arduino A4 (SDA)
 *   Raspberry Pi GPIO3 (SCL, pin 5) -> Arduino A5 (SCL)
 *   Raspberry Pi GND (pin 6)        -> Arduino GND
 */

#include <Wire.h>

// ------------------------------ the run -------------------------------------
const bool  TURN_AT_BEACON   = true;     // true: trial type 2, turn 90 deg at the beacon. false: type 1, stop there
const float STOP_DISTANCE_FT = 1.0;      // stop with the center of rotation this far from the marker (must be < 1.5)
const float START_DELAY_S    = 1.0;      // motors off this long after reset
const int   SEARCH_DIRECTION = 1;        // search turning left (+1, CCW seen from above) or right (-1)
const int   DEFAULT_TURN     = 1;        // turn to make if the arrow was never read: +1 left, -1 right

// ------------------------------ camera --------------------------------------
// MEASURE: how far the camera lens is ahead of the line between the wheels
const float CAMERA_OFFSET_M = 0.10;
// How old a reading is when it gets here (exposure + processing + I2C). The
// reading is matched with where the robot was that long ago. 0 = don't.
const unsigned long CAMERA_LATENCY_MS = 80;

// ------------------------------ search, approach ----------------------------
const float SEARCH_PHI_DOT  = 0.5;       // rad/s. Too fast and the marker blurs
const float SEARCH_STEP_DEG = 0.0;       // 0: turn continuously. > 0: turn this far, pause to look, repeat
const float SEARCH_PAUSE_S  = 0.4;       // the pause, when stepping
const float MARKER_LOST_S   = 1.5;       // facing where the marker was and not seen for this long: search again
const float FACE_TOL_DEG    = 5.0;       // start driving once the marker is this close to straight ahead
const float FACE_RATE       = 0.15;      // ... and the turn has slowed to this [rad/s]
const float ARRIVE_TOL_M    = 0.02;      // arrived when this close to the stop point
const float ARRIVE_SPEED    = 0.02;      // ... and the reference has slowed to this [m/s]
const int   ARROW_MIN_VOTES = 3;         // readings one way, net of the other, before the arrow is believed
const int   ARROW_MAX_VOTES = 20;        // cap on the tally, so the latest (closest) readings decide
const float ARROW_WAIT_S    = 2.0;       // at the beacon with no arrow yet: wait this long, then DEFAULT_TURN

// ------------------------------ robot ---------------------------------------
// From Demo 1 (demo-1/control/README.md, "Calibration")
const float WHEEL_RADIUS_M = 0.0725;     // r
const float WHEEL_BASE_M   = 0.35;       // d, between the wheel contact points

const float COUNTS_PER_REV = 1600.0;
// One sign per wheel for each part, {left, right}:
//   MOTOR_SIGN: flip it if a positive voltage turns that wheel backward
//   ENC_SIGN:   flip it if that wheel's count goes down when the robot rolls forward
const int MOTOR_SIGN[2] = {-1, 1};
const int ENC_SIGN[2]   = {1, -1};

// ------------------------------ controller ----------------------------------
// From Demo 1's drive.ino. Same names in matlab/robot_params.m.
const float K_rho = 0.0825;              // (m/s) per V of Vbar
const float K_phi = 0.471;               // (rad/s) per V of dVa
const float V_STATIC = 0.5;              // V, static friction

const float Kp_rho_dot = 48.0;           // V per m/s
const float Kp_phi_dot = 8.5;            // V per rad/s
const float Kp_rho = 3.0;                // (m/s) per m
const float Ki_rho = 2.0;                // (m/s) per m*s
const float Kp_phi = 4.0;                // (rad/s) per rad
const float Ki_phi = 2.0;                // (rad/s) per rad*s
const float RHO_DOT_LIMIT = 0.15;        // limit on rho_dot_d [m/s]
const float PHI_DOT_LIMIT = 0.75;        // limit on phi_dot_d [rad/s]

// Reference profile. These set how long a run takes: time is scored, so
// raise them (and the two limits above, and MAX_VOLTAGE) once it works.
const float MAX_RHO_DOT = 0.1;           // m/s
const float RHO_ACCEL   = 0.1;           // m/s^2
const float MAX_PHI_DOT = 0.5;           // rad/s
const float PHI_ACCEL   = 1.0;           // rad/s^2

// Hard cap on each wheel's voltage, so a sign or calibration mistake can't
// send the robot off at full speed.
const float MAX_VOLTAGE = 3.0;           // V

// Stopping
const float RHO_TOL = 0.003;             // m (about 0.1 in)
const float PHI_TOL = 0.003;             // rad (about 0.17 deg)
const float SETTLE_HOLD_S    = 0.3;
const float SETTLE_TIMEOUT_S = 3.0;

const float Battery_Voltage = 7.0;
const unsigned long desired_Ts_ms = 10;
const unsigned int PRINT_EVERY = 2;      // print a data line every this many samples
// ----------------------------------------------------------------------------

// ------------------------------ message from the Pi -------------------------
const uint8_t I2C_ADDRESS   = 0x08;
const uint8_t MARKER_MSG_ID = 0x01;
const uint8_t ARROW_UNKNOWN = 0, ARROW_LEFT = 1, ARROW_RIGHT = 2;

// Bytes 1 to 6 of the message, in order. The Uno is little-endian and the
// struct is packed, so the received bytes can be copied straight into it.
struct __attribute__((packed)) MarkerMsg {
    uint8_t  visible;       // 1 if marker 0 is in this frame
    int16_t  angle_cdeg;    // camera axis to marker [0.01 deg], positive = left
    uint16_t distance_mm;   // camera to marker along the floor [mm]
    uint8_t  arrow;         // ARROW_UNKNOWN, ARROW_LEFT or ARROW_RIGHT
};
const uint8_t MSG_LENGTH = sizeof(MarkerMsg) + 2;   // + ID byte + checksum

// (The other two types are here as well because the Arduino build needs
// every type defined above the first function.)
struct Profile {
    float pos;   // reference position
    float vel;   // reference speed (feedforward)
};
enum Stage { WAIT, SEARCH, FACE, APPROACH, TURN, DONE };   // printed as 0 to 5

volatile uint8_t rx_payload[sizeof(MarkerMsg)];     // newest valid message
volatile bool rx_new = false;                       // true until loop() has read it

// Called (inside an interrupt) when the Pi writes. Keeps the message only
// if it has the right length, ID and checksum.
void receiveEvent(int howMany) {
    uint8_t buf[MSG_LENGTH];
    uint8_t n = 0;
    while (Wire.available()) {             // read everything so the buffer is empty
        uint8_t c = Wire.read();
        if (n < MSG_LENGTH) buf[n] = c;
        if (n < 255) n++;
    }
    if (n != MSG_LENGTH || buf[0] != MARKER_MSG_ID) return;
    uint8_t sum = 0;
    for (uint8_t i = 1; i < MSG_LENGTH - 1; i++) sum += buf[i];
    if (sum != buf[MSG_LENGTH - 1]) return;
    for (uint8_t i = 0; i < sizeof(MarkerMsg); i++) rx_payload[i] = buf[i + 1];
    rx_new = true;
}

// Copies the newest message into m. Returns false if nothing new has
// arrived since the last call.
bool readMarker(MarkerMsg &m) {
    noInterrupts();
    bool fresh = rx_new;
    rx_new = false;
    if (fresh) memcpy(&m, (const void *)rx_payload, sizeof(MarkerMsg));
    interrupts();
    return fresh;
}

// ------------------------------ pins, encoders, motors ----------------------
const float STOP_DISTANCE_M = STOP_DISTANCE_FT * 0.3048;
const float SEARCH_STEP_RAD = SEARCH_STEP_DEG * PI / 180.0;
const float FACE_TOL_RAD    = FACE_TOL_DEG * PI / 180.0;

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
// Unlike Demo 1 the target can move while the profile is under way. If it
// jumps so close that the profile can't stop in time, the profile brakes at
// amax, runs past, and comes back, the way the robot itself has to.
void stepProfile(Profile &p, float target, float vmax, float amax, float dt) {
    float remaining = target - p.pos;
    float dir = (remaining >= 0) ? 1.0 : -1.0;
    // fastest speed from which it can still stop at the target
    float v_want = dir * min(vmax, sqrt(2 * amax * fabs(remaining)));
    p.vel += constrain(v_want - p.vel, -amax * dt, amax * dt);
    p.pos += p.vel * dt;
    // stop exactly on the target once it gets there slowly enough
    if ((target - p.pos) * dir <= 0 && fabs(p.vel) <= 4 * amax * dt) {
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
unsigned int pass_count = 0;    // samples since the last printed line

// Odometry for the last HISTORY_LEN samples, so a camera reading can be
// matched with where the robot was when the picture was taken.
const int HISTORY_LEN = 32;
const int LATENCY_SAMPLES = min((int)(CAMERA_LATENCY_MS / desired_Ts_ms), HISTORY_LEN - 1);
float rho_history[HISTORY_LEN], phi_history[HISTORY_LEN];
int history_head = 0;           // index of the newest sample

// The marker, in the odometry's coordinates
float marker_phi = 0;           // heading that points the robot at the marker
float marker_rho = 0;           // rho at which the robot is STOP_DISTANCE from it
float last_seen_time = -1;      // time of the newest sighting, -1 if never
float cam_angle_deg = 0;        // newest raw reading, for the printout
float cam_distance_m = 0;
int arrow_votes = 0;            // > 0: arrow points left, < 0: right
float arrived_time = -1;        // time the robot reached the stop point, -1 if not yet
float search_pause_since = -1;  // time the current search pause began, -1 if turning

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
    Serial.print(F("# driver fault "));
    Serial.print(fault_count);
    Serial.print(F(" at t = "));
    Serial.print(millis() / 1000.0, 2);
    if (fault_count >= MAX_FAULTS) {
        Serial.println(F(" s, too many: motors left off, check the battery"));
        return;
    }
    delayMicroseconds(10);
    digitalWrite(EN_PIN, HIGH);
    Serial.println(F(" s, cleared"));
}

unsigned long next_ms;
unsigned long start_us, last_us;

void startStage(Stage s) {
    stage = s;
    profile_done_time = -1;
    band_since = -1;
    search_pause_since = -1;
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

    // I2C slave with the internal 5 V pull-ups turned off (the Pi has its own)
    Wire.begin(I2C_ADDRESS);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.onReceive(receiveEvent);

    for (int i = 0; i < HISTORY_LEN; i++) {
        rho_history[i] = 0;
        phi_history[i] = 0;
    }

    // What this upload was built with, to check against the file
    Serial.print(F("MOTOR_SIGN "));
    Serial.print(MOTOR_SIGN[0]);
    Serial.print(F(" "));
    Serial.print(MOTOR_SIGN[1]);
    Serial.print(F("  ENC_SIGN "));
    Serial.print(ENC_SIGN[0]);
    Serial.print(F(" "));
    Serial.print(ENC_SIGN[1]);
    Serial.println(TURN_AT_BEACON ? F("  type 2: turn at the beacon") : F("  type 1: stop at the beacon"));
    Serial.println(F("# time stage rho_ref rho phi_ref phi VL VR cam_angle_deg cam_dist_m seen_age_s arrow_votes"));
    Serial.println(F("Ready!"));
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

    history_head = (history_head + 1) % HISTORY_LEN;
    rho_history[history_head] = rho;
    phi_history[history_head] = phi;

    // ---- Camera ----
    // A sighting is a new message that says the marker is visible. It is
    // turned into targets in odometry coordinates right away, so nothing
    // below ever uses a stale camera angle.
    MarkerMsg msg;
    bool seen = readMarker(msg) && msg.visible;
    if (seen) {
        cam_angle_deg  = msg.angle_cdeg / 100.0;
        cam_distance_m = msg.distance_mm / 1000.0;
        float angle = cam_angle_deg * PI / 180.0;
        // Marker position seen from the center of rotation: x ahead, y left.
        // The camera sits CAMERA_OFFSET_M ahead of the center.
        float mx = CAMERA_OFFSET_M + cam_distance_m * cos(angle);
        float my = cam_distance_m * sin(angle);
        // Where the robot was when the picture was taken
        int then = (history_head + HISTORY_LEN - LATENCY_SAMPLES) % HISTORY_LEN;
        marker_phi = phi_history[then] + atan2(my, mx);
        marker_rho = rho_history[then] + sqrt(mx * mx + my * my) - STOP_DISTANCE_M;
        last_seen_time = current_time;

        if (msg.arrow == ARROW_LEFT)  arrow_votes = min(arrow_votes + 1,  ARROW_MAX_VOTES);
        if (msg.arrow == ARROW_RIGHT) arrow_votes = max(arrow_votes - 1, -ARROW_MAX_VOTES);
    }

    // ---- Stages: choose the targets ----
    if (stage == WAIT) {
        if (current_time >= START_DELAY_S) startStage(SEARCH);
    }

    if (stage == SEARCH) {
        if (seen) {
            startStage(FACE);
        } else if (SEARCH_STEP_RAD <= 0) {
            // Continuous: keep the target half a turn ahead, so the profile cruises
            phi_target = phi_prof.pos + SEARCH_DIRECTION * PI;
        } else if (profileDone(phi_prof, phi_target)) {
            // Stepping: look for SEARCH_PAUSE_S (first before moving at all), then step on
            if (search_pause_since < 0) {
                search_pause_since = current_time;
            } else if (current_time - search_pause_since >= SEARCH_PAUSE_S) {
                phi_target += SEARCH_DIRECTION * SEARCH_STEP_RAD;
                search_pause_since = -1;
            }
        }
    }

    if (stage == FACE) {
        if (seen) phi_target = marker_phi;
        if (fabs(phi_target - phi) < FACE_TOL_RAD && fabs(phi_prof.vel) < FACE_RATE) {
            startStage(APPROACH);
            rho_target = marker_rho;
        } else if (profileDone(phi_prof, phi_target) && current_time - last_seen_time > MARKER_LOST_S) {
            startStage(SEARCH);    // looking where it was, and it isn't there
        }
    }

    bool stop_now = false;
    if (stage == APPROACH) {
        if (arrived_time < 0) {
            // Still driving: every sighting refines both targets. With no
            // sightings the robot finishes on the last ones by odometry.
            if (seen) {
                phi_target = marker_phi;
                rho_target = marker_rho;
            }
            if (fabs(rho_target - rho) < ARRIVE_TOL_M && fabs(rho_prof.vel) < ARRIVE_SPEED) {
                arrived_time = current_time;
            }
        }
        if (arrived_time >= 0) {
            bool arrow_known = abs(arrow_votes) >= ARROW_MIN_VOTES;
            if (!TURN_AT_BEACON) {
                stop_now = true;
            } else if (arrow_known || current_time - arrived_time >= ARROW_WAIT_S) {
                int turn = DEFAULT_TURN;
                if (arrow_known) turn = (arrow_votes > 0) ? 1 : -1;
                phi_target += turn * PI / 2;    // 90 deg from the heading that faces the marker
                startStage(TURN);
            }
        }
    }

    // ---- Controller (Demo 1) ----
    float VL = 0, VR = 0;

    if (stage != WAIT && stage != DONE && !stop_now) {
        // 1. Reference
        float phi_speed = (stage == SEARCH) ? SEARCH_PHI_DOT : MAX_PHI_DOT;
        stepProfile(rho_prof, rho_target, MAX_RHO_DOT, RHO_ACCEL, dt);
        stepProfile(phi_prof, phi_target, phi_speed, PHI_ACCEL, dt);
        bool rho_done = profileDone(rho_prof, rho_target);
        bool phi_done = profileDone(phi_prof, phi_target);
        bool profiles_done = rho_done && phi_done;
        if (profiles_done && profile_done_time < 0) profile_done_time = current_time;
        if (!profiles_done) profile_done_time = -1;

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

        // The last turn ends once settled, or after the timeout if friction holds it off target
        if (stage == TURN) {
            bool settled   = in_band && current_time - band_since >= SETTLE_HOLD_S;
            bool timed_out = profile_done_time >= 0 && current_time - profile_done_time >= SETTLE_TIMEOUT_S;
            if (settled || timed_out) stop_now = true;
        }
    }

    if (stop_now) {
        startStage(DONE);
        VL = 0;
        VR = 0;
    }

    applyVoltage(0, VL);
    applyVoltage(1, VR);

    if (stage == DONE) {
        if (!finished) {
            Serial.println(F("Finished"));
            finished = true;
        }
        return;
    }

    if (++pass_count < PRINT_EVERY) return;
    pass_count = 0;
    Serial.print(current_time, 3);
    Serial.print("\t");
    Serial.print(stage);
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
    Serial.print("\t");
    Serial.print(cam_angle_deg, 2);
    Serial.print("\t");
    Serial.print(cam_distance_m, 3);
    Serial.print("\t");
    Serial.print(last_seen_time < 0 ? -1.0 : current_time - last_seen_time, 2);
    Serial.print("\t");
    Serial.print(arrow_votes);
    Serial.println("");
}
