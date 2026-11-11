/*
 * EENG350 Demo 2 - Runs beacon.ino on a PC against a simulated robot,
 * camera and Pi. Built and run by run_sim.py, which is the thing to use.
 *
 * The sketch is compiled unchanged (it is #included below) on top of the
 * stand-in Arduino.h. Its pins are connected to this file's model:
 *
 *   motors    direction + PWM pins -> voltage -> first order wheel speed
 *             (K*sigma/(s + sigma) with a friction deadzone), per wheel
 *   encoders  the A/B pins toggle and the sketch's interrupt handlers are
 *             called as each simulated wheel turns
 *   robot     x, y, heading integrated from the two wheel speeds
 *   camera    every frame: angle and distance from the camera to the
 *             beacon, with a field of view, noise, and a rotation speed
 *             above which the picture is too blurred to detect anything
 *   Pi        packs each frame into the I2C message and delivers it to the
 *             sketch's receive handler `latency` seconds later
 *
 * The wiring and polarities here are the real robot's, written down
 * independently of the sketch, so a wrong pin or sign in the sketch shows
 * up as a robot that drives the wrong way. The message is likewise packed
 * from the byte table in demo-2/README.md, not from the sketch's struct.
 *
 * Simulated time only moves when the sketch calls millis(), 50 us per
 * call, so the sketch's own "wait for the next sample" loop is what runs
 * the physics.
 *
 * Usage: beacon_sim [name=value ...]     (names: the Settings fields below)
 * Prints one line of name=value results.
 */

#include <deque>
#include <random>
#include <string>

#include "Arduino.h"

namespace sim {

// ------------------------------ settings ------------------------------------
// All settable from the command line as name=value.
struct Settings {
    // The trial
    double heading_deg = 90;      // where the beacon starts, from straight ahead, + = left
    double distance_ft = 5;       // center of rotation to beacon at the start
    double arrow = 1;             // 1 = arrow points left, 2 = right
    double max_time = 120;        // give up after this long [s]
    double seed = 1;              // random noise seed

    // The robot. Defaults are what the sketch believes; change them to see
    // how the run copes with a robot that differs from its calibration.
    double wheel_radius = 0.0725; // m
    double wheel_base = 0.35;     // m
    double motor_gain = 2.276;    // wheel rad/s per V (gives K_rho 0.0825, K_phi 0.471)
    double motor_sigma = 5.0;     // 1/s
    double friction_v = 0.5;      // V lost to friction
    double battery_v = 7.0;       // V
    double camera_offset = 0.10;  // m, lens ahead of the wheel axis

    // The camera and Pi
    double fov_deg = 60;          // horizontal field of view
    double max_range = 3.0;       // m, marker too small to detect beyond this
    double frame_period = 0.033;  // s
    double latency = 0.08;        // s from picture to message arriving
    double blur_limit = 1e9;      // rad/s, no detection while turning faster
    double detect_miss = 0;       // chance the marker is missed in a frame where it is in view
    double camera_start = 0;      // s, the Pi sends nothing before this
    double frames_limit = 1e9;    // the camera goes blind after this many sightings (5 = what
                                  // one reading typed into vision/send_marker.py sends)
    double angle_noise_deg = 0.15;
    double distance_noise = 0.01; // fraction of the distance, per meter of distance
    double arrow_range = 2.5;     // m, arrow can't be read beyond this
    double arrow_miss = 0.2;      // chance a frame reports the arrow as unknown
    double arrow_wrong = 0.02;    // chance a frame reports the wrong arrow
};
Settings set;
std::string trace_path, serial_path;

struct Name { const char *name; double *value; };
const Name NAMES[] = {
    {"heading_deg", &set.heading_deg}, {"distance_ft", &set.distance_ft},
    {"arrow", &set.arrow}, {"max_time", &set.max_time}, {"seed", &set.seed},
    {"wheel_radius", &set.wheel_radius}, {"wheel_base", &set.wheel_base},
    {"motor_gain", &set.motor_gain}, {"motor_sigma", &set.motor_sigma},
    {"friction_v", &set.friction_v}, {"battery_v", &set.battery_v},
    {"camera_offset", &set.camera_offset}, {"fov_deg", &set.fov_deg},
    {"max_range", &set.max_range}, {"frame_period", &set.frame_period},
    {"latency", &set.latency}, {"blur_limit", &set.blur_limit},
    {"detect_miss", &set.detect_miss}, {"frames_limit", &set.frames_limit},
    {"camera_start", &set.camera_start},
    {"angle_noise_deg", &set.angle_noise_deg}, {"distance_noise", &set.distance_noise},
    {"arrow_range", &set.arrow_range}, {"arrow_miss", &set.arrow_miss},
    {"arrow_wrong", &set.arrow_wrong},
};

// ------------------------------ the robot's wiring --------------------------
// {left, right}, as built (demo-1/control/README.md, "Hardware")
const int HW_EN_PIN = 4;
const int HW_NSF_PIN = 12;
const int HW_DIR_PIN[2] = {8, 7};
const int HW_PWM_PIN[2] = {10, 9};
const int HW_ENC_A_PIN[2] = {2, 3};
const int HW_ENC_B_PIN[2] = {6, 5};
const int HW_FORWARD_LEVEL[2] = {LOW, HIGH};   // DIR pin level that drives the wheel forward
const int HW_ENC_DIR[2] = {1, -1};             // +1: A leads B when the wheel rolls forward
const double COUNTS_PER_RAD = 1600.0 / (2 * PI);

// ------------------------------ state ---------------------------------------
const unsigned long STEP_US = 50;
unsigned long now_us = 0;

const int PINS = 20;
int pin_level[PINS];
int pwm_duty[PINS];
void (*pin_isr[PINS])();

double wheel_speed[2] = {0, 0};   // rad/s, + = forward
double wheel_angle[2] = {0, 0};   // rad
long encoder_count[2] = {0, 0};   // edges so far, + = forward
double x = 0, y = 0, heading = 0; // center of rotation [m, m, rad]
double beacon_x = 0, beacon_y = 0;

struct Message { double arrive_time; uint8_t bytes[8]; };
std::deque<Message> in_flight;
double next_frame_time = 0;
long sightings = 0;               // frames so far with the marker in them
std::mt19937 rng;

uint8_t wire_buffer[32];
int wire_length = 0, wire_position = 0;
void (*wire_callback)(int) = nullptr;

std::string serial_line;
bool finished = false;            // the sketch printed "Finished"
FILE *serial_file = nullptr, *trace_file = nullptr;

// Results
double first_move_time = -1;
double closest = 1e9;             // smallest center-to-beacon distance so far

double beaconDistance() { return hypot(beacon_x - x, beacon_y - y); }

// Angle from the robot's heading to the beacon [deg], + = beacon on the left
double beaconBearingDeg() {
    double a = atan2(beacon_y - y, beacon_x - x) - heading;
    return atan2(sin(a), cos(a)) * 180 / PI;
}

// ------------------------------ camera and Pi -------------------------------
// One camera frame: what demo2.py would measure, packed as it would send it.
void takeFrame(double time) {
    double camera_x = x + set.camera_offset * cos(heading);
    double camera_y = y + set.camera_offset * sin(heading);
    double dx = beacon_x - camera_x, dy = beacon_y - camera_y;
    double ahead = dx * cos(heading) + dy * sin(heading);
    double left = -dx * sin(heading) + dy * cos(heading);
    double angle_deg = atan2(left, ahead) * 180 / PI;
    double distance = hypot(ahead, left);
    double turn_rate = set.wheel_radius * (wheel_speed[1] - wheel_speed[0]) / set.wheel_base;

    bool visible = fabs(angle_deg) < set.fov_deg / 2 && distance < set.max_range
                   && distance > 0.12 && fabs(turn_rate) < set.blur_limit;

    std::normal_distribution<double> noise(0, 1);
    std::uniform_real_distribution<double> chance(0, 1);
    if (chance(rng) < set.detect_miss || sightings >= set.frames_limit) visible = false;
    if (visible) sightings++;

    Message m;
    m.arrive_time = time + set.latency;
    memset(m.bytes, 0, sizeof(m.bytes));
    m.bytes[0] = 0x01;                                  // message ID
    if (visible) {
        angle_deg += set.angle_noise_deg * noise(rng);
        distance *= 1 + set.distance_noise * distance * noise(rng);

        int arrow = 0;
        if (distance < set.arrow_range && chance(rng) >= set.arrow_miss) {
            arrow = (int)set.arrow;
            if (chance(rng) < set.arrow_wrong) arrow = 3 - arrow;
        }
        int16_t angle_cdeg = (int16_t)lround(angle_deg * 100);
        uint16_t distance_mm = (uint16_t)lround(distance * 1000);
        m.bytes[1] = 1;
        m.bytes[2] = angle_cdeg & 0xFF;                 // little-endian
        m.bytes[3] = (angle_cdeg >> 8) & 0xFF;
        m.bytes[4] = distance_mm & 0xFF;
        m.bytes[5] = (distance_mm >> 8) & 0xFF;
        m.bytes[6] = arrow;
    }
    for (int i = 1; i <= 6; i++) m.bytes[7] += m.bytes[i];   // checksum
    in_flight.push_back(m);
}

// ------------------------------ one time step -------------------------------
void step() {
    const double dt = STEP_US * 1e-6;
    now_us += STEP_US;
    double time = now_us * 1e-6;

    for (int i = 0; i < 2; i++) {
        // Motor: pins -> voltage -> wheel speed
        double duty = pin_level[HW_EN_PIN] ? pwm_duty[HW_PWM_PIN[i]] / 255.0 : 0;
        double voltage = duty * set.battery_v;
        if (pin_level[HW_DIR_PIN[i]] != HW_FORWARD_LEVEL[i]) voltage = -voltage;
        double pushing = 0;                             // what is left after friction
        if (fabs(voltage) > set.friction_v) pushing = voltage - copysign(set.friction_v, voltage);
        wheel_speed[i] += dt * set.motor_sigma * (set.motor_gain * pushing - wheel_speed[i]);
        wheel_angle[i] += dt * wheel_speed[i];
        if (fabs(wheel_speed[i]) > 0.05 && first_move_time < 0) first_move_time = time;

        // Encoder: one edge on channel A per count, B says which way
        long count_now = (long)floor(wheel_angle[i] * COUNTS_PER_RAD);
        while (encoder_count[i] != count_now) {
            int direction = (count_now > encoder_count[i]) ? 1 : -1;
            encoder_count[i] += direction;
            int a = !pin_level[HW_ENC_A_PIN[i]];
            pin_level[HW_ENC_A_PIN[i]] = a;
            pin_level[HW_ENC_B_PIN[i]] = (direction * HW_ENC_DIR[i] > 0) ? !a : a;
            if (pin_isr[HW_ENC_A_PIN[i]]) pin_isr[HW_ENC_A_PIN[i]]();
        }
    }

    // Robot: the two wheel speeds move the center of rotation
    double speed = set.wheel_radius * (wheel_speed[0] + wheel_speed[1]) / 2;
    double turn_rate = set.wheel_radius * (wheel_speed[1] - wheel_speed[0]) / set.wheel_base;
    x += dt * speed * cos(heading);
    y += dt * speed * sin(heading);
    heading += dt * turn_rate;
    closest = fmin(closest, beaconDistance());

    // Camera frames, and messages reaching the Arduino
    if (time >= next_frame_time) {
        next_frame_time += set.frame_period;
        if (time >= set.camera_start) takeFrame(time);
    }
    while (!in_flight.empty() && in_flight.front().arrive_time <= time) {
        memcpy(wire_buffer, in_flight.front().bytes, 8);
        wire_length = 8;
        wire_position = 0;
        in_flight.pop_front();
        if (wire_callback) wire_callback(wire_length);
    }

    if (trace_file && now_us % 10000 == 0) {
        fprintf(trace_file, "%.3f,%.4f,%.4f,%.4f\n", time, x, y, heading);
    }
}

}  // namespace sim

// ------------------------------ Arduino functions ---------------------------
unsigned long millis() { sim::step(); return sim::now_us / 1000; }
unsigned long micros() { return sim::now_us; }
void pinMode(int pin, int mode) { if (mode == INPUT_PULLUP) sim::pin_level[pin] = HIGH; }
void digitalWrite(int pin, int value) { sim::pin_level[pin] = value; }
int digitalRead(int pin) { return sim::pin_level[pin]; }
void analogWrite(int pin, int value) { sim::pwm_duty[pin] = constrain(value, 0, 255); }
void attachInterrupt(int pin, void (*isr)(), int) { sim::pin_isr[pin] = isr; }

SerialSim Serial;
void SerialSim::print(const char *text) {
    for (const char *c = text; *c; c++) print(*c);
}
void SerialSim::print(char c) {
    if (c != '\n') { sim::serial_line += c; return; }
    if (sim::serial_line == "Finished") sim::finished = true;
    if (sim::serial_file) fprintf(sim::serial_file, "%s\n", sim::serial_line.c_str());
    sim::serial_line.clear();
}
void SerialSim::print(int value) { print((long)value); }
void SerialSim::print(unsigned int value) { print((unsigned long)value); }
void SerialSim::print(long value) { print(std::to_string(value).c_str()); }
void SerialSim::print(unsigned long value) { print(std::to_string(value).c_str()); }
void SerialSim::print(double value, int digits) {
    char text[40];
    snprintf(text, sizeof(text), "%.*f", digits, value);
    print(text);
}

WireSim Wire;
void WireSim::onReceive(void (*callback)(int)) { sim::wire_callback = callback; }
int WireSim::available() { return sim::wire_length - sim::wire_position; }
int WireSim::read() { return sim::wire_buffer[sim::wire_position++]; }

// ------------------------------ the sketch ----------------------------------
#include SKETCH

// ------------------------------ main ----------------------------------------
int main(int argc, char **argv) {
    for (int a = 1; a < argc; a++) {
        std::string arg = argv[a];
        size_t eq = arg.find('=');
        std::string name = arg.substr(0, eq), value = (eq == std::string::npos) ? "" : arg.substr(eq + 1);
        bool known = false;
        if (name == "trace")  { sim::trace_path = value;  known = true; }
        if (name == "serial") { sim::serial_path = value; known = true; }
        for (const sim::Name &n : sim::NAMES) {
            if (name == n.name) { *n.value = atof(value.c_str()); known = true; }
        }
        if (!known || eq == std::string::npos) {
            fprintf(stderr, "unknown setting: %s\n", arg.c_str());
            return 2;
        }
    }
    if (!sim::trace_path.empty()) {
        sim::trace_file = fopen(sim::trace_path.c_str(), "w");
        if (sim::trace_file) fprintf(sim::trace_file, "time,x,y,heading\n");
    }
    if (!sim::serial_path.empty()) sim::serial_file = fopen(sim::serial_path.c_str(), "w");

    // The robot starts at the origin facing +x. The beacon is distance_ft
    // away, heading_deg to the left of straight ahead.
    sim::rng.seed((unsigned)sim::set.seed);
    double beacon_range = sim::set.distance_ft * 0.3048;
    sim::beacon_x = beacon_range * cos(sim::set.heading_deg * PI / 180);
    sim::beacon_y = beacon_range * sin(sim::set.heading_deg * PI / 180);
    sim::pin_level[sim::HW_NSF_PIN] = HIGH;             // no driver fault

    // Run the sketch, noting when it changes stage
    double stage_time[DONE + 1];
    for (int s = 0; s <= DONE; s++) stage_time[s] = -1;
    double arrive_distance = -1;        // from the beacon when the approach ended
    double finish_time = -1;
    Stage previous = WAIT;

    setup();
    while (sim::now_us * 1e-6 < sim::set.max_time) {
        loop();
        double time = sim::now_us * 1e-6;
        if (stage != previous) {
            if (stage_time[stage] < 0) stage_time[stage] = time;
            if (previous == APPROACH) arrive_distance = sim::beaconDistance();
            previous = stage;
        }
        if (sim::finished && finish_time < 0) finish_time = time;
        if (finish_time >= 0 && time > finish_time + 1.0) break;   // let it coast to a stop
    }

    const double FT = 0.3048;
    printf("finished=%d first_move=%.2f seen=%.2f approach=%.2f arrive=%.2f done=%.2f "
           "arrive_ft=%.3f final_ft=%.3f closest_ft=%.3f bearing_deg=%.1f\n",
           sim::finished ? 1 : 0, sim::first_move_time, stage_time[FACE], stage_time[APPROACH],
           TURN_AT_BEACON ? stage_time[TURN] : stage_time[DONE], finish_time,
           arrive_distance / FT, sim::beaconDistance() / FT, sim::closest / FT,
           sim::beaconBearingDeg());

    if (sim::trace_file) fclose(sim::trace_file);
    if (sim::serial_file) fclose(sim::serial_file);
    return 0;
}
