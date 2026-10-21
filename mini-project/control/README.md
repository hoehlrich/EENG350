# Control Subsystem

Runs on the Arduino. Receives a goal (0 or 1) for each wheel from the Pi over
I2C and holds each wheel at its goal with a PI position loop wrapped around
the proportional velocity loop from assignment 2. Goal 0 is the position at
reset (tape mark 0 up) and goal 1 is pi rad from there (tape mark 1 up).

## Layout

| Path | What it is |
|------|------------|
| `position_control/position_control.ino` | Main sketch for the demo. Uncomment `#define STEP_TEST` to run the quadrant table as a recorded step test instead of listening to the Pi. |
| `step_response/step_response.ino` | Open loop voltage step on both motors, used to measure K and sigma |
| `tests/motor_check/motor_check.ino` | Drives each motor alone and reports which encoder moved and which way. Run it first, or whenever a wheel runs away |
| `tests/i2c_echo/i2c_echo.ino` | Arduino end of the I2C link on its own: prints every message it receives |
| `tests/i2c_send_goals.py` | Pi end of the I2C link without the vision code: cycles through the quadrants or sends typed goals |
| `matlab/motor_params.m` | Every model and controller parameter in one place |
| `matlab/readArduinoData.m` | Records one experiment from the serial port into a `.mat` file |
| `matlab/openloop_sim.slx`, `matlab/run_openloop.m` | Open loop model, comparison with experiment, and a fit of K and sigma |
| `matlab/position_control.slx`, `matlab/run_position.m` | Closed loop position model, comparison with experiment, and a disturbance rejection demo |
| `matlab/build_models.m` | Rebuilds both `.slx` files from code |

## Hardware

| Connection | Arduino pin |
|------------|-------------|
| Motor driver enable | 4 |
| Left motor (motor 1) direction / PWM | 7 / 9 |
| Right motor (motor 2) direction / PWM | 8 / 10 |
| Left encoder A (yellow) / B (white) | 2 / 6 |
| Right encoder A (yellow) / B (white) | 3 / 5 |
| Encoder Vcc (blue) / GND (green) | 5 V / GND |
| Pi GPIO2 SDA (header pin 3) | A4 (SDA) |
| Pi GPIO3 SCL (header pin 5) | A5 (SCL) |
| Pi GND (header pin 6) | GND |

The encoder pins are how this robot is actually wired, as measured by
`tests/motor_check`. The two B wires are crossed relative to the usual 2/5,
3/6 layout. A positive voltage must also give positive counts, or that wheel
runs away. `MOTOR_DIR = {-1, -1}` flips both motors' direction pins to make
this true without rewiring. The pins and `MOTOR_DIR` must be the same in
`position_control`, `step_response`, and `motor_check`. After any rewiring,
run `tests/motor_check`: it watches all four encoder pins and prints the
values to use. The sketches turn off the Arduino's internal I2C pull-ups so
the bus stays at the Pi's 3.3 V.

## I2C protocol

The Arduino is an I2C slave at address `0x08` on the Pi's bus 1. Communication is one way: the Pi writes goals and the Arduino sends nothing back.

| Direction | Bytes | Python (`smbus2`) |
|-----------|-------|-------------------|
| Pi → Arduino | `[0, left_goal, right_goal]`, each goal 0 or 1 | `bus.write_i2c_block_data(0x08, 0, [left, right])` |

The Arduino ignores any message that isn't a valid goal message. Sending the same goal again does no harm, but the Pi only needs to send when the quadrant changes.

## Controller

Every 10 ms, for each wheel:

```
pos_error      = desired_pos - actual_pos
integral_error = integral_error + pos_error*dt     dt = measured loop time
desired_speed  = Kp_pos*pos_error + Ki_pos*integral_error     limited to ±MAX_SPEED
Voltage        = Kp_vel*(desired_speed - actual_speed)        limited to ±Battery_Voltage
```

For anti-windup, the sketch undoes that pass's integration whenever `desired_speed` or `Voltage` saturates (PID lecture, section 9.3). The Simulink PID block only clamps at its output limit (`MAX_SPEED`), so the model winds up a little more than the Arduino does.

Within `DEADBAND_RAD` (0.02 rad, about 1°) of the goal, the sketch sets the voltage to 0 and freezes the integral. Otherwise a voltage too small to overcome friction sits on the motor and makes it whine. The Simulink model has no deadband.

| Gain | Value | Source |
|------|-------|--------|
| `Kp_vel` | 3.2 V per rad/s | working assignment 2 code |
| `Kp_pos` | 5 (rad/s) per rad | conservative starting value |
| `Ki_pos` | 2 (rad/s) per rad·s | conservative starting value |
| `MAX_SPEED` | 6 rad/s | `run_position.m` |

The first version of `position_control` had the two encoders swapped (each controller read the other wheel), which made the wheels mirror each other and run away. The encoder, direction, and timing code now follow the working assignment 2 code. The Simulink model is continuous time, so it can't predict problems caused by the 10 ms sample time or friction.

Re-measure K and sigma on the real wheels, then retune:
1. Set `Ki_pos = 0`.
2. Raise `Kp_pos` until the step is fast without overshoot.
3. Add `Ki_pos` until a wheel turned by hand comes back quickly without much overshoot.

Keep `motor_params.m` and the constants at the top of `position_control.ino` in sync.

## Running things

**Measure the motors.**
1. Lift the wheels and upload `step_response/`.
2. In MATLAB, from `matlab/`, run `readArduinoData("openloop_data.mat", 5)`.
3. Run `run_openloop`. It prints fitted K and sigma for each wheel.
4. Copy them into `motor_params.m`.

**Position step response.**
1. Uncomment `#define STEP_TEST` in `position_control.ino`.
2. Set both wheels to 0 and upload.
3. Run `readArduinoData("position_data.mat", 7)`, then `run_position`.

The test runs the quadrant table by itself: NE (0 0) until 1 s, then NW (0 1), SW (1 1), SE (1 0) and NE (0 0), 3 s each, 13 s in total. Each change moves one wheel while the other holds, so it covers one wheel to a position, both wheels to independent positions, and both directions. `run_position` prints the overshoot and settling time of every move, and how far each wheel drifts while holding. After the recording, the wheels are held at 0 0, so you can disturb them by hand. The sequence is `SUITE_GOALS` in the sketch and `Suite_Goals` in `motor_params.m`; keep them the same.

**I2C link on its own.**
1. Upload `tests/i2c_echo/` and open the serial monitor at 115200.
2. On the Pi, check that `i2cdetect -y 1` shows `08`.
3. Run `python3 tests/i2c_send_goals.py` (or `-i` to type goals).

**Full system.**
1. Set both wheels to 0.
2. Upload `position_control/` with `STEP_TEST` commented out.
3. Start the vision code, or `i2c_send_goals.py`.

The serial monitor shows time, goals, positions, and voltages 10 times a second.

Opening the USB serial port resets the Uno, which zeroes the encoders. Always put both wheels back to 0 before opening the serial monitor or running `readArduinoData`.

**Simulation only.**
- `run_openloop` and `run_position` plot only the simulation if there is no data file yet.
- Change `motor_params.m`, not the block parameters.
- Run `build_models` only after changing the model structure.
- To make a report, use `publish('run_position.m', 'pdf')`.
