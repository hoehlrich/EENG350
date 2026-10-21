# Control Subsystem

Runs on the Arduino. Turns the robot in place by `TARGET_ANGLE_DEG`, then
drives it straight `TARGET_DISTANCE_FT` and stops (Demo 1 tasks 2 and 3).
The targets are constants in the sketch, set before each upload. There is no
link to the Pi for this demo.

The controller follows the SteeringControl handout. It doesn't control each
wheel separately. It controls the distance traveled ρ and the heading φ
(both measured at the center of rotation), each with an outer position loop
around an inner speed loop. Most of the code comes from the mini project's
`position_control`: pins, encoder interrupts, timing,
anti-windup and deadband.

## Layout

| Path | What it is |
|------|------------|
| `drive/drive.ino` | **The demo sketch.** Set the targets at the top and upload |
| `robot_step_response/robot_step_response.ino` | Open loop V̄a step (forward) or ΔVa step (`#define TEST_ROTATION`) on the floor, to identify Gρ and Gφ |
| `tests/odometry_check/odometry_check.ino` | Push the robot by hand and read ρ and φ. Checks signs and calibrates r and d. Also measures static friction |
| `matlab/robot_params.m` | Every model and controller parameter in one place, same names as `drive.ino` |
| `matlab/readArduinoData.m` | Records one experiment from the serial port into a `.mat` file (from the mini project) |
| `matlab/robot_openloop.slx`, `matlab/run_identify.m` | Open loop model, comparison with the step experiments, and a fit of K and σ for ρ and φ |
| `matlab/robot_motion.slx`, `matlab/run_motion.m` | Full turn and drive model (handout §6 without the camera), inner loop gain sweep, outer loop margins, comparison with a recorded run, final error for the demo cases |
| `matlab/motion_reference.m` | The sketch's reference profile and stage sequence in MATLAB, so the model gets the same references as the robot |
| `matlab/build_models.m` | Rebuilds both `.slx` files from code |

## Hardware

| Connection | Arduino pin |
|------------|-------------|
| Motor driver enable | 4 |
| Left motor (shield M2) direction / PWM | 8 / 10 |
| Right motor (shield M1) direction / PWM | 7 / 9 |
| Left encoder A (yellow) / B (white) | 2 / 6 |
| Right encoder A (yellow) / B (white) | 3 / 5 |
| Encoder Vcc (blue) / GND (green) | 5 V / GND |

Left and right are as seen from behind the robot, facing the way it drives. Each wheel's motor and encoder are on different-numbered pins: the left motor is on the shield's M2 output, but its encoder is on pins 2/6. The mini project paired them the other way (M1 with encoder 2/6), and on this robot that sends each wheel's command to the other wheel. Both wheels still go forward together, so a straight drive looks fine, but every heading correction turns the robot the wrong way, and `drive` went in circles. The handout also calls the right motor "motor 1", which matches.

## Sign conventions

Every wheel quantity in the code is positive when that wheel moves the robot forward. That takes one sign per part, `{left, right}`:

- `ENC_SIGN = {1, -1}`: applied in the encoder interrupt, so each wheel's count goes up when it rolls forward. Flip a wheel's entry if its count goes down when you push the robot forward.
- `MOTOR_SIGN = {-1, 1}`: applied when setting the direction pin, so a positive voltage turns that wheel forward. Flip a wheel's entry if a positive voltage turns it backward.

The motors face opposite ways, which is why each array has one -1. `odometry_check` tests both: push the robot by hand for `ENC_SIGN`, and send `m` for `MOTOR_SIGN`. The `m` pulse runs each motor on its own, so it also catches swapped motor pins.

- ρ = r(θL + θR)/2 is positive forward. φ = r(θR − θL)/d is positive counterclockwise seen from above (turning left). This matches the handout and the camera's sign convention.

The pins, `MOTOR_SIGN` and `ENC_SIGN` must be the same in all three sketches. `WHEEL_RADIUS_M` and `WHEEL_BASE_M` must also match `robot_params.m`.

## Controller

Every 10 ms:

```
reference   rho_ref, rho_dot_ref, phi_ref, phi_dot_ref from a trapezoidal profile
            (speed <= MAX_*_DOT, acceleration <= *_ACCEL)

outer       rho_dot_d = rho_dot_ref + Kp_rho*(rho_ref - rho) + Ki_rho*integral    limited to ±RHO_DOT_LIMIT
            phi_dot_d = phi_dot_ref + Kp_phi*(phi_ref - phi) + Ki_phi*integral    limited to ±PHI_DOT_LIMIT

inner       Vbar = rho_dot_d/K_rho + Kp_rho_dot*(rho_dot_d - rho_dot)
            dVa  = phi_dot_d/K_phi + Kp_phi_dot*(phi_dot_d - phi_dot)

wheels      VR = (Vbar + dVa)/2 ± V_STATIC,  VL = (Vbar - dVa)/2 ± V_STATIC
            both scaled by the same factor if either is over MAX_VOLTAGE (3 V)
```

The sketch runs in stages: `WAIT` (`START_DELAY_S`, motors off), `TURN` (φ target set, ρ held at 0), `DRIVE` (ρ target set, φ held at the turned angle), then `DONE` (motors off). Because φ is held during `DRIVE`, the heading is corrected the whole way, which keeps the robot on a straight line. A stage ends once both errors have stayed inside `RHO_TOL` and `PHI_TOL` for `SETTLE_HOLD_S`. If friction keeps the robot just outside, it ends `SETTLE_TIMEOUT_S` after the profile finished.

How this differs from a plain handout design, and why:

- **Reference profile.** The robot never gets a step in position. It gets a ramp with limited acceleration, so the wheels don't slip. Slip is error the encoders never see. The profile's speed is fed forward into the outer loop, so P on position only has to correct the small error left over.
- **Model feedforward in the inner loop** (`speed_d/K`). With P alone the speed settles about 20% below the command, and an integrator in the outer loop has to make up the difference. In simulation that integrator wound up during a 10 ft run. The robot overshot the end by about 0.8 in and stopped 0.3 in off. With the feedforward the inner loop has a DC gain of 1.
- **Integral only after the profile ends.** Same reason. While the reference is moving, P plus feedforward tracks it. The integral is only there at the end, to push through static friction.
- **`V_STATIC`.** The static friction voltage is added to each wheel. In a simulation with a 0.5 V deadzone and no `V_STATIC`, the robot stalled up to 0.4 in from the goal. With `V_STATIC` anywhere from 33% below to 50% above the true deadzone, the error stayed under about 0.12 in. When `V_STATIC` was only half the deadzone, the error went back up to 0.4 in, so guessing high is safer than guessing low.
- **PI, not PD, outside.** The handout suggests PD for C(s). The profile already keeps the outer loop's error small and slow, so D adds noise without helping. I adds what friction takes away.
- **Deadband.** Same as the mini project: inside the tolerances the voltage is 0 and the integrals freeze, so the motors don't whine.

| Gain | Value | Source |
|------|-------|--------|
| `K_rho`, `K_phi` | 0.0825, 0.471 | r·K/2 and r·K/d from the mini project's wheel model. **Replace with `run_identify` fits** |
| `Kp_rho_dot` | 48 V per m/s | inner pole ≈ 25 1/s on the default model |
| `Kp_phi_dot` | 8.5 V per rad/s | inner pole ≈ 25 1/s on the default model |
| `Kp_rho`, `Ki_rho` | 3, 2 | starting values, checked in `run_motion` |
| `Kp_phi`, `Ki_phi` | 4, 2 | starting values, checked in `run_motion` |
| `MAX_RHO_DOT`, `RHO_ACCEL` | 0.1 m/s, 0.1 m/s² | slow for testing: 10 ft in about 32 s |
| `MAX_PHI_DOT`, `PHI_ACCEL` | 0.5 rad/s, 1 rad/s² | 180° in about 6.8 s |
| `MAX_VOLTAGE` | 3 V per wheel | hard cap so a sign mistake can't make it run off; slow runs need under 2 V |
| `V_STATIC` | 0.5 V | **measure with `odometry_check` (f / t)** |

Keep `robot_params.m` and the constants at the top of `drive.ino` in sync.

## Running things

Opening the USB serial port resets the Uno, which zeroes the encoders and restarts the sketch. For `drive` and `robot_step_response`, that means it starts the run.

**1. Signs and calibration (`tests/odometry_check`).** Upload it and open the serial monitor at 115200. Motors stay off unless you send a command.
1. Push the robot straight forward. Both counts and ρ should go up, and φ should stay near 0. Flip `ENC_SIGN` for a wheel whose count goes down. Turn the robot left by hand: φ should go up.
2. Wheel radius: send `z`, then push the robot exactly 120 in along a tape measure with the center hole on the tape. New `WHEEL_RADIUS_M` = old × 120 / ρ.
3. Wheel base (do it after step 2): send `z`, then spin the robot 5 full turns in place about the center hole. New `WHEEL_BASE_M` = old × φ / 1800.
4. Static friction: on the demo floor, send `f` and `t` a few times each. Use about the smaller of the two as `V_STATIC`. Erring high is safer than low.
5. Send `m`. It pulses the left motor, then the right. Each line should say `ok`. `went backward` means flip that wheel's `MOTOR_SIGN`. `WRONG WHEEL` means the motor pins are swapped.

Copy the results into all three sketches and `robot_params.m`.

**2. Identify Gρ and Gφ.**
1. Put the robot on the floor with about 1–2 ft of room ahead. Upload `robot_step_response/`.
2. In MATLAB, from `matlab/`, run `readArduinoData("rho_step_data.mat", 5)`.
3. Uncomment `#define TEST_ROTATION`, upload, and run `readArduinoData("phi_step_data.mat", 5)`.
4. Run `run_identify`. It prints K and σ for ρ and φ. Copy them into `robot_params.m`, and `K_rho` and `K_phi` into `drive.ino`.

The bottom plots show the other output during each step: φ̇ while driving forward, ρ̇ while turning. Both should stay near 0. If φ̇ drifts during the forward step, the motors don't match, and the φ loop has to correct for it.

**3. Tune in simulation.** Run `run_motion`. It shows inner loop steps for a few gains, the outer loop phase margins, the simulated run with `TARGET_*` from `robot_params.m`, and the final error for a list of demo cases. Retune `Kp_*_dot` first if the identified model is very different from the default, keeping the inner pole well above the outer crossover. Copy the gains into `drive.ino`.

**4. Record a run.** Set the same short target in `drive.ino` and `robot_params.m` (the USB cable has to reach). Upload, put the robot down, and run `readArduinoData("drive_data.mat", 7)`, then `run_motion`.

**5. Calibrate under power.** Unplug USB, and use the reset button to start runs. Try 10 ft straight, 90° + 3 ft, and −45° + 5 ft, and measure where the center hole ends up.
- Consistently short or long by the same fraction: scale `WHEEL_RADIUS_M`.
- Turns consistently over or under: scale `WHEEL_BASE_M`. Odometry over-reports turns when the wheels scrub, so the best d under power is usually a little bigger than the push test gives.
- Drifts sideways on the straight run: check the signs with `odometry_check` and that the heading loop is working (φ in the recorded data should stay at its target).

**Simulation only.**
- `run_identify` and `run_motion` plot only the simulation if there is no data file yet.
- Change `robot_params.m`, not the block parameters.
- Run `build_models` only after changing the model structure.
- To make a report, use `publish('run_motion.m', 'pdf')`.
- The model has no friction or slip, and its saturation is per channel (V̄a, ΔVa ≤ 2·`MAX_VOLTAGE`) instead of per wheel. It also stages the run on fixed timing: each stage ends `SETTLE_HOLD_S` after its profile finishes.
