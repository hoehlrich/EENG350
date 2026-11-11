# Control Subsystem (Demo 2)

Runs on the Arduino. Listens to the Pi's marker messages, finds the beacon,
drives to `STOP_DISTANCE_FT` from it and, for trial type 2, turns 90° the way
the arrow points.

The motion controller is Demo 1's, unchanged: distance ρ and heading φ each
follow a trapezoidal reference, with an outer PI position loop around an
inner speed loop ([`demo-1/control/README.md`](../../demo-1/control/README.md),
"Controller"). Demo 1 gave it two fixed targets. Here the targets come from
the camera and keep changing while the robot moves.

## Layout

| Path | What it is |
|------|------------|
| `beacon/beacon.ino` | **The demo sketch.** Set `TURN_AT_BEACON` at the top and upload |
| `tests/marker_echo/marker_echo.ino` | Arduino end of the I2C link on its own: prints every message it receives, decoded, or why it would be dropped |
| `tests/host_sim/run_sim.py` | Compiles `beacon.ino` for the PC and runs it against a simulated robot, camera and Pi (`sim_main.cpp`, with `Arduino.h` and `Wire.h` standing in for the Arduino's). No hardware needed |
| `matlab/robot_params.m` | Every model and controller parameter in one place, same names as `beacon.ino` |
| `matlab/beacon_approach.slx`, `matlab/run_beacon.m` | Simulink model of the whole run (handout §6 with a camera), demo cases, and comparison with a recorded run |
| `matlab/beacon_supervisor.m`, `matlab/beacon_camera.m` | The code inside the model's Supervisor and Camera blocks: the sketch's stage logic, and the camera |
| `matlab/build_models.m` | Rebuilds the `.slx` from code |
| `matlab/readArduinoData.m` | Records one run from the serial port into a `.mat` file (from Demo 1) |

The Pi end of the link test, `send_marker.py`, is in [`../vision/`](../vision/README.md).
Demo 1's `odometry_check` and `robot_step_response` are still the tools for
signs, calibration and identification. They have not changed, so they are
not copied here.

## Hardware

The same as Demo 1 (motor shield, encoders, pins and signs:
[`demo-1/control/README.md`](../../demo-1/control/README.md), "Hardware"),
plus the I2C link to the Pi on A4 (SDA), A5 (SCL) and GND. The message is
described in [`../README.md`](../README.md#the-message).

## From a camera reading to targets

The camera says where the marker is relative to the camera: angle α (left
positive) and distance D. The controller needs targets in what the encoders
measure: a heading φ and a distance traveled ρ. For each message with the
marker visible:

```
mx = CAMERA_OFFSET_M + D*cos(α)        marker position from the center of
my = D*sin(α)                          rotation: mx ahead, my to the left

marker_phi = φ_then + atan2(my, mx)                     heading that points at the marker
marker_rho = ρ_then + sqrt(mx² + my²) - STOP_DISTANCE   ρ at which the robot is at the stop point
```

φ_then and ρ_then are the odometry from `CAMERA_LATENCY_MS` ago, when the
picture was taken, read from a short history the sketch keeps. Without that,
a reading taken while the robot is turning would be added to a heading the
robot has already left.

`marker_phi = φ + α` is the handout's φ_des = φ + φ_s (§6). Two things are
added to it. The camera is not at the center of rotation, which matters
close up: at the stop point the camera is 0.2 m from the marker while the
center is 0.3 m from it. And the target goes through the reference profile instead of
straight into the controller, so the wheels never get a step.

Because the targets are stored in odometry coordinates, they stay valid
when no new reading comes. If the marker drops out of view on the way in,
the robot finishes the approach on the last targets.

## Stages

| Stage | Targets | Next stage |
|-------|---------|------------|
| `WAIT` (0) | Motors off | `SEARCH` after `START_DELAY_S` |
| `SEARCH` (1) | φ target kept half a turn ahead, so the robot turns steadily at `SEARCH_PHI_DOT`. With `SEARCH_STEP_DEG` > 0 it instead turns that far, stops for `SEARCH_PAUSE_S` to look, and repeats | `FACE` on the first sighting |
| `FACE` (2) | φ target = `marker_phi`, updated on every sighting | `APPROACH` once the marker is within `FACE_TOL_DEG` of straight ahead and the turn has slowed. Back to `SEARCH` if the robot is facing where the marker was and hasn't seen it for `MARKER_LOST_S` |
| `APPROACH` (3) | φ target = `marker_phi` and ρ target = `marker_rho`, both updated on every sighting | Arrived once within `ARRIVE_TOL_M` of the stop point and nearly stopped. Type 1: `DONE`. Type 2: `TURN` as soon as the arrow is known |
| `TURN` (4) | φ target = heading that faces the marker ± 90° | `DONE` once settled, as in Demo 1 |
| `DONE` (5) | Motors off | |

**The arrow.** Every sighting that includes an arrow reading is a vote:
+1 for left, −1 for right, with the total capped at ±`ARROW_MAX_VOTES` so the
most recent readings, taken closest to the beacon, decide. The arrow is
known once the total reaches ±`ARROW_MIN_VOTES`. If the robot arrives with no
answer it waits `ARROW_WAIT_S`, still looking, then turns `DEFAULT_TURN`.

**Changes to Demo 1's controller code.** There are two. The reference
profile used to assume its target never moved. Now, if the target jumps so
close that the profile can't stop in time, the profile brakes at its
acceleration limit, runs past and comes back, instead of snapping to the
target at full speed. And the stage's "settled" test is only used to end
the last turn. The earlier stages end on the looser tests in the table,
because the demo scores time, and 1.5 ft is a wide target.

## Settings

All at the top of `beacon.ino`. The ones to look at first:

| Constant | Value | What it is |
|----------|-------|------------|
| `TURN_AT_BEACON` | `true` | `true` for trial type 2, `false` for type 1 |
| `STOP_DISTANCE_FT` | 1.0 | Center of rotation to marker at the stop. Must be under 1.5, and big enough that the front of the robot stays off the beacon |
| `CAMERA_OFFSET_M` | 0.10 | **Measure:** camera lens ahead of the line between the wheels |
| `CAMERA_LATENCY_MS` | 80 | Guess at the age of a reading. See the table under "Bringing it up" |
| `SEARCH_PHI_DOT` | 1.5 rad/s | Search speed. Limited by motion blur. 0.5 rad/s is known to work |
| `SEARCH_STEP_DEG` | 0 | 0 = turn continuously. About 40 = stop and look every 40° |
| `MAX_RHO_DOT`, `RHO_ACCEL` | 0.3 m/s, 0.3 m/s² | 3 times Demo 1's values. A trial takes 7 to 14 s in simulation (15 to 33 s at Demo 1's) |
| `MAX_PHI_DOT`, `PHI_ACCEL` | 1.5 rad/s, 3 rad/s² | 3 times Demo 1's values |
| `MAX_VOLTAGE` | 4 V | Cap on each wheel. The 3× speeds need up to about 3.7 V in simulation |

The robot and controller constants are Demo 1's final ones. Keep them the
same in `matlab/robot_params.m`.

## Bringing it up on the robot

Opening the USB serial port resets the Uno and starts the run, as in Demo 1.

1. **The link.** Upload `tests/marker_echo`, open the serial monitor at
   115200, and run `python3 send_marker.py 30 1.5 L` on the Pi. The monitor
   should print `marker at 30.00 deg, 1.500 m, arrow left`. Then run
   `demo2.py` and hold a beacon in front of the camera: the angle should go
   positive when the marker moves to the camera's left, and the distance
   should match a tape measure.
2. **The sketch, wheels off the ground.** Upload `beacon`. After 1 s both
   wheels should turn in opposite directions (the search), left wheel
   backward. Run `python3 send_marker.py 30 1.5 L`: the wheels should finish
   a short left turn, run forward together for about 5 s, then turn left
   again and stop. With `R` the last turn is the other way.
3. **On the floor, still without the camera.** The same command drives the
   robot about 1.3 m (1.5 m + `CAMERA_OFFSET_M` − 1 ft) along a line about
   28° left of where it was pointing when the reading arrived. This is all
   odometry, so it should be as accurate as Demo 1 was.
4. **With the camera, trial type 1.** Start `demo2.py`, set
   `TURN_AT_BEACON = false`, put the beacon 5 ft away and reset. Measure from
   the center of rotation to the beacon when it stops.
5. **Trial type 2**, with each beacon, from several start headings.
6. **Speed.** Raise `MAX_RHO_DOT`, `RHO_ACCEL`, `MAX_PHI_DOT`, `PHI_ACCEL` and
   `SEARCH_PHI_DOT` a step at a time, keeping `RHO_DOT_LIMIT` and
   `PHI_DOT_LIMIT` about 50% above the two speeds, and `MAX_VOLTAGE` high
   enough for them (driving at a steady speed v takes about
   v / `K_rho` / 2 + `V_STATIC` volts on each wheel, and turning at a steady
   rate takes rate / `K_phi` / 2 + `V_STATIC`). Try the new values in
   `run_sim.py` first.

| What happens | Likely cause |
|--------------|--------------|
| Searches forever with the beacon in view | No valid messages: check with `marker_echo`. Or the marker blurs while turning: lower `SEARCH_PHI_DOT`, set `SEARCH_STEP_DEG` to 40, or shorten the camera's exposure |
| Swings past the marker when it first sees it, then comes back | `CAMERA_LATENCY_MS` is too low. Too high and it stops short and creeps up instead |
| Stops too close or too far by the same amount every time | `CAMERA_OFFSET_M`, or the camera's distance is off (marker size, calibration). Or just change `STOP_DISTANCE_FT` |
| Drives past the beacon's side | The camera isn't pointing straight ahead, or the angle sign is wrong (step 1) |
| Waits 2 s at the beacon, then turns left every time | The arrow isn't being read: `arrow_votes`, the last column of the serial output, stays 0. Tune the colors (`vision/README.md`) |
| Turns the wrong way | `arrow_votes` has the wrong sign: the Pi is reading the wrong color |

## PC simulation of the sketch

```
cd tests/host_sim
python3 run_sim.py                              # 60 trials: 10 start headings x 4/5/6 ft x both arrows
python3 run_sim.py TURN_AT_BEACON=false         # trial type 1
python3 run_sim.py heading_deg=135 distance_ft=6 arrow=2 trace=path.csv   # one trial
python3 run_sim.py MAX_RHO_DOT=0.3 RHO_DOT_LIMIT=0.45 RHO_ACCEL=0.3      # try faster values
python3 run_sim.py blur_limit=0.2 SEARCH_STEP_DEG=40   # a camera that can't see while turning
```

This compiles the real `beacon.ino` with `g++` and runs it against a model
of the robot (motors, encoders that call the sketch's own interrupt
handlers, friction), the camera (field of view, range, frame rate, noise,
blur) and the Pi (the I2C message, delivered late). An upper case argument
changes a constant in the sketch for that run only. A lower case one
changes the simulated robot or camera, so the sketch can be tried against a
robot that differs from what it believes: `wheel_base=0.37`, `latency=0.2`,
`friction_v=0.8`. The full list is the `Settings` struct in `sim_main.cpp`.

Use it to check a change to the stage logic before uploading, and to try
faster speeds. It has no wheel slip, so it will accept accelerations the
real floor won't.

## Simulink model

`beacon_approach.slx` is the handout's §6 block diagram: Demo 1's Forward
and Rotation channels, the conversion to (x, y), and a Camera block that
works out the angle to the beacon from where the robot is. Between the
camera and the channels sits a Supervisor block running
`beacon_supervisor.m`, which is the sketch's stage logic line for line.

1. Run `run_beacon`. It simulates the trial set at the top of
   `robot_params.m` and plots the path, heading, distance and stage, then
   prints arrival time and final position for a list of demo cases.
2. To compare with the robot: set the beacon up as `BEACON_HEADING_DEG`,
   `BEACON_DISTANCE_FT` and `ARROW` say, with a USB cable that reaches, and
   run `readArduinoData("beacon_data.mat", 12)`, then `run_beacon` again.
   The columns are time, stage, ρ reference, ρ, φ reference, φ, VL, VR,
   camera angle (deg), camera distance (m), time since the last sighting
   (s) and arrow votes.

Change `robot_params.m`, not the block parameters. Edit
`beacon_supervisor.m` or `beacon_camera.m` freely: the blocks call those
files. Run `build_models` only after changing the model's structure. The
model has no friction, deadband or camera noise.

## What has been tested

None of this has run on the robot yet. What has been checked:

- All sketches compile for the Uno (`beacon` uses 39% of flash, 37% of RAM).
- The message: 206 messages packed by the Pi code (now `make_message()` in `vision/demo2.py`, byte-for-byte the same) were decoded
  correctly by `beacon.ino`'s receive code, and messages with a wrong
  length, ID or checksum were dropped.
- `run_sim.py`: 60 of 60 type 2 trials and 30 of 30 type 1 trials pass,
  stopping 1.00 ft from the beacon and turning the right way, in 7 to 14 s
  (type 2) at the current speeds (3 times Demo 1's; 15 to 33 s at Demo 1's).
  At the current speeds they also all pass with: message latency of 20 to
  200 ms; 120 frames per second; 90% of frames missing the marker; 30% of
  arrow readings wrong; friction of 0.2 to 0.8 V; a miscalibrated robot (as
  below); and only 5 readings in the whole run. A camera that can't detect
  anything while turning faster than 1 rad/s fails the 1.5 rad/s search,
  and passes with `SEARCH_STEP_DEG=40 SEARCH_PAUSE_S=0.2` (7 to 19 s).
  At Demo 1's speeds they also all pass with: message latency
  of 0 to 250 ms; 5 frames per second; 90% of frames missing the marker;
  30% of arrow readings wrong; friction of 0.2 to 0.8 V; a robot whose
  wheel base and wheel radius are a few percent off, whose camera is 4 cm
  from where the sketch thinks, and whose battery is at 8 V; a 30° field of
  view with a 1.5 rad/s search; speeds about 4 times higher (6 to 12 s per
  trial); and only 5 readings in the whole run, which is what one line
  typed into `send_marker.py` sends. Each of those was tried on its own.
  With 90% of frames missed and 30% of arrow readings wrong at the same
  time, 1 trial of 60 turned the wrong way. With a camera that can't detect anything while turning
  faster than 0.2 rad/s the continuous search fails, as it should, and
  `SEARCH_STEP_DEG=40` passes.
- The simulation does catch mistakes: with a motor sign, an encoder sign or
  the message ID changed in the sketch, every trial fails.
- The Simulink model gives the same times and stopping distances as
  `run_sim.py`.
