#!/usr/bin/env python3
"""EENG350 Demo 2 - Try beacon.ino on a simulated robot, on a PC.

Compiles ../../beacon/beacon.ino together with sim_main.cpp (a simulated
robot, camera and Pi, see the top of that file) and runs demo trials.
Nothing here touches the Arduino. It checks the sketch's logic: stages,
signs, the I2C message, stopping distance, and which way it turns. It
can't check what only the real robot knows: wheel slip, the real camera,
lighting.

    python3 run_sim.py                      all start headings x 4/5/6 ft x both arrows
    python3 run_sim.py heading_deg=135 distance_ft=6 arrow=2      one trial
    python3 run_sim.py TURN_AT_BEACON=false                       trial type 1
    python3 run_sim.py MAX_RHO_DOT=0.3 RHO_DOT_LIMIT=0.4          try faster gains
    python3 run_sim.py blur_limit=0.2 SEARCH_STEP_DEG=40          camera that blurs when turning

Arguments are NAME=VALUE:
    UPPER CASE  a constant at the top of beacon.ino, changed for this run
                only (the sketch file itself is not modified)
    lower case  a setting of the simulated robot or camera (the Settings
                list in sim_main.cpp), for example latency=0.15,
                friction_v=0.7, wheel_base=0.36, camera_offset=0.12
    trace=FILE  (one trial) save time,x,y,heading every 10 ms as CSV
    serial=FILE (one trial) save what the sketch printed to Serial

A trial passes if the robot starts within 30 s, stops between FRONT_FT and
1.5 ft from the beacon and, for type 2, ends up turned 90 degrees the way
the arrow points.

Needs g++.
"""

import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SKETCH = HERE / "../../beacon/beacon.ino"

# The beacon counts as hit if the center of rotation gets this close [ft]:
# the front of the robot (the camera) is 10.5 cm ahead of the center, plus
# about 1 cm for the camera body: 0.115 m = 0.38 ft.
FRONT_FT = 0.38

SWEEP_HEADINGS = [0, 20, -20, 45, 90, 135, 180, -135, -90, -45]
SWEEP_DISTANCES = [4, 5, 6]


def patch_sketch(text, overrides):
    """Return the sketch with `const ... NAME = value;` replaced for each override."""
    for name, value in overrides.items():
        pattern = r"^(\s*const\s+[\w ]+?\b%s(?:\[\d*\])?\s*=\s*)[^;]+;" % re.escape(name)
        text, count = re.subn(pattern, lambda m: m.group(1) + value + ";", text,
                              count=1, flags=re.MULTILINE)
        if count == 0:
            sys.exit(f"no constant named {name} in {SKETCH.name}")
    return text


def build(overrides, build_dir):
    """Compile the (patched) sketch with the simulator. Returns (program, sketch text)."""
    text = patch_sketch(SKETCH.read_text(), overrides)
    sketch_copy = build_dir / "sketch.cpp"
    sketch_copy.write_text(text)
    program = build_dir / "beacon_sim"
    subprocess.run(["g++", "-O2", "-I", str(HERE), f'-DSKETCH="{sketch_copy}"',
                    str(HERE / "sim_main.cpp"), "-o", str(program)], check=True)
    return program, text


def run_trial(program, settings):
    """Run one trial and return its results as a dict of floats."""
    args = [f"{name}={value}" for name, value in settings.items()]
    output = subprocess.run([str(program)] + args, check=True,
                            capture_output=True, text=True).stdout
    return {key: float(value) for key, value in
            (item.split("=") for item in output.split())}


def judge(result, arrow, turns):
    """Return a list of what went wrong in a trial (empty = pass)."""
    problems = []
    if not result["finished"]:
        problems.append("never finished")
    if not 0 <= result["first_move"] <= 30:
        problems.append("did not start within 30 s")
    if result["arrive_ft"] < 0 or result["arrive_ft"] > 1.5 or result["final_ft"] > 1.5:
        problems.append("not within 1.5 ft")
    if result["closest_ft"] < FRONT_FT:
        problems.append("hit the beacon")
    if turns:
        # After a left turn the beacon is on the robot's right (-90 deg)
        wanted = -90 if arrow == 1 else 90
        if abs(result["bearing_deg"] - wanted) > 15:
            problems.append("wrong turn")
    return problems


def main():
    sketch_overrides, sim_settings = {}, {}
    for arg in sys.argv[1:]:
        if arg in ("-h", "--help") or "=" not in arg:
            sys.exit(__doc__)
        name, value = arg.split("=", 1)
        (sketch_overrides if name.isupper() else sim_settings)[name] = value

    with tempfile.TemporaryDirectory() as build_dir:
        program, text = build(sketch_overrides, Path(build_dir))
        turns = re.search(r"TURN_AT_BEACON\s*=\s*true", text) is not None
        print("Trial type", "2 (turn at the beacon)" if turns else "1 (stop at the beacon)")

        # One trial if the start was given, otherwise the whole sweep
        if "heading_deg" in sim_settings or "distance_ft" in sim_settings:
            trials = [{}]
        else:
            trials = [{"heading_deg": h, "distance_ft": d, "arrow": a, "seed": seed}
                      for seed, (h, d, a) in enumerate(
                          (h, d, a) for d in SWEEP_DISTANCES for h in SWEEP_HEADINGS
                          for a in ((1, 2) if turns else (1,)))]

        print("heading  dist  arrow |  seen  arrive   done | stop at  closest  beacon at | result")
        print("  (deg)  (ft)        |   (s)     (s)    (s) |    (ft)     (ft)      (deg) |")
        failures = 0
        times = []
        for trial in trials:
            settings = {**sim_settings, **trial}
            result = run_trial(program, settings)
            arrow = int(float(settings.get("arrow", 1)))
            problems = judge(result, arrow, turns)
            failures += bool(problems)
            times.append(result["done"])
            print("%7g %5g  %5s | %5.1f  %6.1f %6.1f | %7.2f  %7.2f  %9.1f | %s" % (
                float(settings.get("heading_deg", 90)), float(settings.get("distance_ft", 5)),
                "left" if arrow == 1 else "right", result["seen"], result["arrive"],
                result["done"], result["final_ft"], result["closest_ft"],
                result["bearing_deg"], ", ".join(problems) or "ok"))

        print("%d of %d trials passed, time %.1f to %.1f s" % (
            len(trials) - failures, len(trials), min(times), max(times)))
        sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
