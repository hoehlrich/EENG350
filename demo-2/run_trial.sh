#!/bin/bash
# EENG 350 SEED Lab - Demo 2: run one trial from the Pi.
#
#   ./run_trial.sh 1     trial type 1: find the beacon, drive to it, stop
#   ./run_trial.sh 2     trial type 2: the same, then turn 90 deg the way the arrow points
#
# (trial1.sh and trial2.sh just call this.)
#
# What it does:
#   1. Copies control/beacon/beacon.ino, sets TURN_AT_BEACON for this trial
#      type, and compiles it with arduino-cli. The file in the repo is not changed.
#   2. Starts the vision code (vision/demo2.py) so the camera is
#      already sending when the robot starts.
#   3. Uploads the sketch to the Arduino over USB. The Arduino resets when the
#      upload finishes, which is the go signal: it starts searching 1 s later.
#   4. Keeps the vision code running until you press Ctrl+C.
#
# Needs: arduino-cli with the arduino:avr core on the Pi, the Arduino plugged
# into the Pi's USB, and vision/camera_calibration.npz.

set -e

TRIAL="$1"
if [ "$TRIAL" = "1" ]; then
    TURN=false
elif [ "$TRIAL" = "2" ]; then
    TURN=true
else
    echo "Usage: $0 1|2"
    exit 1
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
FQBN="arduino:avr:uno"
PORT="${PORT:-/dev/ttyACM0}"        # override with: PORT=/dev/ttyUSB0 ./trial1.sh

if ! command -v arduino-cli > /dev/null; then
    echo "arduino-cli is not installed. Install it, then run: arduino-cli core install arduino:avr"
    exit 1
fi
if [ ! -e "$PORT" ]; then
    echo "No Arduino at $PORT. Plug it into the Pi (or set PORT=...)."
    exit 1
fi
if [ ! -f "$HERE/vision/camera_calibration.npz" ]; then
    echo "Missing $HERE/vision/camera_calibration.npz (copy it from Demo 1)."
    exit 1
fi

# 1. Compile a copy of the sketch with this trial's setting
BUILD="/tmp/demo2_trial$TRIAL"
rm -rf "$BUILD"
mkdir -p "$BUILD/beacon"
cp "$HERE/control/beacon/beacon.ino" "$BUILD/beacon/"
sed -i "s/^const bool  TURN_AT_BEACON *= *[a-z]*;/const bool  TURN_AT_BEACON   = $TURN;/" "$BUILD/beacon/beacon.ino"
if ! grep -q "TURN_AT_BEACON   = $TURN;" "$BUILD/beacon/beacon.ino"; then
    echo "Could not set TURN_AT_BEACON in beacon.ino (did that line change?)"
    exit 1
fi
echo "== Trial $TRIAL: compiling beacon.ino (TURN_AT_BEACON = $TURN)"
arduino-cli compile --fqbn "$FQBN" --output-dir "$BUILD/out" "$BUILD/beacon"

# 2. Start the vision code
echo "== Starting vision"
python3 "$HERE/vision/demo2.py" &
VISION=$!
trap 'kill $VISION 2>/dev/null; wait $VISION 2>/dev/null' EXIT
sleep 3                             # camera on and sending before the robot starts
if ! kill -0 $VISION 2>/dev/null; then
    echo "Vision code stopped on its own, not uploading."
    exit 1
fi

# 3. Upload: the robot starts when this finishes
echo "== Uploading to $PORT. The robot starts about 1 s after this finishes."
arduino-cli upload --fqbn "$FQBN" -p "$PORT" --input-dir "$BUILD/out" "$BUILD/beacon"
echo "== Running. Ctrl+C to stop the vision code."

# 4. Wait for the vision code (Ctrl+C ends it)
wait $VISION
