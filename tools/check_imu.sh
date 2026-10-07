#!/usr/bin/env bash
# Why is the IMU not publishing? Runs the chain in order and stops at the
# first thing that is actually wrong.
#
#   ./tools/check_imu.sh              on the Pi
#   ssh Scrapify@10.42.0.241 'cd ~/varun/nexva_ws && ./tools/check_imu.sh'
#
# "No BNO055 - imu is not being published" is the DRIVER's last step, not the
# first. It is the same message whether the board is unplugged, the bus is off,
# or the libraries are missing - so it tells you nothing on its own. This walks
# the chain underneath it and names the step that failed.
#
# Exit 0 only when the board actually answers.
set -u

BUS="${BUS:-1}"
ADDR_HEX="${ADDR_HEX:-28}"
STEP=0
fail() { echo; echo "[imu] STOPPED AT STEP $STEP: $1"; echo "[imu] fix: $2"; exit 1; }
pass() { echo "[imu]   OK  - $1"; }

echo "[imu] checking the IMU chain on $(hostname) ($(uname -m))"
echo

STEP=1; echo "[imu] 1/6 is this a Pi?"
if [ -r /proc/device-tree/model ]; then
    pass "$(tr -d '\0' < /proc/device-tree/model)"
else
    echo "[imu]   not a Pi - the Adafruit libraries import 'board', which only"
    echo "[imu]   exists on Pi-like hardware. On a laptop the driver is EXPECTED"
    echo "[imu]   to say 'No BNO055'. Run this on the robot."
    exit 1
fi

STEP=2; echo "[imu] 2/6 is the I2C bus enabled?"
[ -e "/dev/i2c-$BUS" ] \
    && pass "/dev/i2c-$BUS exists" \
    || fail "no /dev/i2c-$BUS" \
            "sudo raspi-config -> Interface Options -> I2C -> enable, then reboot"

STEP=3; echo "[imu] 3/6 are the i2c tools installed?"
command -v i2cdetect >/dev/null \
    && pass "i2cdetect present" \
    || fail "i2cdetect missing" "sudo apt install -y i2c-tools"

STEP=4; echo "[imu] 4/6 does anything answer on the bus?"
SCAN="$(i2cdetect -y "$BUS" 2>&1)"
echo "$SCAN" | sed 's/^/[imu]       /'
FOUND="$(echo "$SCAN" | tr ' ' '\n' | grep -cE "^(28|29)$" || true)"
DEVICES="$(echo "$SCAN" | tail -n +2 | tr ' ' '\n' | grep -cE '^[0-9a-f]{2}$' || true)"

if [ "$FOUND" -eq 0 ] && [ "$DEVICES" -eq 0 ]; then
    fail "the bus is COMPLETELY EMPTY - nothing is connected to it at all" \
"this is wiring, not software.

[imu]      FIRST CHECK THE OBVIOUS ONE: is the BNO055 wired to the ESP32
[imu]      instead of the Pi? That has happened on this robot, and it looks
[imu]      exactly like this - a totally idle Pi bus - because from the Pi's
[imu]      side nothing is there. The ROS driver runs on the PI, so the board
[imu]      must be on the PI's header.

[imu]      Four wires, Docs/circuit.txt section 3:
[imu]        BNO055 VIN -> Pi pin 1  (3V3)
[imu]        BNO055 GND -> Pi pin 6  (GND)   <- without a shared ground I2C never settles
[imu]        BNO055 SDA -> Pi pin 3  (GPIO 2)
[imu]        BNO055 SCL -> Pi pin 5  (GPIO 3)

[imu]      Otherwise: not plugged in, no power (is the board's LED on?),
[imu]      SDA/SCL swapped, or ground not shared. Re-seat and re-run."
fi

if [ "$FOUND" -eq 0 ]; then
    fail "devices answered, but none at 0x28 or 0x29" \
"something else is on the bus but not the BNO055. Check SDA/SCL are not
[imu]      swapped, and that the board is powered."
fi
pass "a BNO055 answered on the bus"

STEP=5; echo "[imu] 5/6 are the Python libraries installed?"
python3 -c "import board, busio, adafruit_bno055" 2>/dev/null \
    && pass "board + adafruit_bno055 import" \
    || fail "the Adafruit libraries are missing" \
"pip3 install --break-system-packages adafruit-circuitpython-bno055 adafruit-blinka"

STEP=6; echo "[imu] 6/6 can we actually read it?"
python3 - "$ADDR_HEX" <<'PY'
import sys
import board, busio, adafruit_bno055
addr = int(sys.argv[1], 16)
s = adafruit_bno055.BNO055_I2C(busio.I2C(board.SCL, board.SDA), address=addr)
accel = s.linear_acceleration
calib = s.calibration_status
print(f'[imu]   OK  - linear_acceleration = {accel}')
print(f'[imu]   OK  - calibration (sys, gyro, accel, mag) = {calib}')
if accel is None or any(v is None for v in accel):
    print('[imu]   NOTE: a None reading means a disturbed I2C transaction. '
          'The driver skips those rather than publishing NaN.')
if calib and calib[2] < 2:
    print(f'[imu]   NOTE: accelerometer calibration is {calib[2]}/3. Rest the '
          'robot on a few different faces to raise it. The stall guard uses '
          'the accelerometer, so this one matters.')
PY
[ $? -eq 0 ] || fail "the board is on the bus but would not read" \
    "check the wiring is firm; a marginal connection scans but fails to read"

echo
echo "[imu] ============================================================"
echo "[imu]  The board is wired and readable. Now bring it up in ROS:"
echo "[imu]    ros2 launch nexva_sensor imu.launch.py"
echo "[imu]    ros2 topic hz /imu          # should be ~50 Hz"
echo "[imu]    ros2 run nexva_sensor imu_monitor"
echo "[imu]"
echo "[imu]  BENCH CHECK, before trusting the stall guard: turning the robot"
echo "[imu]  LEFT must make yaw INCREASE. If it does not, fix axis_remap -"
echo "[imu]  the guard integrates FORWARD acceleration, so a board mounted"
echo "[imu]  the wrong way makes every one of its thresholds meaningless."
echo "[imu] ============================================================"
