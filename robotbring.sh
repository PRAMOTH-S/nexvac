#!/usr/bin/env bash
# Half 1 of 2: bringup only - robot model, ESP32, odometry, lidar.
#
#   ./robotbring.sh
#   WAIT=120 ./robotbring.sh      allow longer for the lidar to spin up
#
# Leave this running. Then, in a SECOND terminal:
#
#   ./robotnav.sh                 Nav2 + AMCL (and the web UI's mode switching)
#   ./web.sh                      the browser control panel
#
# This used to be step 1 of robot.sh, which then started the mode manager in
# the same terminal once its readiness check passed. The check was the problem,
# not the sequencing: it shelled out to `ros2 topic echo --once` and
# `tf2_echo` on a 3 s timeout, and those take 1-2 s just to start a Python
# process and finish DDS discovery - longer on a loaded Pi - so they timed out
# while the robot was perfectly healthy. Worse, `ros2 topic echo` subscribes
# RELIABLE and the RPLIDAR publishes /scan BEST_EFFORT, which never connects at
# all. Bringup looked up, and nothing ever started on top of it.
#
# Split in two so the two halves can be restarted independently: Nav2 can be
# bounced without power-cycling the ESP32 link, and a failed readiness check
# can no longer stop Nav2 from being started by hand.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

WAIT="${WAIT:-60}"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[bringup] ERROR: nothing built yet. Run:  ./build.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# The mode manager and the mission scripts find the workspace through this.
# Exported here too so a shell that only ever ran this script still has it.
export NEXVA_WS="$WS"

BRINGUP_PID=""

cleanup() {
    echo
    echo "[bringup] shutting down..."
    if [ -n "$BRINGUP_PID" ]; then
        kill "$BRINGUP_PID" 2>/dev/null || true
        wait "$BRINGUP_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[bringup] 1/2 starting bringup (robot model, ESP32, odometry, lidar)..."
ros2 launch nexva_bringup bringup.launch.py &
BRINGUP_PID=$!

echo "[bringup] 2/2 waiting for it to come up (up to ${WAIT}s)..."
echo "[bringup]     the ESP32 is reset and the agent restarted, so the first"
echo "[bringup]     few seconds are expected to be quiet"

# One rclpy process watching all three at once, with sensor QoS so the
# BEST_EFFORT lidar is actually received. See tools/wait_for_bringup.py.
if python3 "$WS/tools/wait_for_bringup.py" --timeout "$WAIT"; then
    READY=true
else
    READY=false
fi

echo
if [ "$READY" = true ]; then
    echo "[bringup] ============================================================"
    echo "[bringup]  BRINGUP READY - leave this terminal running."
    echo "[bringup]"
    echo "[bringup]  Next, in another terminal:   ./robotnav.sh"
    echo "[bringup]  Then the web UI:             ./web.sh"
    echo "[bringup] ============================================================"
else
    echo "[bringup] ============================================================"
    echo "[bringup]  NOT READY after ${WAIT}s - see the reasons above."
    echo "[bringup]"
    echo "[bringup]  Bringup is STILL RUNNING and was not killed, so you can"
    echo "[bringup]  read its output above and fix the hardware without"
    echo "[bringup]  restarting the ESP32 link. If it is actually fine and the"
    echo "[bringup]  wait was just slow, start the other half anyway:"
    echo "[bringup]      ./robotnav.sh"
    echo "[bringup]  or retry the check alone:"
    echo "[bringup]      python3 tools/wait_for_bringup.py --timeout 120"
    echo "[bringup] ============================================================"
fi
echo

# Stay up either way: this terminal owns bringup, and killing it on a failed
# check would take the ESP32 and lidar down with it for no good reason.
wait "$BRINGUP_PID"
