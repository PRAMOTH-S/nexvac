#!/usr/bin/env bash
# Half 2 of 2: the mode manager - Nav2 + AMCL on a saved map, and the web UI's
# mode switching. Run ./robotbring.sh in another terminal FIRST.
#
#   ./robotnav.sh
#   MAP=sep23map1 ./robotnav.sh
#   MAP=/abs/path/to/other.yaml ./robotnav.sh
#   SLAM=true ./robotnav.sh          map by hand instead of localising
#   FORCE=true ./robotnav.sh         start even if the bringup check fails
#   WAIT=20 ./robotnav.sh            how long to wait for bringup before giving up
#
# The mode manager (nexva_explore) owns whatever runs on top of bringup: it
# starts the `navigate` mission (Nav2 + AMCL on MAP) as soon as it is up, and
# the web UI can then switch it to explore / clean / manual / stop. One mission
# at a time; the previous one is stopped, and the wheels halted, before the
# next starts. The missions are launch/realbot/robot_*.sh. SLAM=true asks for
# `manual` (slam_toolbox + map autosave, drive from the web joystick) instead.
#
# Starting Nav2 before bringup is up is the failure that looks like "Nav2 is
# broken": AMCL comes up with no scan to match and no transform to anchor, logs
# nothing obviously wrong, and never publishes map -> odom. So this checks
# first - but unlike the old combined script, a failed check only warns and
# FORCE=true overrides it. The check being wrong must never be the thing that
# stops you starting Nav2.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

MAP="${MAP:-sep23map2}"
SLAM="${SLAM:-false}"
WAIT="${WAIT:-20}"
FORCE="${FORCE:-false}"

# A bare name means one of our own maps; anything with a / is a path as given.
if [[ "$MAP" != */* ]]; then
    MAP="$WS/src/nexva_navigation/maps/$MAP.yaml"
fi

if [ "$SLAM" != "true" ] && [ ! -f "$MAP" ]; then
    echo "[nav] ERROR: no map at $MAP"
    echo "[nav] available:"
    ls -1 "$WS/src/nexva_navigation/maps/"*.yaml 2>/dev/null | sed 's/^/[nav]   /'
    echo "[nav] or run without a map:  SLAM=true ./robotnav.sh"
    exit 1
fi

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[nav] ERROR: nothing built yet. Run:  ./build.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# The mode manager and the mission scripts find the workspace through this.
export NEXVA_WS="$WS"

MANAGER_PID=""

cleanup() {
    echo
    echo "[nav] shutting down..."
    # SIGINT, not SIGTERM: the manager stops the running mission and halts the
    # wheels itself on the way out, and that takes a moment. Bringup is in the
    # other terminal and is deliberately left alone.
    if [ -n "$MANAGER_PID" ] && kill -0 "$MANAGER_PID" 2>/dev/null; then
        kill -INT "$MANAGER_PID" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 "$MANAGER_PID" 2>/dev/null || break
            sleep 1
        done
        kill "$MANAGER_PID" 2>/dev/null || true
    fi
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[nav] 1/2 checking bringup is up (up to ${WAIT}s)..."
if python3 "$WS/tools/wait_for_bringup.py" --timeout "$WAIT"; then
    echo "[nav]     bringup OK"
elif [ "$FORCE" = "true" ]; then
    echo "[nav]     check failed, but FORCE=true - starting anyway"
else
    echo
    echo "[nav] ERROR: bringup does not look up. Nav2 NOT started."
    echo "[nav]   Start it first, in another terminal:  ./robotbring.sh"
    echo "[nav]   If it IS running and the check is simply slow or wrong:"
    echo "[nav]       FORCE=true ./robotnav.sh"
    exit 1
fi

echo
echo "[nav] 2/2 starting the mode manager..."
if [ "$SLAM" = "true" ]; then
    echo "[nav]     initial mode: manual (slam_toolbox + map autosave)"
    ros2 launch nexva_explore mode_manager.launch.py initial_mode:=manual &
else
    echo "[nav]     initial mode: navigate (Nav2 + AMCL on $MAP)"
    ros2 launch nexva_explore mode_manager.launch.py \
        initial_mode:=navigate initial_map:="$MAP" &
fi
MANAGER_PID=$!

echo
echo "[nav] ============================================================"
echo "[nav]  READY. Start the web UI in another terminal:  ./web.sh"
echo "[nav]  Seed the initial pose there before sending any goal."
echo "[nav]  The web UI can switch modes: navigate / explore / clean /"
echo "[nav]  manual / stop (topic: set_robot_mode, state: robot_mode)."
echo "[nav]  Ctrl-C here stops the mission and the manager, and LEAVES"
echo "[nav]  bringup running in the other terminal."
echo "[nav] ============================================================"
echo

wait "$MANAGER_PID"
