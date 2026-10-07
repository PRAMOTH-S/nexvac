#!/usr/bin/env bash
# EXPLORE mission: slam_toolbox + on-demand map saver + pose store + frontier explorer.
#
# The robot drives itself: it looks at the live map, finds the borders
# between known free space and unknown space, plans a path to the nearest
# one, and repeats until nothing unknown is reachable - then sweeps what it
# mapped. The map is NOT written continuously any more. It is saved on demand
# (web UI "Save map", or automatically when exploration finishes) to
# $MAP_DIR/$MAP_NAME.{pgm,yaml} with the robot's pose in
# ~/nexva_maps/$MAP_NAME.pose.yaml, and recorded in ~/nexva_maps/map.md.
#
# Assumes bringup is already up (./robotbring.sh does that). Normally started by the
# mode manager, which hands over the map through the environment:
#
#   MAP_NAME   name for the NEW map (default: explore)
#   MAP_SOURCE 'hardware' - which ~/nexva_maps/ folder it is saved in
#
# By hand:
#   MAP_NAME=kitchen ./launch/realbot/robot_explore.sh
set -e

# Two levels up: this script lives in launch/realbot/, the workspace root is
# where build.sh, install/ and src/ are.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"

MAP_SOURCE="${MAP_SOURCE:-hardware}"
MAPS_ROOT="${NEXVA_MAPS:-$HOME/nexva_maps}"
MAP_DIR="${MAP_DIR:-$MAPS_ROOT/$MAP_SOURCE}"
MAP_NAME="${MAP_NAME:-explore}"
BRINGUP_WAIT="${BRINGUP_WAIT:-10}"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[explore] ERROR: nothing built yet. Run:  ./build.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "[explore] 1/3 map will be saved on demand to: $MAP_DIR/$MAP_NAME.{pgm,yaml}"
mkdir -p "$MAP_DIR"

echo "[explore] 2/3 checking bringup (/scan, /odom, TF, up to ${BRINGUP_WAIT}s)..."
# The same check robotbring.sh uses: one process, sensor QoS. The old
# `timeout 3 ros2 topic echo /scan --once` loop paid 1-2 s of Python start-up
# and DDS discovery per try - on a busy Pi that ran out the timeout while the
# lidar was publishing fine, and the mission refused to start.
if ! python3 "$WS/tools/wait_for_bringup.py" --timeout "$BRINGUP_WAIT"; then
    echo "[explore] ERROR: bringup is not fully up (see the [wait] lines above)."
    echo "[explore]   Start it with ./robotbring.sh; this mission starts no hardware."
    exit 1
fi
echo "[explore]     bringup live"

MISSION_PID=""
CLEANED=false
cleanup() {
    [ "$CLEANED" = true ] && return
    CLEANED=true
    echo
    echo "[explore] shutting down..."
    if [ -n "$MISSION_PID" ] && kill -0 "$MISSION_PID" 2>/dev/null; then
        kill -INT "$MISSION_PID" 2>/dev/null || true
        wait "$MISSION_PID" 2>/dev/null || true
    fi
    echo "[explore] map (if saved) in $MAP_DIR/$MAP_NAME.{pgm,yaml}"
}
trap cleanup EXIT INT TERM

echo "[explore] 3/3 starting SLAM + map saver + pose store + frontier explorer..."
echo "[explore]     the explorer LOCALIZES FIRST: it holds still, publishing a zero"
echo "[explore]     cmd_vel, until map -> base_footprint is live and fresh and a /map"
echo "[explore]     has arrived. Watch for 'Localizing: ...' then 'Localized after Ns'."
echo "[explore]     It gives up with an error rather than driving blind; the gate is"
echo "[explore]     inside frontier_explorer, so nothing can start the wheels early."
echo "[explore]     Then it explores on its own; stop it from the web UI"
echo
ros2 launch nexva_explore explore.launch.py \
    map_source:="$MAP_SOURCE" \
    map_dir:="$MAP_DIR" \
    map_name:="$MAP_NAME" &
MISSION_PID=$!

wait "$MISSION_PID"
