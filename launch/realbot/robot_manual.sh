#!/usr/bin/env bash
# MANUAL mission: slam_toolbox + on-demand map saver, nothing that drives.
#
# Drive from the web joystick while it maps. Use this before explore: driving
# by hand is how you find out whether the lidar, the motors and the encoders
# are all actually working, without the robot deciding to go somewhere on its
# own. The map is NOT autosaved: press "Save map" in the web UI when you are
# done and it lands in $MAP_DIR/$MAP_NAME.{pgm,yaml}, recorded in
# ~/nexva_maps/map.md.
#
# Assumes bringup is already up (./robotbring.sh does that). Normally started by the
# mode manager, which hands over the map through the environment:
#
#   MAP_NAME   name for the map (default: manual)
#   MAP_SOURCE 'hardware' - which ~/nexva_maps/ folder it is saved in
#
# By hand:
#   MAP_NAME=kitchen ./launch/realbot/robot_manual.sh
set -e

# Two levels up: this script lives in launch/realbot/, the workspace root is
# where build.sh, install/ and src/ are.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"

MAP_SOURCE="${MAP_SOURCE:-hardware}"
MAPS_ROOT="${NEXVA_MAPS:-$HOME/nexva_maps}"
MAP_DIR="${MAP_DIR:-$MAPS_ROOT/$MAP_SOURCE}"
MAP_NAME="${MAP_NAME:-manual}"
BRINGUP_WAIT="${BRINGUP_WAIT:-10}"
SLAM_WAIT="${SLAM_WAIT:-60}"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[manual] ERROR: nothing built yet. Run:  ./build.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

echo "[manual] 1/3 map will be saved to: $MAP_DIR/$MAP_NAME.{pgm,yaml} (on demand)"
mkdir -p "$MAP_DIR"

echo "[manual] 2/3 checking bringup (/scan, /odom, TF, up to ${BRINGUP_WAIT}s)..."
# The same check robotbring.sh uses: one process, sensor QoS. The old
# `timeout 3 ros2 topic echo /scan --once` loop paid 1-2 s of Python start-up
# and DDS discovery per try - on a busy Pi that ran out the timeout while the
# lidar was publishing fine, and the mission refused to start.
if ! python3 "$WS/tools/wait_for_bringup.py" --timeout "$BRINGUP_WAIT"; then
    echo "[manual] ERROR: bringup is not fully up (see the [wait] lines above)."
    echo "[manual]   Start it with ./robotbring.sh; this mission starts no hardware."
    exit 1
fi
echo "[manual]     bringup live"

MISSION_PID=""
NAV_PID=""
CLEANED=false
cleanup() {
    [ "$CLEANED" = true ] && return
    CLEANED=true
    echo
    echo "[manual] shutting down..."
    if [ -n "$NAV_PID" ] && kill -0 "$NAV_PID" 2>/dev/null; then
        kill -INT "$NAV_PID" 2>/dev/null || true
        wait "$NAV_PID" 2>/dev/null || true
    fi
    if [ -n "$MISSION_PID" ] && kill -0 "$MISSION_PID" 2>/dev/null; then
        kill -INT "$MISSION_PID" 2>/dev/null || true
        wait "$MISSION_PID" 2>/dev/null || true
    fi
    echo "[manual] map is only on disk if you pressed Save map: $MAP_DIR/$MAP_NAME.{pgm,yaml}"
}
trap cleanup EXIT INT TERM

echo "[manual] 3/3 starting SLAM + map saver + Nav2..."
echo "[manual]     drive from the web UI joystick; press Save map when done (no autosave)"
echo
ros2 launch nexva_explore manual.launch.py \
    map_source:="$MAP_SOURCE" \
    map_dir:="$MAP_DIR" \
    map_name:="$MAP_NAME" &
MISSION_PID=$!

# Nav2 on top of the live SLAM map, so goals / tours / zones work WHILE
# mapping by hand - "Nav2 must be working always".
#
# BOTH arguments matter. navigation.launch.py includes slam_launch.py when
# (slam and use_localization), and localization_launch.py when
# (not slam and use_localization). slam:=true alone would therefore start a
# SECOND slam_toolbox, and the two would fight over map->odom - the exact
# failure mode_manager exists to prevent. use_localization:=false suppresses
# both branches, leaving only navigation_launch.py: planner, controller, BT
# navigator and waypoint_follower, planning against the map and TF this
# mission's own slam_toolbox already publishes. No AMCL either, which is
# correct - SLAM owns the pose here.
#
# Held back until SLAM has ACTUALLY localized, not for a guessed 10 s. The
# old `sleep 10` had the same fault as the explorer's TimerAction: on a cold
# Pi 5 slam_toolbox can take longer, and Nav2 coming up against a map frame
# that does not exist yet means every goal fails to plan until something
# restarts. tools/wait_for_slam.py waits for /map AND a map -> base_footprint
# transform whose stamp is fresh (a lookup at Time() happily returns the last
# transform of a dead publisher forever, so the age is what proves it live).
echo "[manual]     waiting for SLAM to localize before starting Nav2 (up to ${SLAM_WAIT}s)..."
if ! python3 "$WS/tools/wait_for_slam.py" --timeout "$SLAM_WAIT"; then
    echo "[manual] ERROR: SLAM never localized - NOT starting Nav2."
    echo "[manual]   Mapping continues; drive by hand and check the slam_toolbox output."
else
    ros2 launch nexva_navigation navigation.launch.py slam:=True use_localization:=False &
    NAV_PID=$!
    echo "[manual]     Nav2 up on the live map - goals and zones available"
fi

wait "$MISSION_PID"
