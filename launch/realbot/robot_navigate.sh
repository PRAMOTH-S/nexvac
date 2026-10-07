#!/usr/bin/env bash
# NAVIGATE mission: Nav2 with AMCL on a saved map. Exactly the navigation
# half that robot.sh used to start itself; it now starts here so the mode
# manager can swap it for explore/clean/manual and back.
#
# Assumes bringup is already up (./robotbring.sh does that). Normally started by the
# mode manager, which hands over the map through the environment:
#
#   MAP_NAME   a map's name (resolved below), or
#   MAP        a map .yaml path, used as given
#   MAP_SOURCE 'hardware' - which ~/nexva_maps/ folder MAP_NAME is looked up in
#
# By hand:
#   ./launch/realbot/robot_navigate.sh                  sep23map2
#   MAP_NAME=sep23map1 ./launch/realbot/robot_navigate.sh
#   MAP=/abs/path/to/other.yaml ./launch/realbot/robot_navigate.sh
#
# A bare name is looked for in ~/nexva_maps/$MAP_SOURCE/ first (maps the
# robot built itself), then in src/nexva_navigation/maps/ (the ones that
# shipped with the workspace).
set -e

# Two levels up: this script lives in launch/realbot/, the workspace root is
# where build.sh, install/ and src/ are.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"

MAP_SOURCE="${MAP_SOURCE:-hardware}"
MAPS_ROOT="${NEXVA_MAPS:-$HOME/nexva_maps}"
MAP_DIR="${MAP_DIR:-$MAPS_ROOT/$MAP_SOURCE}"
MAP="${MAP:-}"
MAP_NAME="${MAP_NAME:-}"
DEFAULT_MAP_NAME="sep23map2"
BRINGUP_WAIT="${BRINGUP_WAIT:-10}"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[navigate] ERROR: nothing built yet. Run:  ./build.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# A map by name: the robot's own autosaved maps first, then the ones that
# shipped with nexva_navigation.
find_map() {
    local name="$1" candidate
    for candidate in "$MAP_DIR/$name.yaml" "$WS/src/nexva_navigation/maps/$name.yaml"; do
        if [ -f "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done
    return 1
}

list_maps() {
    ls -1 "$MAP_DIR"/*.yaml "$WS/src/nexva_navigation/maps/"*.yaml 2>/dev/null \
        | grep -v '\.as_mapped\.yaml$' | sed 's/^/[navigate]   /' || true
}

echo "[navigate] 1/3 resolving the map..."
# MAP without a slash is a name, not a path.
if [ -n "$MAP" ] && [[ "$MAP" != */* ]]; then
    MAP_NAME="${MAP_NAME:-$MAP}"
    MAP=""
fi
if [ -z "$MAP" ]; then
    MAP_NAME="${MAP_NAME:-$DEFAULT_MAP_NAME}"
    if ! MAP="$(find_map "$MAP_NAME")"; then
        echo "[navigate] ERROR: no map called \"$MAP_NAME\" in $MAP_DIR or src/nexva_navigation/maps"
        echo "[navigate] available:"
        list_maps
        exit 1
    fi
fi
if [ ! -f "$MAP" ]; then
    echo "[navigate] ERROR: no map at $MAP"
    exit 1
fi
MAP_NAME="${MAP_NAME:-$(basename "${MAP%.yaml}")}"
echo "[navigate]     map: $MAP"

echo "[navigate] 2/3 checking bringup (/scan, /odom, TF, up to ${BRINGUP_WAIT}s)..."
# The same check robotbring.sh uses: one process, sensor QoS. The old
# `timeout 3 ros2 topic echo /scan --once` loop paid 1-2 s of Python start-up
# and DDS discovery per try - on a busy Pi that ran out the timeout while the
# lidar was publishing fine, and the mission refused to start.
if ! python3 "$WS/tools/wait_for_bringup.py" --timeout "$BRINGUP_WAIT"; then
    echo "[navigate] ERROR: bringup is not fully up (see the [wait] lines above)."
    echo "[navigate]   Start it with ./robotbring.sh; this mission starts no hardware."
    exit 1
fi
echo "[navigate]     bringup live"

MISSION_PID=""
ZONE_PID=""
CLEANED=false
cleanup() {
    [ "$CLEANED" = true ] && return
    CLEANED=true
    echo
    echo "[navigate] shutting down..."
    # zone_coverage first: it is the one that can still be publishing cmd_vel.
    if [ -n "$ZONE_PID" ] && kill -0 "$ZONE_PID" 2>/dev/null; then
        kill -INT "$ZONE_PID" 2>/dev/null || true
        wait "$ZONE_PID" 2>/dev/null || true
    fi
    if [ -n "$MISSION_PID" ] && kill -0 "$MISSION_PID" 2>/dev/null; then
        kill -INT "$MISSION_PID" 2>/dev/null || true
        wait "$MISSION_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[navigate] 3/3 starting Nav2 + AMCL + zone coverage..."
echo "[navigate]     seed the initial pose from the web UI before sending a goal"
echo
ros2 launch nexva_navigation navigation.launch.py map:="$MAP" &
MISSION_PID=$!

# The web UI's "Vacuum zone" needs this node. Nothing started it before, so a
# drawn zone was published to a topic with no subscriber: the page said "sent"
# and the robot never moved.
#
# Safe to leave running here - it is idle until a zone arrives on
# /zone_coverage/zone. Deliberately NOT started in the clean mission, where
# auto_clean already owns the base; two planners publishing cmd_vel at each
# other is exactly the fight mode_manager exists to prevent.
sleep 8
ros2 run nexva_coverage zone_coverage --ros-args \
    --params-file "$WS/src/nexva_coverage/config/zone_coverage.yaml" \
    -p use_sim_time:=false &
ZONE_PID=$!
echo "[navigate]     zone_coverage up (draw a zone in the web UI to use it)"

wait "$MISSION_PID"
