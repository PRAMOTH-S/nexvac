#!/usr/bin/env bash
# CLEAN mission: Nav2 with AMCL on a saved map + the coverage cleaner.
#
# No exploring. AMCL localises against the saved map (the tuned
# nav2_params.yaml, so the web UI's goals, waypoints and zones keep working),
# and frontier_explorer starts straight in `clean` mode to sweep the floor.
# The autosaver keeps a copy of the map under $MAP_DIR/$MAP_NAME.* and
# records it in ~/nexva_maps/map.md.
#
# Assumes bringup is already up (./robotbring.sh does that). Normally started by the
# mode manager, which hands over the map through the environment:
#
#   MAP_NAME   a map's name (resolved below), or
#   MAP        a map .yaml path, used as given
#   MAP_SOURCE 'hardware' - which ~/nexva_maps/ folder MAP_NAME is looked up in
#
# By hand:
#   MAP_NAME=kitchen ./launch/realbot/robot_clean_saved.sh
#   MAP=~/nexva_maps/hardware/kitchen.yaml ./launch/realbot/robot_clean_saved.sh
#   ./launch/realbot/robot_clean_saved.sh        the last map saved, per map.md
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
MAP_REGISTRY="${NEXVA_MAP_REGISTRY:-$MAPS_ROOT/map.md}"
MAP="${MAP:-}"
MAP_NAME="${MAP_NAME:-}"
BRINGUP_WAIT="${BRINGUP_WAIT:-10}"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[clean] ERROR: nothing built yet. Run:  ./build.sh"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# map_field <map.md> <source> <key> - one field out of one section of map.md.
map_field() {
    local file="$1" source="$2" key="$3"
    [ -f "$file" ] || return 1
    awk -v want="$source" -v key="$key" '
        /^##[ \t]+/ { section = $2; next }
        section == want {
            if ($0 ~ "^- \\*\\*" key ":\\*\\*") {
                sub("^- \\*\\*" key ":\\*\\*[ \t]*", "")
                print
                exit
            }
        }
    ' "$file"
}

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
        | grep -v '\.as_mapped\.yaml$' | sed 's/^/[clean]   /' || true
}

echo "[clean] 1/3 resolving the map..."
# MAP without a slash is a name, not a path.
if [ -n "$MAP" ] && [[ "$MAP" != */* ]]; then
    MAP_NAME="${MAP_NAME:-$MAP}"
    MAP=""
fi
if [ -z "$MAP" ] && [ -n "$MAP_NAME" ]; then
    if ! MAP="$(find_map "$MAP_NAME")"; then
        echo "[clean] ERROR: no map called \"$MAP_NAME\" in $MAP_DIR or src/nexva_navigation/maps"
        echo "[clean] available:"
        list_maps
        exit 1
    fi
fi
if [ -z "$MAP" ]; then
    # Nothing named: the last map this source saved, per map.md.
    MAP="$(map_field "$MAP_REGISTRY" "$MAP_SOURCE" map_file || true)"
    MAP_NAME="$(map_field "$MAP_REGISTRY" "$MAP_SOURCE" map_name || true)"
    if [ -z "$MAP" ]; then
        echo "[clean] ERROR: no map named and none recorded for \"$MAP_SOURCE\" in $MAP_REGISTRY."
        echo "[clean]   Build one first (explore or manual from the web UI), or name one:"
        echo "[clean]   MAP_NAME=sep23map2 $0"
        echo "[clean] available:"
        list_maps
        exit 1
    fi
    echo "[clean]     map.md: \"$MAP_NAME\" saved $(map_field "$MAP_REGISTRY" "$MAP_SOURCE" saved_at || echo '?')"
fi
if [ ! -f "$MAP" ]; then
    echo "[clean] ERROR: no map at $MAP"
    exit 1
fi
MAP_NAME="${MAP_NAME:-$(basename "${MAP%.yaml}")}"
echo "[clean]     map: $MAP"
echo "[clean]     saving as: $MAP_DIR/$MAP_NAME.{pgm,yaml}"
mkdir -p "$MAP_DIR"

# clean.launch.py reads these while building its description (for the
# .as_mapped backup); the launch arguments below carry them to the nodes.
export MAP MAP_NAME MAP_SOURCE MAP_DIR

echo "[clean] 2/3 checking bringup (/scan, /odom, TF, up to ${BRINGUP_WAIT}s)..."
# The same check robotbring.sh uses: one process, sensor QoS. The old
# `timeout 3 ros2 topic echo /scan --once` loop paid 1-2 s of Python start-up
# and DDS discovery per try - on a busy Pi that ran out the timeout while the
# lidar was publishing fine, and the mission refused to start.
if ! python3 "$WS/tools/wait_for_bringup.py" --timeout "$BRINGUP_WAIT"; then
    echo "[clean] ERROR: bringup is not fully up (see the [wait] lines above)."
    echo "[clean]   Start it with ./robotbring.sh; this mission starts no hardware."
    exit 1
fi
echo "[clean]     bringup live"

MISSION_PID=""
CLEANED=false
cleanup() {
    [ "$CLEANED" = true ] && return
    CLEANED=true
    echo
    echo "[clean] shutting down..."
    if [ -n "$MISSION_PID" ] && kill -0 "$MISSION_PID" 2>/dev/null; then
        kill -INT "$MISSION_PID" 2>/dev/null || true
        wait "$MISSION_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[clean] 3/3 starting Nav2 + AMCL on the saved map, then the cleaner..."
echo "[clean]     AMCL is seeded from ~/nexva_maps/$MAP_NAME.pose.yaml when valid; else seed it from the web UI"
echo
ros2 launch nexva_explore clean.launch.py \
    map:="$MAP" \
    map_name:="$MAP_NAME" \
    map_source:="$MAP_SOURCE" \
    map_dir:="$MAP_DIR" &
MISSION_PID=$!

wait "$MISSION_PID"
