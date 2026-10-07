#!/usr/bin/env bash
# The web UI. Run this after ./robotbring.sh and ./robotnav.sh are up.
#
#   ./web.sh
#   PORT=8081 ./web.sh
#   WAYPOINTS=/abs/path/to/other.yaml ./web.sh
#   NOWAIT=true ./web.sh        start immediately, do not wait for Nav2
#
# The page itself comes up either way. Waiting for map_server first is only so
# the map, the waypoint check and the zone canvas work on the first load
# instead of erroring until Nav2 catches up.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
WAIT="${WAIT:-30}"
NOWAIT="${NOWAIT:-false}"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[web] ERROR: nothing built yet. Run:  colcon build"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

# aiohttp is the one dependency that is not a ROS package, and its absence
# shows up as a bare ModuleNotFoundError from inside a launch file.
if ! python3 -c "import aiohttp" 2>/dev/null; then
    echo "[web] ERROR: aiohttp is not installed - the bridge cannot start."
    echo "[web]   sudo apt install -y python3-aiohttp"
    echo "[web]   (or) pip3 install --break-system-packages aiohttp"
    exit 1
fi

if [ "$NOWAIT" != "true" ]; then
    echo "[web] waiting for Nav2's map_server (up to ${WAIT}s)..."
    DEADLINE=$(( SECONDS + WAIT ))
    READY=false
    while [ "$SECONDS" -lt "$DEADLINE" ]; do
        if ros2 node list 2>/dev/null | grep -q '/map_server'; then
            READY=true
            break
        fi
        sleep 1
    done

    if [ "$READY" = true ]; then
        echo "[web]     map_server is up"
    else
        echo "[web]     map_server never appeared - starting anyway."
        echo "[web]     The page will load but the map and waypoint check will"
        echo "[web]     fail until ./robotbring.sh and ./robotnav.sh are running."
    fi
fi

# The address to actually type, rather than 0.0.0.0.
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
echo "[web] ============================================================"
echo "[web]  http://${IP:-<this-machine>}:${PORT}"
echo "[web]  Ctrl-C to stop."
echo "[web] ============================================================"
echo

ARGS=(host:="$HOST" port:="$PORT")
if [ -n "${WAYPOINTS:-}" ]; then
    ARGS+=(waypoints:="$WAYPOINTS")
fi

exec ros2 launch nexva_web web.launch.py "${ARGS[@]}"
