#!/usr/bin/env bash
# Build the workspace.
#
#   ./build.sh                    incremental build of everything
#   ./build.sh nexva_web          just that package (and nothing it depends on)
#   ./build.sh nexva_web nexva_sensor
#   CLEAN=true ./build.sh         delete build/ install/ log/ first
#   TEST=true  ./build.sh         run the test suites afterwards
#   ./build.sh -- --parallel-workers 2     anything after -- goes to colcon
#
# Incremental by default. A full rebuild also recompiles the vendored
# micro-ROS Agent and the RPLIDAR driver, which are C++ and slow, and they
# change about once a year - so wiping is opt-in rather than automatic.
#
# --symlink-install is always on: the Python nodes are then edited in place
# and a rebuild is only needed when you add a file, change an entry point, or
# touch a launch/config file.
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

CLEAN="${CLEAN:-false}"
TEST="${TEST:-false}"
FORCE_ALL="${FORCE_ALL:-false}"

# Split "package names" from "extra colcon args" on a literal --
PACKAGES=()
EXTRA=()
SEEN_SEP=false
for arg in "$@"; do
    if [ "$arg" = "--" ]; then SEEN_SEP=true; continue; fi
    if [ "$SEEN_SEP" = true ]; then EXTRA+=("$arg"); else PACKAGES+=("$arg"); fi
done

source /opt/ros/jazzy/setup.bash

if [ "$CLEAN" = "true" ]; then
    echo "[build] CLEAN: removing build/ install/ log/"
    rm -rf build install log
fi

SELECT=()
if [ "${#PACKAGES[@]}" -gt 0 ]; then
    SELECT=(--packages-select "${PACKAGES[@]}")
    echo "[build] building: ${PACKAGES[*]}"
else
    echo "[build] building: everything in src/"
    if [ ! -d "$WS/install" ]; then
        echo "[build] first build - the micro-ROS Agent and RPLIDAR driver are"
        echo "[build] C++ and will take several minutes"
    fi
fi

# simluationsequnce/ is a whole second ROS workspace sitting in this tree. Left
# alone it gets crawled and built as part of this one, which fails on its
# missing deps and, worse, would install a second set of same-named packages.
# Scoped to src/ rather than the whole tree. simluationsequnce/ is a second,
# self-contained ROS workspace living inside this one; crawling it would build
# a duplicate set of packages here. A COLCON_IGNORE marker would also work, but
# it would sit at that workspace's own root and stop IT from building itself.
BASE=(--base-paths "$WS/src")

# CMake bakes absolute paths into CMakeCache.txt, so a build/ tree copied from
# another machine - or from the Pi, where this workspace lives at a different
# path - fails every C++ package with "the current CMakeCache.txt directory is
# different than the directory where CMakeCache.txt was created". The build
# directory is a regenerable artifact and is gitignored, so the fix is simply
# to drop the stale ones and let them rebuild.
if [ -d "$WS/build" ]; then
    STALE=()
    for cache in "$WS"/build/*/CMakeCache.txt; do
        [ -f "$cache" ] || continue
        if ! grep -q "CMAKE_HOME_DIRECTORY:INTERNAL=$WS/" "$cache" 2>/dev/null; then
            STALE+=("$(dirname "$cache")")
        fi
    done
    if [ "${#STALE[@]}" -gt 0 ]; then
        echo "[build] stale CMake cache from another path in ${#STALE[@]} package(s):"
        for dir in "${STALE[@]}"; do
            echo "[build]     $(basename "$dir")"
            rm -rf "$dir"
        done
        echo "[build] removed those build dirs; they will be regenerated"
    fi
fi

# This tree gets moved between the Pi (aarch64) and a laptop (x86_64), and
# the install/ directory comes along with it. A package whose prebuilt
# libraries are for the other architecture cannot be linked against here, and
# the failure is opaque: "Relocations in generic ELF (EM: 183)".
#
# micro_ros_agent is the one that matters. It builds its upstream agent with
# ExternalProject_Add + GIT_REPOSITORY, so rebuilding it needs the network,
# and its aarch64 build is what the Pi actually runs - deleting that from a
# laptop would break the robot's copy for no gain, since only the Pi ever
# talks to the ESP32. So skip it rather than rebuild or remove it.
SKIP=()
if [ "$FORCE_ALL" != "true" ] && [ -d "$WS/install" ]; then
    case "$(uname -m)" in
        x86_64)  WRONG='ARM aarch64' ;;
        aarch64) WRONG='x86-64' ;;
        *)       WRONG='' ;;
    esac

    if [ -n "$WRONG" ]; then
        for pkgdir in "$WS"/install/*/; do
            pkg="$(basename "$pkgdir")"
            lib="$(find "$pkgdir" -maxdepth 2 -name '*.so' -print -quit 2>/dev/null)"
            [ -n "$lib" ] || continue
            if file -b "$lib" 2>/dev/null | grep -q "$WRONG"; then
                SKIP+=("$pkg")
            fi
        done
    fi

    if [ "${#SKIP[@]}" -gt 0 ]; then
        echo "[build] skipping ${#SKIP[@]} package(s) built for another architecture:"
        for pkg in "${SKIP[@]}"; do
            echo "[build]     $pkg  (prebuilt for $WRONG, this machine is $(uname -m))"
        done
        echo "[build] they are kept as-is. To rebuild them here anyway:"
        echo "[build]     FORCE_ALL=true CLEAN=true ./build.sh    (needs network)"
    fi
fi

SKIPARG=()
if [ "${#SKIP[@]}" -gt 0 ]; then
    SKIPARG=(--packages-skip "${SKIP[@]}")
fi

colcon build --symlink-install "${BASE[@]}" "${SELECT[@]}" "${SKIPARG[@]}" "${EXTRA[@]}"

if [ "$TEST" = "true" ]; then
    echo
    echo "[build] running tests..."
    colcon test "${SELECT[@]}"
    colcon test-result --verbose
fi

echo
echo "[build] done. In each new terminal:"
echo "[build]     source $WS/install/setup.bash"
echo "[build] or just use ./robotbring.sh, ./robotnav.sh and ./web.sh, which"
echo "[build] source it themselves."
