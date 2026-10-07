#!/usr/bin/env bash
# Split in two. Use ./robotbring.sh and ./robotnav.sh instead.
#
# This script started bringup and then, in the SAME terminal, waited for a
# readiness check before starting Nav2. The check shelled out to
# `ros2 topic echo --once` and `tf2_echo` on a 3 s timeout. Those need 1-2 s
# just to start a Python process and finish DDS discovery - longer on a loaded
# Pi - so they timed out on a perfectly healthy robot. And `ros2 topic echo`
# subscribes RELIABLE while the RPLIDAR publishes /scan BEST_EFFORT, which
# never connects at all. The result was bringup running fine and Nav2 never
# starting, with no obvious reason why.
#
# The check now lives in tools/wait_for_bringup.py: one process, correct QoS,
# no dependence on the ros2 daemon. And the two halves are separate terminals,
# so a check that is wrong can no longer stop you from starting Nav2 by hand.
#
#   terminal 1:  ./robotbring.sh
#   terminal 2:  ./robotnav.sh
#   terminal 3:  ./web.sh
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cat <<EOF
[robot] This script has been split in two.

[robot]   terminal 1:  ./robotbring.sh     bringup: ESP32, odometry, lidar
[robot]   terminal 2:  ./robotnav.sh       Nav2 + AMCL, mode switching
[robot]   terminal 3:  ./web.sh            browser control panel

[robot] Same arguments as before, now on the half that uses them:
[robot]   MAP=sep23map1 ./robotnav.sh
[robot]   SLAM=true ./robotnav.sh
[robot]   WAIT=120 ./robotbring.sh

[robot] Why: the old readiness check timed out on a healthy robot and Nav2
[robot] then never started. See the comments at the top of this file.
EOF

exit 1
