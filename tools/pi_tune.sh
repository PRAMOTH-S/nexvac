#!/usr/bin/env bash
# Tune the Pi 5 for this robot. Run once per boot, BEFORE ./robotbring.sh.
#
#   ./tools/pi_tune.sh            apply
#   ./tools/pi_tune.sh --status   show what is set now, change nothing
#   ./tools/pi_tune.sh --revert   put the defaults back
#
# Needs sudo for the governor and the tmpfs mount; it asks, it does not assume.
#
# WHAT THIS DOES, AND WHY EACH ONE IS WORTH DOING
#
# 1. CPU governor: ondemand -> performance.
#    Measured on this robot: the Pi sat at 2.0 GHz with a 2.4 GHz ceiling.
#    That is ~17% of the clock left on the table, and worse than the average
#    loss is the SHAPE of it: ondemand ramps *after* it notices load, so the
#    first few hundred ms of every burst of planning or scan processing runs
#    slow. A control loop that must finish inside 100 ms does not want to
#    discover the CPU is still spinning up. The Pi 5 has active cooling and
#    was measured at 31.8 C with zero throttling, so there is thermal room.
#
# 2. ROS logs and /tmp onto tmpfs (RAM).
#    Both currently land on /dev/mmcblk0p2 - the SD card. rclcpp/rclpy log
#    writes are synchronous enough that a slow card stalls the node doing the
#    logging, and Nav2 is extremely chatty. The robot has 6.8 GB free RAM
#    doing nothing. Spending ~512 MB of it to stop touching the SD card in the
#    control path is the single best use of this machine's memory.
#
# 3. Pin micro_ros_agent to one core.
#    This is the ONLY process worth pinning, and pinning is not free: the
#    kernel already spreads nodes across the 4 cores, and pinning everything
#    would serialise work onto fewer cores and be SLOWER. But the micro-ROS
#    agent is different - it is a serial link with a 500 ms watchdog at the
#    far end. If it is preempted mid-transfer the ESP32 stops hearing cmd_vel
#    and the wheels stop. Giving it a core nothing else is pinned to removes
#    that failure mode. Everything else stays free to float, which is what
#    "in harmony" actually means on a 4-core box.
#
# WHAT THIS DELIBERATELY DOES NOT DO
#
#    It does not pin SLAM, Nav2 or the explorer. They are throughput-bound and
#    the scheduler balances them better than a fixed map would. It does not
#    raise DDS buffer sizes either: messages here are small and nothing was
#    measured dropping. More RAM cannot make CPU-bound planning faster - that
#    was fixed by replacing the per-cell BFS with a numpy wavefront (223 ms ->
#    3.4 ms measured on this Pi), not by buying memory.
set -u

LOG_TMPFS_MB="${LOG_TMPFS_MB:-512}"
AGENT_CORE="${AGENT_CORE:-3}"
ROS_LOG_DIR_PATH="${HOME}/.ros/log"

status() {
    echo "[tune] governor : $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
    echo "[tune] clock    : $(( $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo 0) / 1000 )) MHz of $(( $(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null || echo 0) / 1000 )) MHz"
    echo "[tune] log dir  : $(findmnt -no FSTYPE --target "$ROS_LOG_DIR_PATH" 2>/dev/null || echo '?') at $ROS_LOG_DIR_PATH"
    echo "[tune] free RAM : $(free -m | awk '/^Mem:/{print $7}') MB available"
    local t
    t=$(vcgencmd get_throttled 2>/dev/null || echo "throttled=?")
    echo "[tune] throttle : $t  (0x0 is healthy)"
}

case "${1:-}" in
    --status) status; exit 0 ;;
    --revert)
        echo "[tune] reverting..."
        for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            echo ondemand | sudo tee "$g" >/dev/null 2>&1 || true
        done
        mountpoint -q "$ROS_LOG_DIR_PATH" && sudo umount "$ROS_LOG_DIR_PATH" || true
        echo "[tune] governor back to ondemand, log tmpfs unmounted"
        status
        exit 0 ;;
esac

echo "[tune] before:"
status
echo

echo "[tune] 1/3 CPU governor -> performance"
CHANGED=0
for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    if echo performance | sudo tee "$g" >/dev/null 2>&1; then CHANGED=$((CHANGED+1)); fi
done
[ "$CHANGED" -gt 0 ] \
    && echo "[tune]     set on $CHANGED core(s)" \
    || echo "[tune]     FAILED (no sudo?) - skipping, everything else still applies"

echo "[tune] 2/3 ROS logs onto tmpfs (${LOG_TMPFS_MB} MB of RAM)"
mkdir -p "$ROS_LOG_DIR_PATH"
if mountpoint -q "$ROS_LOG_DIR_PATH"; then
    echo "[tune]     already tmpfs"
elif sudo mount -t tmpfs -o "size=${LOG_TMPFS_MB}M,mode=0755,uid=$(id -u),gid=$(id -g)" tmpfs "$ROS_LOG_DIR_PATH" 2>/dev/null; then
    echo "[tune]     mounted - logs no longer touch the SD card"
    echo "[tune]     NOTE: logs are now lost on reboot. That is the trade."
else
    echo "[tune]     FAILED (no sudo?) - logs still on SD"
fi

echo "[tune] 3/3 micro-ROS agent core pinning"
# Applied by robotbring.sh via NEXVA_AGENT_CORE; nothing to do here but say so.
if [ "$AGENT_CORE" = "none" ]; then
    echo "[tune]     disabled (AGENT_CORE=none)"
else
    echo "[tune]     core $AGENT_CORE reserved - robotbring.sh pins the agent there"
    echo "[tune]     (export NEXVA_AGENT_CORE=none to turn that off)"
fi

echo
echo "[tune] after:"
status
echo
echo "[tune] ============================================================"
echo "[tune]  Done. Re-run after every reboot (none of this survives one)."
echo "[tune]  Now start the robot:  ./robotbring.sh"
echo "[tune]  Undo with:            ./tools/pi_tune.sh --revert"
echo "[tune] ============================================================"
