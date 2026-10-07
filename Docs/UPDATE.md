# Update log

One entry per update pushed to the robot. Newest first.

This is the shared history for a workspace **two people work on**, so every
entry says **who** made it. If you pull or push and something behaves
differently from yesterday, the answer should be here.

**Keep entries short.** What changed, who changed it, what the other person has
to do about it. The deep reasoning, measurements and failed approaches belong in
[BUILD_LOG.md](BUILD_LOG.md); this file is the "what do I need to know" summary.

## How to add an entry

Copy this block to the top of the list and fill it in:

```
## YYYY-MM-DD — <one line summary>
**By:** <your name>
**Changed:** <packages / files>
**You need to:** <rebuild? re-run something? nothing?>
<2-4 lines on what is different now>
```

---

## 2026-10-07 — One-command push to GitHub (`gitpush.sh`)
**By:** Pramoth (with Claude)
**Changed:** `gitpush.sh` (new), `.gitignore`
**You need to:** nothing. To use it: `./gitpush.sh` (needs `gh auth login` once).

- `./gitpush.sh` lists what changed, asks "What did you change today?", uses your
  answer as the commit message, and pushes to `github.com/PRAMOTH-S/nexva_ws`
  (private). Passing the message as an argument skips the question. `--dry` shows what would go up. Not the same as `push.sh` (that one is laptop → Pi).
- Never force-pushes. If GitHub has newer commits, it rebases first, and on a conflict it stops.
- `.gitignore` now also excludes `push.sh`'s `.push_history` / `.last_push` / `.push_by`.

## 2026-10-06 — Map picker, map delete, localize-before-explore, safer push
**By:** Varun (with Claude)
**Changed:** `nexva_web`, `nexva_explore`, `nexva_sensor`, `push.sh`, `tools/`
**You need to:** `./push.sh` then restart bringup. Nothing on your side breaks.

- **Web UI:** CLEAN / NAVIGATE / MANUAL now pick the map from **buttons in a
  popup** instead of typing the name. Saved maps can be **deleted** from the
  page (bundled `sep23map1/2` are refused — they are source files).
- The **localization pill** now reads `localized (SLAM)` or `localized (AMCL)`
  instead of wrongly showing "not localized" during explore, where there is no
  AMCL at all.
- **Explore waits for a real pose** before it drives, instead of a fixed 8 s
  sleep that could start the explorer before SLAM had a transform. It now holds
  station publishing zero velocity, logs `Localizing (Ns/60s): <what is
  missing>`, and reports mode `localizing` until `map -> base_footprint` is
  live **and its stamp is fresh** — a dead publisher's cached transform no
  longer counts as localized. `localize_timeout` (60 s) ends it with a clear
  error rather than driving blind. `robot_manual.sh` had the same `sleep 10`
  before Nav2; also replaced.
- **`push.sh` is now safe for two people** — see the next entry.

## 2026-10-06 — push.sh: incremental and shared-workspace safe
**By:** Varun (with Claude)
**Changed:** `push.sh`
**You need to:** nothing — the new defaults protect your work.

- Default now syncs with `--update`: **a file newer on the Pi is left alone**,
  so a push cannot silently overwrite edits you made on the robot. `--mine`
  overrides when that is actually intended.
- Only the **packages that changed** are rebuilt (~40 s → seconds). `--full`
  after touching a message, a dependency, or anything others compile against.
- **Warns before rebuilding if the robot is running** — colcon replaces files
  under live nodes, and with `--symlink-install` a running process can read
  half-written code.

## 2026-10-05 — IMU reduced to accelerometer-only, motor-PWM meddling removed
**By:** Varun (with Claude)
**Changed:** `nexva_sensor` (deleted `stall_guard.py`, `odom_guard.py`, added
`motion_check.py`), `nexva_explore` configs, `nexva_frimware/wheel_odometry.py`
**You need to:** rebuild. If the robot ever spins with nothing commanding it,
run the `/pid_limits` reset below.

- `stall_guard` used to raise the ESP32's PWM floor to break a suspected stall
  and restore it afterwards. Killed before restoring, it left the firmware
  driving on its own — **the robot span with nothing publishing `/cmd_vel`**.
  Removed. Nothing in `nexva_sensor` writes `/pid_limits` or `/cmd_vel` now.
- Replaced by `motion_check`: accelerometer in, verdict out, **no authority**.
  Reports `moved` / `still` / `unsure` / `no imu` on `motion_check/status`.
- Reset the firmware by hand if needed:
  `ros2 topic pub --once /pid_limits geometry_msgs/msg/Vector3 "{x: 150.0, y: 255.0, z: 0.30}"`
  (or power-cycle the ESP32 — it boots to compiled defaults).

## 2026-10-05 — robot.sh split into robotbring.sh + robotnav.sh
**By:** Varun (with Claude)
**Changed:** `robot.sh` (now only prints guidance), `robotbring.sh`,
`robotnav.sh`, `tools/wait_for_bringup.py`
**You need to:** use the two new scripts. `./robot.sh` no longer starts anything.

```
terminal 1:  ./robotbring.sh     bringup: ESP32, odometry, lidar
terminal 2:  ./robotnav.sh       Nav2 + mode manager
terminal 3:  ./web.sh            browser UI
```

The old combined script hung before starting Nav2. Its readiness check shelled
out to `ros2 topic echo` and `tf2_echo` on a 3 s timeout; those need 1–2 s just
to start, and the TF one timed out on a healthy robot. Replaced by one rclpy
process that checks all three conditions at once.

## 2026-10-01 — Cleaning sweeps the whole area instead of going square by square
**By:** Varun (with Claude)
**Changed:** `nexva_explore`
**You need to:** rebuild. Cleaning behaves differently — this is intended.

`clean_mode` defaulted to `blocks` (drive to the nearest red square, sit in it
until green). Now defaults to `sweep`: boustrophedon rows along the room's long
axis, obstacle-split regions done one at a time. Validated in the simulator
first. Planning was also ~65× faster on the Pi (223 ms → 3.4 ms), which fixed
the stop-and-go at every waypoint.
