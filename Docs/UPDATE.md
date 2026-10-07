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

## 2026-10-07 — Nav2 uses the real 30 x 30 cm square; explorer turns faster
**By:** Pramoth (with Claude)
**Changed:** `nexva_navigation/config/nav2_params.yaml`, `nexva_explore/config/{explore,clean}.yaml`
**You need to:** `./push.sh`, then restart `./robotnav.sh`.

- Nav2 modelled the robot as a 0.22 m circle (44 cm wide): gaps under ~44 cm
  counted as closed and corners were kept ~7 cm further off than needed. Both
  costmaps now use the real 300 x 300 mm square (+1 cm padding), and MPPI's
  CostCritic checks the corners (`consider_footprint: true`) so turning beside a
  wall cannot clip them. Checked: Nav2's costmap loads and configures with it.
- Explore/clean spin away from an obstacle at **0.8 rad/s** (was 0.5): a 90 deg
  turn in ~2 s instead of 3.1 s. If turns overshoot or wobble, set it back to 0.5.
- **Not yet tested in the sim or on the robot.**

## 2026-10-07 — Explore/clean re-route around sudden obstacles at once
**By:** Pramoth (with Claude)
**Changed:** `nexva_explore` (`frontier_explorer.py`, new `test/test_live_obstacles.py`)
**You need to:** `./push.sh` (rebuilds `nexva_explore`), then restart `./robotnav.sh`.

- The explorer planned only from the SLAM map (updated every 5 s) and replanned
  every 2.5 s, so a box or person appearing in front was invisible to the path
  for up to ~7.5 s: the robot stopped, turned, then swung back at the old path.
- Every replan now also marks what the lidar sees **within 1 m** as blocked, and
  something closing in ahead (inside `slow_distance`, 0.45 m) triggers a replan
  at once (at most 1/s). Nothing is written into the map; a stale lidar adds nothing.
- Off switch: `live_obstacle_range: 0`. **Not yet tested in the sim or on the robot.**

## 2026-10-07 — Website: left/right wheel speed, pop-up messages, icons
**By:** Pramoth (with Claude)
**Changed:** `nexva_web` (`nav_client.py`, `web/index.html`, tests)
**You need to:** `./push.sh`, then on the Pi `./build.sh nexva_web` and restart `./web.sh`.

- Status card shows **Left wheel / Right wheel** speed (m/s, from `/enco/counts`,
  falls back to `/joint_states`) with what each wheel was commanded; the command
  line turns orange when a wheel is >0.06 m/s off it (stall, slip, wiring).
- A line under the wheels names the data source or what is missing - including
  "the web bridge on the robot is an older build" when it sends no speed at all.
- Errors and successes pop up at the top (repeats suppressed); header shows a
  live connection badge; mission buttons have icons.

## 2026-10-07 — One-command push to GitHub (`gitpush.sh`)
**By:** Pramoth (with Claude)
**Changed:** `gitpush.sh` (new), `.gitignore`
**You need to:** nothing. To use it: `./gitpush.sh` (needs `gh auth login` once).

- `./gitpush.sh` lists what changed, asks "What did you change today?", uses your
  answer as the commit message, and pushes to `github.com/PRAMOTH-S/nexvac`
  (public). It always asks; a blank answer is refused. `--dry` shows what would go up. Not the same as `push.sh` (that one is laptop → Pi).
- Never force-pushes. If GitHub has newer commits, it rebases first, and on a conflict it stops.
- `.gitignore` now also excludes `push.sh`'s `.push_history` / `.last_push` / `.push_by`.

## 2026-10-06 — Website: live speed, always-visible STOP, new look
**By:** Pramoth (with Claude)
**Changed:** `nexva_web` (`nav_client.py`, `web/index.html`, tests)
**You need to:** same as the entry above (`./build.sh nexva_web`, restart `./web.sh`).

- Live robot speed (measured vs commanded) and an **"obstacle ahead — slowing"**
  badge when Nav2's collision monitor acts (navigate mode only).
- **STOP is pinned to the bottom of the screen** with the speed and mode beside it;
  it sends the same full e-stop as before. Explore/Clean card moved to the top.
- Restyled: two columns on desktop, segmented tabs, light/dark theme. Every
  button and id is unchanged; page title is now "Nexva Control".

## 2026-10-06 — Nav2 (navigate mode) sees obstacles earlier
**By:** Pramoth (with Claude)
**Changed:** `nexva_navigation/config/nav2_params.yaml`
**You need to:** `./push.sh`, then restart `./robotnav.sh`.

- Inflation 0.25 m / scaling 2.0 → **0.55 m / 4.0** on both costmaps. With
  robot_radius 0.22 the old value gave MPPI one 5 cm ring of cost, so it only
  "felt" an obstacle ~3 cm from the body and then hesitated or ran recoveries.
- Local costmap 5 → 10 Hz (= lidar rate), publish 2 → 5 Hz (RViz only).
- Doorways are unaffected (only the 0.22 m core blocks). Paths run a little
  further from walls in navigate mode. **Not yet measured on the robot.**

## 2026-10-06 — Missions: "Permission denied" fixed, faster startup check
**By:** Pramoth (with Claude)
**Changed:** `nexva_explore/mode_manager.py`, `launch/realbot/robot_*.sh`, script permissions
**You need to:** `./push.sh` (rebuilds `nexva_explore`), restart `./robotnav.sh`.

- `could not start clean: [Errno 13] Permission denied`: missions are now started
  as `bash <script>`, so a lost execute bit (rsync/copy) cannot block them again.
- The four mission scripts checked `/scan` with `ros2 topic echo --once` on a 3 s
  timeout, which can time out on a busy Pi while the lidar is fine. They now use
  `tools/wait_for_bringup.py` (the same check `robotbring.sh` uses).
- If bringup fails with `No such file ... /home/Scrapify/nexva_ws/install/...`, the
  `install/` folder was copied from another workspace: `CLEAN=true ./build.sh` once.

## 2026-10-06 — push.sh: records every push, merges UPDATE.md, one password
**By:** Pramoth (with Claude)
**Changed:** `push.sh`, new `tools/merge_update_md.py`
**You need to:** nothing. Pushes go to `~/Pramoth/nexva_ws` (`PI_DIR=` to change).

- Each push appends time / who / files / packages to `.push_history` on the Pi
  and `.last_push` here; every run prints the last 5 and what changed since.
  `bash push.sh --last` shows only that.
- **UPDATE.md is merged entry by entry with the Pi's copy before sending**, so
  neither person's entries are lost; it stops if one entry differs on both sides.
  `bash push.sh --pull-update` only pulls and merges (do this before editing it).
- One SSH connection per run (one password prompt); a failed rsync is no longer
  recorded as a push; all `*.sh` (incl. `launch/realbot/`) made executable on the Pi.

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
