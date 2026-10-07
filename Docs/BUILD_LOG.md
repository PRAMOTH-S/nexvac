# Nexva Build Log

Running record of what changed and why, session by session — so a fresh chat (human or AI) can
pick up context without re-reading the whole diff history. **Append a new entry at the top for
every work session that changes code/config.** Keep entries short: what changed, why, what's
still broken/open. Don't restate what's derivable from `git log`/`git diff` — link to the commit
instead.

Related: [LAUNCHES.md](LAUNCHES.md) (how to run), [ARCHITECTURE.md](ARCHITECTURE.md) (what's what).

---

## Entry template (copy this for a new entry)

```
## YYYY-MM-DD — <one-line summary>
Commit(s): <hash or "uncommitted">
Changed: <files/packages touched>
Why: <the actual reason, not just what>
Open/broken: <anything left half-done, known-bad, or to verify next session>
```

---

## 2026-10-05 — Docs/WIRING.md, and circuit.txt found to contradict the firmware

Commit(s): uncommitted
Changed: new `Docs/WIRING.md`; pointers in `Docs/ARCHITECTURE.md`.

A diagram-led wiring guide (6 Mermaid diagrams: whole-robot, IMU→Pi, ESP32→motors/encoders,
USB, power, data flow) written for whoever physically assembles the robot rather than for
someone who already knows it.

**Pin numbers were taken from `microros_code.ino`, not from prose — and that exposed two real
errors in `Docs/circuit.txt`:**

| What | circuit.txt | firmware | consequence of trusting circuit.txt |
|---|---|---|---|
| Left motor direction | IN1=GPIO 25, IN2=GPIO 26 | **IN1=26, IN2=25** | left wheel drives backwards |
| Encoders | left 34/36, right 35/39 | **left 22/21, right 32/33** | no counts at all — odometry dead |

Both would produce a robot that is electrically sound and behaves wrongly, which is the worst
kind of documentation bug. `circuit.txt` is now explicitly marked historical in ARCHITECTURE.md
and WIRING.md leads with "wire to the firmware".

The guide also leads the IMU section with the ESP32-vs-Pi mistake that actually happened, since
an empty `i2cdetect` is indistinguishable from a dead board from the software side.

Verified: all 6 diagrams parse (header, bracket/paren/quote balance, 6 subgraph/6 end), the
ESP32 pin table has all 10 rows and every pin matches the firmware.

## 2026-10-05 — "imu is not being published": the BNO055 was on the ESP32, not the Pi

Commit(s): uncommitted
Changed: new `tools/check_imu.sh`; troubleshooting section in `Docs/LAUNCHES.md`.

Symptom: `No BNO055 - imu is not being published`. That line is the driver's LAST step and is
identical for every upstream cause, so it is not a diagnosis.

Walked the chain on the Pi (`Scrapify@10.42.0.241:~/varun/nexva_ws`, read-only over ssh):
aarch64 Pi 5 ✓, `/dev/i2c-1` present ✓, `i2cdetect` installed ✓, `board` + `adafruit_bno055`
import ✓, workspace current with today's code ✓, `nexva_sensor` built with all four
executables ✓ — **and `i2cdetect -y 1` completely empty, every address `--`**. A direct read
gave `ValueError: No I2C device at address: 0x28`.

Cause, found by the user: **the BNO055's I2C is wired to the ESP32, not the Pi.** The ROS
driver runs on the Pi, so from the Pi's side the bus is genuinely idle — no software check
could have found this, only the wiring.

Decision: **rewire the IMU to the Pi** (4 wires, pins 1/3/5/6). Keeps everything already built
working unchanged — driver, stall guard, bench check — with no firmware reflash. The considered
alternatives were having the ESP32 publish `sensor_msgs/Imu` over micro-ROS (risky: ~300 B
against the 512 B serial MTU, the same pressure that once stalled `/odom` to 1 Hz) or moving
stall detection into the firmware's 50 Hz loop (best latency, most work, rewrites the Pi-side
guard).

`tools/check_imu.sh` now encodes the whole chain so this is one command next time, and the
empty-bus branch names the ESP32-vs-Pi mistake first because that is what it actually was.
Verified: stops at step 1 on the laptop ("not a Pi — expected"), and reproduces the real
verdict at step 4 when run on the robot.

## 2026-10-05 — Diagnostics tab: Pi health, live log, allowlisted console

Commit(s): uncommitted
Changed: `src/nexva_web/` — new `nexva_web/pi_health.py`, `nexva_web/console_ops.py`,
`test/test_diagnostics.py`; `nav_client.py`, `web_bridge.py`, `web/index.html`, `setup.py`.
The page is now two tabs (**Control** / **Diagnostics**) by plain show/hide — nothing is
unmounted, so every existing control, listener, the WebSocket and the map canvas survive; the
canvas redraws on `requestAnimationFrame` when Control returns.

- **Pi health**: CPU total + per-core from `/proc/stat` deltas (first call says "waiting for a
  second sample" rather than guessing; a parked core reports `null`, not 0%), clock, load,
  temperature from `/sys/class/thermal` (**hottest zone of the preferred type** — naive lookup
  picked a 20°C zone on a box with two `acpitz`), memory + swap, disk, uptime, hostname.
  Every section returns `{'ok': bool, 'why': ...}`; **no section ever invents a number**.
- **Throttling** (`vcgencmd get_throttled`) decoded to words: bits 0-3 now, +16 since boot —
  under-voltage / ARM freq capped / currently throttled / soft temp limit. The since-boot row
  is highlighted because a brownout mid-turn is never caught live. On a battery Pi this is the
  signal that makes a power problem look like a power problem instead of a software bug.
- **Live log**: `/rosout` at TRANSIENT_LOCAL depth 25 (rcl offers depth 1000 × every node — a
  startup avalanche; transient-local still hands over rcl's 10 s backlog so the page opens with
  history). 400-entry ring, DEBUG dropped, 1000-char cap, monotonic seq. New clients are seeded
  from the ring before anything live — a log that starts empty cannot diagnose what already
  happened. **The callback never logs**: its own warnings would return via `/rosout` and
  feedback-loop.
- **Console — allowlisted, NOT a shell.** This UI is unauthenticated on `0.0.0.0` and drives
  real motors, so arbitrary execution would be a physical-safety hole. Allowed (read-only
  introspection only): `topic list/info/hz/echo --once`, `node list/info`, `param list/get`,
  `service list`, `df -h`, `free -m`, `uptime`, `vcgencmd measure_temp/get_throttled`,
  `i2cdetect -y 1`, `ls -l` on globbed `/dev/{esp*,ttyUSB*,ttyACM*}` (glob expanded in Python).
  Refused: `topic pub`, `ros2 run/launch/lifecycle`, `param set`, sudo, rm, systemctl, reboot.
  No shell anywhere, list-form argv, args must match `^[A-Za-z0-9_/]{1,200}$`, per-op timeout
  5-8 s with process-group kill, 20 kB output cap, max 2 concurrent so a flood cannot starve
  e-stop in the same executor.

Independently re-verified here (not taken on trust): no `shell=True` / `os.system` / string
command anywhere; `;`, `&&`, backticks, `$()`, `|`, newline, space, `..`, leading `-`, and
quotes ALL refused at `build_argv`; `topic_pub`/`run`/`launch`/`param_set`/`shell`/`reboot`/
`systemctl` all refused as operations; health snapshot correct on x86 with throttling honestly
`ok=False` ("not a Raspberry Pi"); throttle decode correct for `0x0`/`0x1`/`0x50005`/`0xF000F`.
23 diagnostics tests + 27 explore tests pass; all 12 packages build.

**Known boundary**: the argument regex excludes `.`, so dotted param names
(`qos_overrides./x`) cannot be read — widen deliberately if needed. `ros2 <cmd>` starts the
ros2 CLI daemon on first use (normal CLI behaviour).

NOT verified without the Pi/aiohttp: real `vcgencmd` parsing, `i2cdetect`, the `/dev/esp`
listing, the aiohttp routes, and browser rendering.

## 2026-10-05 — Set the robot's 2D pose by pointing at the map

Commit(s): uncommitted
Changed: `src/nexva_web/nexva_web/nav_client.py` (`set_pose_at`, `_pose_problems`),
`web_bridge.py` (`set_pose` command), `web/index.html` (pose mode on the map canvas).

Until now the only way to seed AMCL was `set_initial_pose` from a **named waypoint**, which
needs the robot physically parked on a surveyed point. Now: click **Set 2D pose**, drag on the
map from where the robot is in the direction it faces — the RViz "2D Pose Estimate" gesture,
so muscle memory carries over. One drag gives both the position and the heading, drawn as a
live arrow before it is committed.

- **The spot is checked against the loaded map before publishing** (`mapcheck.OccupancyMap` +
  `check_waypoint`, the same footprint-sampling check the waypoint CLI uses). Seeding AMCL
  inside a wall does not fail loudly — the filter converges somewhere wrong and every goal
  afterwards misbehaves for reasons that look nothing like a bad initial pose. Refusing up
  front with "pose is in an occupied cell" is the whole point.
- If the map cannot be read, the check is **skipped with a warning rather than refusing** —
  blocking a seed because the checker broke would be worse than seeding unchecked.
- A drag shorter than 8 px is rejected: too short to read a direction from, and guessing one
  would point the robot somewhere the operator never chose.
- Mode toggle shares the canvas with zone drawing, so one gesture never means two things.
- `/initialpose` is published 3× with a **zero header stamp** — same reasoning as
  `Waypoint.to_initial_pose`: stamping "now" loses the race against TF and AMCL silently drops
  it with an extrapolation error.

Verified: `./build.sh nexva_web` clean; `nav_client` imports; JS braces/parens/brackets balance
and every `$('id')` has a matching element. NOT verified: the gesture in a real browser, and
AMCL actually accepting the seed (no robot, and `web_bridge` needs aiohttp which is Pi-only).

## 2026-10-05 — BNO055 enabled (accelerometer only) + stall guard for "wheels spin, bot stuck"

Commit(s): uncommitted
Changed: `src/nexva_sensor/` (`bno055_imu.py` accel-only mode, new `stall_guard.py`, `setup.py`,
`package.xml`, `launch/imu.launch.py`), `src/nexva_bringup/launch/bringup.launch.py`.

Reported symptom: the wheels spin but the robot does not move. Encoders feed `/odom`, so
everything downstream believes it is driving.

- **Accel-only by default.** `accel_only:=true` publishes linear acceleration and angular
  velocity but marks orientation unavailable (REP-145 `orientation_covariance[0] = -1`)
  regardless of calibration, and skips the magnetometer — the thing that makes an uncalibrated
  BNO055 heading confidently wrong. `accel_only:=false` restores full behaviour.
- **IMU now starts with bringup.** `imu.launch.py` defaults `enable`/`accel_only`/`stall_guard`
  true; `bringup.launch.py` includes it behind `imu:=true` (`imu:=false` skips both nodes).
  A missing or unwired board cannot fail bringup.

**Detection — two accelerometer channels, both answering "did this body change velocity?",
the one question an accelerometer answers without drift:**
- **A, transition test (primary):** over a 1 s window compare Δv from `/odom` against
  ∫(ax−bias)dt, both projected on the *direction of travel*. Arming on |Δv_wheel| alone
  **missed the collision-at-cruise case entirely** (measured: never fired), so either side arms
  it — a pinned start-up is wheels +0.15 / body 0; a collision at cruise is wheels 0 / body −0.15.
- **B, divergence since the last anchor:** catches a steady-state pin, which A cannot (A is
  silent at constant velocity by design). Drift-bounded three ways: threshold grows with
  elapsed time, abandoned after 6 s, and re-anchored to the wheels only when A positively
  confirms agreement. Never re-anchored while stalled — that was self-confirming and freed a
  still-pinned robot at 2.6 s.
- **C, vibration: computed and published, deliberately NOT a trigger.** Measured sd(ax) on a
  rough *free* floor 0.27–1.57 vs 0.19–0.53 pinned — **the bumpy free floor out-vibrates the
  pin**, so it is not a discriminator here. (`nexva_explore/stall_watch` measured 0.10 vs 0.10.)
  `use_vibration:=true` plus a measured threshold enables it if hardware says otherwise.
- Rejected: raw |accel| (at constant velocity a healthy robot reads ~0 too), and releasing on
  absence of evidence. Verdicts are tri-state STALL / FREE / NO-OPINION; only positive FREE
  evidence releases.

**Response — bounded and reversible:** latched `stall_guard/stalled` + numeric
`stall_guard/status` → raise `/pid_limits` min PWM by 20 every 1 s, hard-capped at
`min(254, max_min_pwm=210)` **in code**, so no parameter value can invert the PWM band → after
3 s, zero Twist at 10 Hz. Re-asserted every 2 s because the web UI re-sends limits every 5 s.
Baseline learned from other publishers' `/pid_limits`, with a 15 s echo-suppression list —
without it the guard ratcheted its own boost into the baseline. Restored on recovery and on
shutdown. No reversing manoeuvre: this robot has no rear sensing.

Synthetic results (real node, isolated domain, 15 s runs): free driving **never** fires (gap
≤0.002 vs 0.054 limit); rough floor **never**; pinned from rest fires at 3.74 s, PWM
150→170→190→210, zeros from 6.79 s; collision at cruise fires 0.70 s after impact; freed by the
boost releases 1.17 s later and PWM returns to 150; stationary with odom lying **never**
("IDLE"); no IMU **never**.

Independently re-verified here: with `/odom` lying 0.15 m/s and `cmd_vel` live but **no IMU at
all**, the guard stayed `False`, reported "NO IMU … cannot and will not fire", and never
published to `/pid_limits`. That is the critical failure mode — an unwired board must never let
it start raising motor current.

**Unverifiable without the board:** axis/mount (assumes x forward, level, gravity-free linear
acceleration — the at-rest bias estimator absorbs a leak but nobody has measured it); real bias
magnitude and drift, which set channel B's entire margin; whether 50 Hz aliases real motor
vibration (it did in sim, inflating integrated body velocity); real vibration separation; and
whether a +20 PWM step actually frees this chassis on carpet. **Bench-check first:
`ros2 run nexva_sensor imu_monitor`, turning left must raise yaw.** Treat every threshold as
provisional until a real floor has been driven.

## 2026-10-05 — Explore saves on demand; pose carried across the explore→clean handoff

Commit(s): uncommitted
Changed: `src/nexva_explore/` (`launch/explore.launch.py`, `launch/clean.launch.py`, new
`nexva_explore/pose_store.py` + `map_saver` node, `frontier_explorer.py` handoff,
`test/test_pose_store.py`), `launch/realbot/{robot_explore,robot_clean_saved}.sh`,
`src/nexva_web/` (`nav_client.py`, `web_bridge.py`, `web/index.html`).

Asked for: stop auto-saving the map during explore; add a web button to save it; and on
finishing exploration behave like the sim — stop, save the 2D pose and heading, save the map,
then load that map and re-seed from the saved pose. Pose to live in a writable file.

- **Autosave removed from explore.** `map_autosaver` is gone from `explore.launch.py` (and the
  `min_save_interval` plumbing). The node stays in the package — `clean` and `manual` still use
  it.
- **Pose file:** `~/nexva_maps/<map_name>.pose.yaml` with `map_name, source, frame, x, y,
  yaw_rad, yaw_deg, saved_at, saved_unix`. Overridable by `pose_file` param / `NEXVA_POSE_FILE`.
  Written **atomically** (temp file in the same dir, fsync, `os.replace`) because a crash
  mid-write would otherwise leave a truncated file that the NEXT run reads as truth. Throttled
  to 1 s with a 30 s heartbeat — it is an SD card. Pose is tracked **only during explore**, so a
  clean run never overwrites the resume point.
- **Save on demand:** `/save_map` (String: bare name, empty for default, or JSON `{name, id,
  pose}`) → `map_saver` writes pgm+yaml, the `map.md` entry, the pose file, and the
  slam_toolbox graph when `/slam_toolbox/serialize_map` exists; replies on `/save_map_result`
  (JSON `ok, id, name, map_yaml, pose, posegraph, warnings, error`). One command, one
  consistent snapshot — a missing pose graph is a warning, not a failure.
- **Handoff:** on exploration completing, `start_cleaning` now runs `begin_handoff` — zero
  Twist held 1.5 s, capture `robot_pose()`, publish `/save_map` with it, wait ≤30 s for the
  matching id, THEN switch to clean. `replan`/`follow_path` are parked meanwhile and the mode
  stays `explore` so the web's mode list is unaffected. On failure/timeout it logs a `!!!`
  banner, publishes a failure result for the web, and **still cleans on the live SLAM map** —
  never silently lose a run. `_reset_mission_state` and the `stop` command clear the handoff.
- **Seeding:** `initial_pose_seeder` in `clean.launch.py` publishes `/initialpose` (zero stamp,
  tight covariance), waits for AMCL to subscribe, confirms via `/amcl_pose`; the cleaner starts
  when it exits (min 9 s, matching the old timer). Guards: file present and valid, same map
  name, same source, not >10 min older than the map yaml — otherwise warn and fall back.
  `seed_pose:=false` restores the old behaviour.
- **Web:** `{cmd:'save_map', name}` → ack then a broadcast `save_result`; a SAVE MAP button in
  the mission card; `/api/maps` now carries each map's pose, shown as a badge on its row.

Verified: `./build.sh nexva_explore nexva_web` clean; 27 tests pass (4 new: round-trip,
corrupt/missing, NaN, staleness guard); both launch files `--show-args`; `nav_client` imports;
on an isolated domain with synthetic TF the pose file wrote at 1 s, fell back to the heartbeat
when the pose stopped changing, and pinned on save; `map_saver` produced pgm+yaml+map.md+pose
with a success reply; the seeder published x=1.5 y=0.25 yaw=40.1° against a fake AMCL and got
confirmation; a wrong map name correctly warned and skipped.

NOT verified: the handoff state machine end to end, slam_toolbox `serialize_map`, real AMCL
accepting the seed, `web_bridge` (aiohttp is Pi-only), the page in a browser.

Incident: the agent doing this work ran `git stash`/`git stash pop` by accident while a second
agent was writing to `src/nexva_sensor/`. Checked immediately — stash list empty, the other
agent's files (`stall_guard.py`, `setup.py`, `package.xml`) all intact. No loss.

## 2026-10-05 — robot.sh split in two; found why Nav2 sometimes never started

Commit(s): uncommitted
Changed: new `robotbring.sh`, `robotnav.sh`, `tools/wait_for_bringup.py`; `robot.sh` reduced to
a pointer that exits 1; references updated in `build.sh`, `push.sh`, `web.sh`,
`launch/realbot/*.sh`, `mode_manager.py`, `mode_manager.launch.py`, `clean.launch.py`.

Reported symptom: "bringup runs but nav never launches, gets stuck there sometimes".

**Root cause — the readiness check, not the sequencing.** `robot.sh` gated Nav2 on three
checks, each `timeout 3`: `ros2 topic echo /scan --once`, same for `/odom`, and
`ros2 run tf2_ros tf2_echo odom base_footprint | grep -q .`. Each is a fresh Python process
that needs rclpy init + DDS discovery before it can receive anything. Measured on this laptop
with healthy publishers running:

| check | result | elapsed |
|---|---|---|
| `/scan` echo | OK | 1 s |
| `/odom` echo | OK | +2 s |
| `tf2_echo` | **TIMED OUT (143)** | +3 s |
| one loop iteration | — | **6 s** vs a 3 s-per-check budget |

So `/scan` and `/odom` would pass and **TF never would** — the loop spent the entire `WAIT`
and then refused to start Nav2, on a robot that was completely fine. A Pi 5 under the load of a
just-started bringup is slower than this laptop, which is why it was intermittent rather than
constant. (A second hypothesis — that `ros2 topic echo` subscribes RELIABLE and so could never
receive the RPLIDAR's BEST_EFFORT `/scan` — was tested and **disproved**: the CLI adapts to the
publisher's QoS. Recorded so nobody re-derives it.)

Fix: `tools/wait_for_bringup.py` — one rclpy node, three subscriptions (sensor QoS on all, which
receives from BEST_EFFORT and RELIABLE publishers alike), a TF listener, one discovery, held
open until all three are seen. Talks DDS directly, so a stale `ros2 daemon` cannot hide the
graph. **0 s where the old loop cost 6 s.** Exits 1 naming exactly what is missing.

Split into two terminals so the halves restart independently: Nav2 can be bounced without
power-cycling the ESP32 link, and — the point — a readiness check that is wrong can no longer
*prevent* Nav2 from starting. `robotnav.sh` warns and refuses, but `FORCE=true` overrides it,
and `robotbring.sh` never kills bringup on a failed check.

Verified: all shell syntax; helper exits 1 in ~7 s with correct per-item diagnosis when nothing
is running, and exits 0 in ~1 s against live publishers including a BEST_EFFORT `/scan`;
old-vs-new timing table above reproduced. NOT verified on real hardware.

## 2026-10-01 — Sweep + both fixes ported to the REAL bot (nexva_explore)

Commit(s): uncommitted
Changed: `src/nexva_explore/nexva_explore/frontier_explorer.py` (AST-level transplant),
`stall_watch.py` (verbatim drop-in of the rewritten sim version), `config/{clean,explore}.yaml`
(`clean_mode: sweep`), new `test/test_cleaning_plan.py` (the sim's planner suite, path-adjusted).
`robot_clean_saved.sh` was already flipped to sweep earlier today.

Method: three-way AST classification (sim vs region-agent backup vs real) split every differing
function into *sim-side change → port* vs *real-bot adaptation → keep*. 10 function body-swaps
(raw source slices, comments preserved), 9 new functions inserted after their sim predecessors,
5 targeted merges (`publish_status` keeps `publish_mode()`/`publish_status_json()` and gains
`sweep_status()`; `follow_path` keeps its staleness/pause head and only its goal check moves to
`arrive_tolerance()`; `__init__` gains `row_point_spacing`, flips `clean_mode`→`sweep` and
passes 3→2; `_reset_mission_state` gains `clean_coverage`/`goal_regions`/`clean_rows`).
All real-only machinery untouched: sensor QoS, scan TF transform, staleness stops,
rear-clearance gate, velocity clamps, runtime mode control, status_json.

Verified: 23/23 ported planner tests pass against the real file; package builds; node smoke on
an isolated domain — starts clean, `explore→clean` on `/frontier_explorer/command`, status_json
intact, `sweep=` field present. Adversarial 3-auditor workflow (parity / regressions / synthetic
coverage parity) run after the transplant — result recorded below this entry when it lands.

Adversarial audit (3 independent agents, zero blockers): (1) parity — all 19 transplanted
functions AST-identical to sim, `stall_watch.py` byte-identical, params declared/read exactly
once, nothing on the real side overrides `clean_mode`; (2) regressions — every hunk of the
634-line diff examined, exactly the declared surgery, no real-only path touched; (3) behaviour —
the real file's planner output is byte-for-byte identical to the sim's on every synthetic case,
and an extra run with the REAL robot's own params (footprint 0.30, clearance 0.07) still passes
every ≥98% coverage gate (min 99.09%).

Advisories from the audit, not fixed here:
- `use_imu_stall: false` keeps the new stall detector inert on hardware (pre-existing and
  intended — no IMU enabled). Flip it only after the BNO055 is live AND the detector is
  re-calibrated on the real lidar.
- Sweep arrival tolerance tightens to 0.1215 m — below anything validated on the real
  controller; watch for near-goal oscillation.
- The speck-fill threshold scales with the real footprint to ~0.80 m — small real obstacles
  may not split regions (driving still avoids them; the mask is never edited). Plan build may
  pause the node a few seconds on the Pi 5; the cmd_vel watchdogs cover the wheels.

NOT verified on hardware. First real clean: supervised, hand on the cutoff.

## 2026-10-01 — Sweep is now the default clean; sim validated, two defects fixed

Commit(s): uncommitted. **All changes in `simluationsequnce/` (the sim) — the real robot's
`src/` untouched today.** Port to the real bot comes after sim results are accepted.

The complaint: cleaning moved "box by box". Cause: `clean_mode` defaulted to `'blocks'`
(nearest red square, sit until green). The sim already had a real boustrophedon sweep behind
`clean_mode:='sweep'` — rows along the long axis, start corner nearest the robot, obstacle-split
regions, nearest-region ordering. It was simply never the default, **and the shell scripts
(`run_clean_saved_map.sh`, `robot_clean_saved.sh`, `test_sweep.sh`) pinned `blocks` over the
node default — the first "sweep" test silently ran blocks mode until that was found.**

Changed (sim): `clean_mode` default → `sweep` in node + launch files + the three shell scripts;
sweep correctness fixes (edge rows pinned to the drivable extent — the old origin-anchored
sampling left an uncleaned strip at the far wall; `step` floored and capped at 0.9×swath;
arrival tolerance capped below `cleaning_radius` while sweeping; intermediate row waypoints;
passes 3→2); `sweep=` progress in `robot_status`. Synthetic coverage: ≥99% on all cases, 100%
on the standard rooms (was as low as 89% with wide spacing). New harness: `test_sweep.sh` +
`tools/sweep_recorder.py` → verdict + `test_sweep_runs/<stamp>/`.

30-min live run after the flip: coverage climbed steadily to 48% (sweep works; full room
projects ~1 h at 0.15 m/s) but exposed two real defects, fixed by parallel agents:
1. **Region fragmentation** — 29 regions where ~16 are real. SLAM speckle (a 1-px dot and a
   short dash) split rows; unreachable floor (outside the wall, inside box obstacles) was
   planned and counted. Fix: plan only floor connected to the robot; fill speck-sized holes
   (≤13 cells, derived from footprint+inflation) for REGION decisions while never editing the
   drive mask (a speck and a chair leg look identical in the grid). 29→19 regions, transit
   47→29 m, coverage held ≥99.9%. Merging further was measured and REJECTED (drove 6–13 m
   more). Images: scratchpad `regions/home_{before,after}_regions.png`.
2. **False "CRASH" stalls** — ground truth (Gazebo model pose, independent of the lidar)
   showed 3 of 5 crashes fired while driving at exactly the commanded 0.15 m/s; one 12-min
   stretch thrashed ~30 crashes in one region. The detector's per-beam model (`-d·cosθ`)
   treats every return as a point: parallel walls really change by ~0 and obstacle-edge beams
   jump by metres, so on long straight rows the fit collapses. **SLAM pose is NOT valid ground
   truth here — it uses the same lidar.** Fix in progress at time of writing: predict from
   local wall direction (dr = -d·n_x/(n·u)), reject range-jump/grazing beams, abstain
   (`stall=degenerate`) when the scene carries no motion information.

**Incident:** an agent's isolated sim run defaulted `map_dir` to the shared
`~/vac_main1_maps/sim/` and its autosaver overwrote `home.*` with a 90 s fragment. Restored
from a 13:51 backup with user approval (same map ⇒ numbers stay comparable); posegraph MAY be
torn — clean runs auto-fall back to AMCL+map_updater if so. Lesson now in the harness rules:
isolated clean runs must pass `map_dir:=<scratch>`.

**Stall fix landed** (`stall_watch.py`): per-beam prediction from the local wall tangent
(dr = -d·n_x/(n·b)), range-jump and grazing-incidence beams dropped, abstention
(`stall=degenerate`) below `min_info=25`. Calibration against Gazebo truth: moving fit
p05/p50 = 0.975/0.998 vs pinned max |0.035| — a clean gap at the unchanged 0.25 threshold.
On recordings: 46 sustained would-be false crashes → 0; all 11 deliberate pins and the one
real live hold still caught. 201 tests green.

**Final combined 300 s run** (`test_sweep_runs/20261001_151107`): full plan from t=0
(posegraph healed by the previous run's re-serialize), **21 regions / 538 waypoints** (was
29/542), `sweep=rows` throughout, **1 crash (was 8** in the equivalent pre-fix window; the
earlier baseline burned 12 min thrashing ~30 crashes in one region), `view_backs` median
exactly 1.0 while driving, coverage 10% at 300 s and no thrash — over a full ~1 h clean the
crash reduction is where the time comes back.

Open: port to the real bot's `nexva_explore` once sim results are accepted — the same
`clean_mode` flip (real `robot_clean_saved.sh` already flipped), the planner changes
(edge rows, step cap, arrival tolerance, speck-fill, reachable-floor), and the new
`stall_watch.py` — noting the real bot port runs `use_imu_stall:=false` until the BNO055 is
enabled, so the stall-watch half is dormant there anyway.

## 2026-09-30 — Sim explore/clean stack ported onto the real robot (nexva_explore)

Commit(s): uncommitted
Changed: new package `src/nexva_explore/`; new `launch/realbot/*.sh`; `robot.sh` step 3;
`src/nexva_web/{nav_client.py,web_bridge.py,web/index.html,package.xml}`.
**Untouched on purpose: the real URDF, bringup, firmware nodes, Nav2 config.**

Ported from `simluationsequnce/` (the vac_main1 Gazebo workspace): `frontier_explorer` +
`stall_watch` (explore/clean brain, drives `/cmd_vel` directly, no Nav2), `mode_manager`
(mission supervisor), `map_autosaver`, `map_updater`, `map_registry`, `map_library`,
`scan_grid`.

**Two silent-failure bugs the port had to fix — both would have "built fine" and not worked:**
1. **The lidar is mounted backwards.** `base_footprint→base_link` is yawed +90°, then
   `base_link→laser` another +90° = **180° total**, so the scanner's +X points backwards. The
   sim's `scan_callback` assumed the lidar sat at the base origin facing forward and treated raw
   (range, angle) as base-frame points. Uncorrected, every forward-clearance check would have
   measured *behind* the robot — it would have driven into obstacles reading "clear ahead".
   Now transformed via a cached TF lookup, so the URDF stays the source of truth.
2. **`/scan` QoS.** The RPLIDAR publishes BEST_EFFORT; the sim subscribed RELIABLE. That
   combination never connects — no error, no scans, no motion.

**Safety gates added** (Gazebo made these unnecessary; real motors do not): lidar-staleness stop
(`scan_timeout` 0.5 s — previously the node drove on the last snapshot forever if the lidar
died), pose-staleness stop (`pose_timeout` 2.0 s — `robot_pose()` returned a frozen pose
indefinitely on TF failure), rear-clearance gate on reverse with `allow_blind_reverse` defaulting
False (10 of 11 reverse paths never checked behind, and an occluded rear sector reads as *clear*),
bounded forced-reverse pulses, and an absolute velocity clamp as the last statement before
publish.

**Runtime mode control is new.** In the sim the mode was fixed at launch and could only be
changed by killing and relaunching the whole stack. Added `/frontier_explorer/command`,
latched `/frontier_explorer/mode`, and `/frontier_explorer/status_json`, with
`_reset_mission_state()` shared between `__init__` and the command handler so they cannot drift
(a missed field gives a "clean" run that instantly declares itself done). `pause` is a separate
flag, NOT `mode='done'` — `done` is absorbing and stops publishing `cmd_vel` entirely.

**Three integration bugs I found and fixed after the agents finished:**
- `mode_manager` defined `def handle()`, which shadows `rclpy.Node.handle` — a property
  `Node.__init__` uses as a context manager. The node could never construct
  (`TypeError: 'method' object does not support the context manager protocol`). Renamed
  `handle_request`. Not a sim bug; introduced by the port.
- `map_autosaver`/`map_updater` raised `RCLError: rcl_shutdown already called` on every normal
  SIGTERM. Added `ExternalShutdownException` handling + an `rclpy.ok()` guard.
- `mode_manager`'s shutdown stop-burst failed with "publisher's context is invalid": rclpy's own
  signal handler tears the context down *before* `spin()` returns, so `destroy_node()` had
  nothing to publish with — the burst failed exactly when it matters. Now takes over
  SIGINT/SIGTERM (`SignalHandlerOptions.NO`) and stops the mission while ROS is still up.
  **Verified: 16 zero Twists reach `/cmd_vel` on SIGTERM.**
- Web `estop()` also told `mode_manager` to stop, and `web_bridge.main()` calls `estop()` in its
  `finally` — so restarting `./web.sh` would have aborted a running clean. Now
  `estop(stop_mission=False)` on the shutdown path; the STOP button still does the full stop.

Verified: all 12 packages build; all 4 nodes start clean on an isolated domain; all 4 launch
files pass `--show-args`; mode contract tested end to end (latched state on connect, bad map
refused listing real alternatives, `clean sep23map2` starts a real mission, `stop` returns to
idle, no stray processes); explorer `explore→paused→clean` verified live; `status_json` carries
every key; JS balanced with no dangling element ids.

**NOT verified:** nothing has run against real hardware — no robot on this machine. The
readiness waits, the missions themselves, and all tuning values are untested on the floor.
`web_bridge` cannot run here (aiohttp is Pi-only). First real run should be supervised with a
hand on the physical cutoff.

## 2026-09-30 — Runtime PWM/speed limits for the ESP32 PID, from the web UI

Commit(s): uncommitted
Changed:
- `Testing/ESP/microros_code/microros_code.ino` — `MAX_PWM`/`MIN_PWM`/`MAX_LINEAR_SPEED` were
  compile-time `#define`s; now `DEFAULT_*` plus live variables `motor_min_pwm`,
  `motor_max_pwm` (int — 32-bit aligned loads are atomic on ESP32, so the motor path reads
  them without the spinlock) and `max_linear_speed` (rosTask-only, no lock needed). New
  subscriber `/pid_limits` (`geometry_msgs/Vector3`: x=min PWM, y=max PWM, z=max m/s;
  `<= 0` leaves a field alone) and `limitsCallback()` next to the existing `gainsCallback()`.
  `wheelControl()` now takes the band as arguments, snapshotted once per `controlStep()`
  under `state_mux`. **Executor handle count raised 2 → 3** — adding a subscription without
  this makes `rclc_executor_add_subscription` fail and entity creation abort silently.
- `src/nexva_web/nexva_web/nav_client.py` — `set_pid_limits()`, `reset_pid_limits()`,
  publisher on `/pid_limits`, and a 5 s republish timer.
- `src/nexva_web/nexva_web/web_bridge.py` — `pid_limits` and `pid_limits_reset` commands.
- `src/nexva_web/web/index.html` — "Motor limits (ESP32 PID)" card.

Why: the PWM→speed curve drifts with battery state (the firmware's own comment records
0.00341·(PWM−147) on 23 Sep vs 0.0065·(PWM−108) on 28 Sep with no code change), and the
breakaway floor depends on the floor surface. Retuning needed a reflash.

Design points worth knowing:
- Firmware clamps hard: PWM 0–255, max PWM ≥ 40, speed ≤ 0.50 m/s, and forces min < max
  after clamping (an inverted band would pin both wheels at max the moment a slider is
  dragged the wrong way). The bridge validates the same bounds first and **refuses** an
  inverted band rather than letting the firmware silently fix it.
- The ESP32 reverts to compiled defaults on every reboot — and bringup reboots it on purpose.
  So once the operator has applied limits, the bridge republishes them every 5 s. Until then
  nothing is sent and the firmware's defaults are the truth.
- Raising `min_pwm` does NOT move the feedforward breakaway (`z` on `/pid_gains`). Set both.

Verified: Python compiles, `nexva_web` builds, JS brace/paren balance clean.
**NOT verified: the firmware does not compile-check here** — `arduino-cli` is not installed on
this machine. Flash and check the debug serial for `limits: pwm 150..255  speed 0.30 m/s`
after `ros2 topic pub --once /pid_limits geometry_msgs/msg/Vector3 "{x: -1, y: -1, z: 0.3}"`.

## 2026-09-30 — push.sh: laptop → Pi deploy

Commit(s): uncommitted
Changed: new `push.sh` at the workspace root
Why: the Pi's copy at `Scrapify@10.42.0.241:~/varun/nexva_ws` had none of the new scripts, so
`./build.sh` there failed with "command not found".

rsync of **source only**. `build/`, `install/`, `log/` are hard-excluded: the Pi is aarch64 and
the laptop x86_64, and copying artifacts either way is the direct cause of the stale-cache and
wrong-architecture failures logged in the entry below. Also excludes `.git/`,
`simluationsequnce/`, `robot_zigzag_ui/`.

Verified: passwordless SSH works, remote dir exists, `./push.sh --dry` reports 377 files /
2.2 MB with no build artifacts in the list. **The real push was not run from this session** —
it writes to the robot and triggers a build there.

Note for the Pi: do **not** `sudo ./build.sh`. Root-owned files in `build/`/`install/` then
block the normal user.

Also: removed one broken symlink in `simluationsequnce/install/.../__pycache__/` (a
`--symlink-install` leftover pointing at a deleted `.pyc`) that VS Code was raising ENOENT on.

## 2026-09-30 — Root launcher scripts: build.sh, robot.sh, web.sh

Commit(s): uncommitted
Changed: new `build.sh`, `robot.sh`, `web.sh` at the workspace root (all executable)
Why: replace the three hand-typed `ros2 launch` commands with scripts that source ROS
themselves and enforce the ordering that was previously only in someone's head.

`robot.sh` gates Nav2 on bringup being **proven** up — `/scan` + `/odom` + the
`odom -> base_footprint` TF all live — rather than on a fixed sleep. Starting AMCL before
those exist is the failure mode that presents as "Nav2 is broken".

Two real defects found and worked around while testing the full build:
- **Stale CMake caches.** `build/*/CMakeCache.txt` in 4 packages still pointed at
  `/home/Scrapify/nexva_ws` (this tree was copied off the Pi), which fails every C++ package
  with "current CMakeCache.txt directory is different". `build.sh` now detects and removes
  those build dirs.
- **Wrong-architecture prebuilds.** `install/micro_ros_agent/lib/*` are aarch64 binaries from
  the Pi; on x86_64 the link fails with `Relocations in generic ELF (EM: 183)`. `build.sh`
  now **skips** that package on a mismatched machine rather than deleting it — only the Pi
  runs the agent, and rebuilding needs network (ExternalProject + GIT_REPOSITORY).
  `FORCE_ALL=true` overrides.

Also: `build.sh` writes `COLCON_IGNORE` into `simluationsequnce/`, which is a whole second ROS
workspace inside this tree and was being crawled into this workspace's build.

Verified: `./build.sh` → 11 packages finished, micro_ros_agent skipped with an explanation.
Guard paths tested: `MAP=doesnotexist ./robot.sh` lists the real maps; `./web.sh` refuses with
install instructions because aiohttp is absent on this x86 box.

Open/broken:
- Neither `robot.sh` nor `web.sh` has been run against real hardware from this session — only
  their failure paths were exercised. The readiness-wait logic is untested on a live robot.
- `micro_ros_agent` remains unbuilt for x86_64. Fine for a laptop; on the Pi it is already
  built and will not be skipped there.

## 2026-09-30 — BNO055 IMU added to nexva_sensor, disabled by default

Commit(s): uncommitted
Changed: `src/nexva_sensor/` — new `nexva_sensor/bno055_imu.py` and `nexva_sensor/imu_monitor.py`
(both ported from `simluationsequnce/src/vaccum/vaccum/`), new `launch/imu.launch.py`,
`setup.py` (two entry points + launch install), `package.xml` (rclpy, sensor_msgs, std_msgs,
cv_bridge exec deps)

Why: the IMU is needed by the explore/clean port coming from the sim workspace — that code
defaults `use_imu_stall:=true` and there was no `sensor_msgs/Imu` publisher anywhere in this
build. Landing the driver now, switched off, so it is in the tree and verified to build without
changing any current behaviour.

Verified:
- `colcon build --packages-select nexva_sensor` clean
- `ros2 pkg executables nexva_sensor` → `bno055_imu`, `cam`, `imu_monitor`
- `ros2 launch nexva_sensor imu.launch.py` (default) starts **no** nodes
- `enable:=true` on this x86 box starts the node, logs the missing Adafruit libraries with the
  exact install command, and keeps warning — no crash, nothing published

Fixed while porting: the sim's launch used `ParameterValue(..., value_type=[int])` for
`axis_remap`, which throws `Unrecognized data type: [<class 'int'>]` on Jazzy. Now
`value_type=List[int]`. The sim's own launch files still carry the broken form.

Open/broken:
- `frame_id` defaults to `base_link`, not `imu_link` — `nexva.urdf.xacro` has no `imu_link`, and
  stamping with a frame that has no transform makes consumers drop the message silently. Measure
  the board's mount offset, add the link, then switch the default.
- Axis remap and calibration are unverified — nobody has had the board on the bench yet. The
  identity remap is only correct if the board's x points forward.
- Adafruit libs are not installed on the Pi yet (as far as this session knows).

## 2026-09-30 — Generated documentation set

Commit(s): uncommitted
Changed: added `Docs/LAUNCHES.md`, `Docs/ARCHITECTURE.md`, `Docs/BUILD_LOG.md` (this file)
Why: needed a persistent reference so context survives a chat/session switch
Open/broken: none touched in code. Flagged during the audit (still true as of this entry,
not yet fixed):
- `launch.md` and `.vscode/settings.json` hardcode `/home/Scrapify/nexva_ws/...`, actual path is
  `/home/varun/nexvabuilds/rasp/nexva_ws`.
- `robot_zigzag_ui/` is a disconnected prototype (rosbridge on 9090, mismatched topics) — not
  wired to any package, safe to ignore unless explicitly reviving it.
- `nexva_navigation/waypoint_follower.py` has no entry point registered — orphaned.
- `nexva_description/launch/gazebo.launch.py` references `rviz/rviz.rviz`, which doesn't exist.
- Several packages still have template "TODO: Package description" in `package.xml`
  (nexva_bringup, nexva_description, nexva_navigation, nexva_sensor, nexva_slam).
- `nexva_frimware/setup.py.save` is a stray backup file.

## Uncommitted working tree (snapshot 2026-09-30, after the IMU work)

**Nothing in this log has been committed.** Check `git status` before trusting this list.

Modified:
- `src/nexva_navigation/config/nav2_params.yaml`
- `src/nexva_web/nexva_web/nav_client.py`, `web_bridge.py`, `src/nexva_web/web/index.html` —
  pre-existing work (zone drawing + teleop), not touched this session
- `src/nexva_sensor/setup.py`, `package.xml` — this session (IMU)

Untracked:
- `src/nexva_coverage/` — whole package, new, uncommitted
- `src/nexva_sensor/launch/`, `nexva_sensor/bno055_imu.py`, `imu_monitor.py` — this session
- `src/nexva_sensor/nexva_sensor/cam.py` — pre-existing, still uncommitted
- `Docs/ARCHITECTURE.md`, `Docs/BUILD_LOG.md`, `Docs/LAUNCHES.md` — this session
- `simluationsequnce/` — the vac_main1 sim workspace, dropped in as the source for the
  explore/clean port. Large (has its own build/install/log). Probably should **not** be
  committed into this repo as-is; consider a sibling directory or `.gitignore`.
- `robot_zigzag_ui/`, `nav(1).rviz`, `launch.md`, a screenshot — pre-existing

---

## Prior history (from `git log`, summarized for context — not a substitute for `git log -p`)

- `fe58051` Initial Nexva ROS2 workspace
- `0c7defa` Testing — added ESP32 firmware bring-up test sketches
- `dc8b9d3` Agent Restart is working no need to press the reset button — this is the
  cleanup_processes/RTS-pulse logic now in `nexva_frimware/launch/robot.launch.py`
- `b5792ed` No delay in odom with lidar, joint is publishing but direction is wrong - before break
  — early wheel_joint_publisher/wheel_odometry work, wrong-direction bug noted at the time
- `098fbb4` urdf tf is working
- `91f14dd` robot model
- `02ce650` map created — likely one of the sep23map1/sep23map2 maps
- `fd0294a` inflation to 0.05 — Nav2 costmap inflation tuning
- `5e80070` Nav2 is working great
- `99d9c6e` Website and Nav2 working — nexva_web reaching a working state alongside Nav2
- `1bbcf74` Update micro-ROS Agent — vendored `uros/micro-ROS-Agent` bump
- `1ce423a` Nithanam Code with PID — latest commit; PID work likely lives in
  `Testing/ESP/microros_code/microros_code.ino` (firmware-side velocity control), not yet
  reflected in a host-side doc — verify against current `.ino` if this matters for your task.

Note: the "wrong-direction" odom bug from `b5792ed` and the open-loop-PWM-with-a-later-PID-TODO
comment in `Testing/ed_check.py` predate `1ce423a`'s PID commit — if you're debugging odometry
direction or velocity tracking, check whether `1ce423a` actually resolved these before assuming
they're still open.
