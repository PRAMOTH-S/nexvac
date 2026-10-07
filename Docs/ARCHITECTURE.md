# Nexva Workspace — What's What

Robot: "Nexva" autonomous vacuum. Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) + ESP32 (motor
driver + encoders, micro-ROS) + RPLIDAR C1 + BNO055 IMU. See [circuit.txt](circuit.txt) for wiring
(note: that file uses namespace/node names — `vac_main1`, `bno055_imu`, `odom_guard` — that don't
match current code; treat it as background/aspirational, not literal).

## src/ packages (ours)

### nexva_bringup
Top-level launch glue only, no nodes. `launch/bringup.launch.py` starts robot_state_publisher +
`nexva_frimware`'s firmware bridge + RPLIDAR. See [LAUNCHES.md](LAUNCHES.md).

### nexva_frimware  *(package name is misspelled everywhere — leave as-is, renaming breaks imports)*
PC-side firmware interface: bridges the micro-ROS agent link, derives `/joint_states` and `/odom`
from raw encoder counts (this was moved off the ESP32 because `nav_msgs/Odometry` is 724B, bigger
than the 512B serial MTU — fragmenting it stalled the link to 1Hz).
- `launch/robot.launch.py` — kills stale `micro_ros_agent`, hard-resets ESP32 via RTS pulse, then
  starts agent + wheel_joint_publisher + wheel_odometry.
- `wheel_joint_publisher.py` — encoder counts → `/joint_states` (needs `encoder_cpr`, default 662.0,
  must match ESP32 firmware).
- `wheel_odometry.py` — encoder counts → `/odom` + TF (odom→base_footprint). Detects ESP32 reboot
  via counter-jump threshold.
- `esp32_interface.py` — old logger node, superseded, not launched by anything.
- `setup.py.save` — stray backup file, safe to delete.

### nexva_navigation
Wraps Nav2 bringup (AMCL or SLAM + planner/controller/BT navigator).
- `config/nav2_params.yaml` — the Nav2 tuning file (AMCL, costmaps, planner, controller, BT).
- `maps/sep23map1.*`, `maps/sep23map2.*` — two saved maps, different origins (~1.8m apart).
  Waypoints and zones are bound to one specific map name — mismatches are a known failure mode.
- `nexva_navigation/waypoint_follower.py` — orphaned interactive CLI goal-sender, duplicates
  `nexva_web`'s waypoint concept but has no entry point registered; not runnable via `ros2 run`
  as-is.

### nexva_slam
Thin wrapper around `slam_toolbox` (`config/slam_params.yaml`, Ceres solver settings). Separate
path from `navigation.launch.py`'s own `slam:=true` option — check which one you actually mean to
use before editing tuning.

### nexva_web
Browser control panel — the **active** UI (port 8080).
- `nav_client.py` — ROS-facing node (`nexva_web_bridge`): AMCL/TF localization tracking, `/initialpose`
  seeding, `/cmd_vel` teleop (capped vx≤0.26 m/s, wz≤1.5 rad/s to protect the ESP32 serial link),
  NavigateToPose/FollowWaypoints action clients, zone commands, e-stop.
- `web_bridge.py` — aiohttp WebSocket/HTTP server, entry point `web_bridge`. Routes: `/ws` (commands),
  `/api/waypoints`, `/api/state`, `/api/plan`, `/api/map_meta`, `/api/map.png`.
- `waypoints.py` — loads `config/waypoints.yaml`, enforces map-name match before allowing poses.
- `mapcheck.py` — hand-rolled PGM parser; checks a waypoint's cell + footprint isn't
  occupied/unknown/off-map (catches a Nav2 NavFn silent-short-stop failure mode).
- `cli.py` — entry point `waypoint_cli`, terminal equivalent of the web UI for validating
  waypoints/map/initial-pose without a browser.
- `web/index.html` — the actual single-page UI (vanilla JS, no framework), now two tabs:
  **Control** (map canvas, waypoint buttons, tour builder, zone drag-select, **Set 2D pose**
  drag, joystick teleop, motor limits, mission buttons, e-stop) and **Diagnostics** (Pi health,
  live log, console). Tabs show/hide — nothing unmounts, so the WebSocket and canvas survive.
- `pi_health.py` — CPU/clock/load/temperature/memory/disk/uptime plus **`vcgencmd get_throttled`
  decoded to words** (under-voltage, ARM capped, throttled, soft temp limit — now and since
  boot). Each section reports `ok`/`why`; nothing is fabricated when a source is missing, which
  is what happens on any non-Pi machine.
- `console_ops.py` — the **allowlisted** command console. Read-only ROS introspection plus fixed
  diagnostics. No shell, list-form argv, strict argument pattern, per-op timeout and output cap.
  Deliberately not a terminal: this UI is unauthenticated on the LAN and drives real motors, so
  arbitrary execution would be a physical-safety hole rather than only a data one.
- `config/waypoints.yaml` — 5 named waypoints, bound to map `sep23map2`.

### nexva_explore  *(ported from the vac_main1 sim, 2026-09-30)*
Autonomous **explore** and **clean**, driving `/cmd_vel` **directly — it does not use Nav2**.
This is a different philosophy from `nexva_coverage` below, which plans zones and hands goals
to Nav2. Both now exist; `nexva_explore` is the one wired to the web UI's mission buttons.

- `frontier_explorer.py` — the brain. Reads the live occupancy grid, drives to frontiers until
  the map is closed, then sweeps the floor as 0.5 m coverage blocks (red → green). Entry points
  `frontier_explorer` (starts in explore) and `auto_clean` (starts in clean).
- `stall_watch.py` — pure-Python lidar/gyro stall detector, no ROS. Verbatim from the sim.
- `mode_manager.py` — the mission supervisor. Owns which mission is running by launching and
  killing `launch/realbot/*.sh` process groups. One mission at a time.
- `map_autosaver.py` — writes `/map` to disk on every update + a slam_toolbox pose graph.
- `map_updater.py` — folds live scans into a saved map during a clean run.
- `map_registry.py` / `map_library.py` — where maps live and what is usable.
- `map_saver.py` — one-shot save on `/save_map`, replying on `/save_map_result`. Explore no
  longer autosaves; this is what the web's SAVE MAP button drives. Writes map + registry +
  pose + pose graph as a single snapshot.
- `pose_store.py` — keeps `~/nexva_maps/<map>.pose.yaml` current while exploring (atomic
  write, 1 s throttle, 30 s heartbeat) and provides ROS-free `save_pose`/`load_pose`. This is
  the 2D pose + heading a `clean` run seeds AMCL from, so the robot resumes where it stopped
  instead of guessing. Tracked only during explore — cleaning never overwrites it.
- `scan_grid.py` — scan→cell geometry shared by the updater.

Maps live in `~/nexva_maps/<source>/` with a `map.md` registry — deliberately **outside** the
workspace, which rsyncs between the laptop and the Pi. `map_library` also lists the pre-existing
`src/nexva_navigation/maps/*.yaml` (tagged `origin: nexva_navigation`), so `sep23map1`/`sep23map2`
are selectable for a clean run even though the autosaver never wrote them.

**Runtime control** (this did not exist in the sim — mode was fixed at launch):
| Topic | Type | Purpose |
|---|---|---|
| `/frontier_explorer/command` | String | `explore`/`clean`/`pause`/`resume`/`stop` |
| `/frontier_explorer/mode` | String (latched) | `explore`/`clean`/`paused`/`done`/`stopped`/`recovering` |
| `/frontier_explorer/status_json` | String, 1 Hz | JSON: mode, pose, coverage, blocks, crashes, `scan_age_s`, `pose_age_s`, … |
| `/coverage_blocks` | MarkerArray (latched) | the coloured floor squares |
| `/set_robot_mode` → `/robot_mode` | String / String (latched) | mission supervisor JSON contract |

**Two things that had to change for real hardware** (both would have been silent failures):
1. **The lidar is mounted backwards.** `base_footprint→base_link` is yawed +90°, then
   `base_link→laser` another +90° — so the scanner's +X points **backwards**. The sim assumed the
   lidar sat at the base origin facing forward. Scans are now transformed via a TF lookup;
   without it every "forward clearance" check reads behind the robot.
2. **`/scan` QoS.** The RPLIDAR publishes BEST_EFFORT; the sim subscribed RELIABLE. A RELIABLE
   subscriber simply never connects to a BEST_EFFORT publisher — no error, no scans, no motion.

**Safety gates added** (absent in the sim, where Gazebo made them unnecessary): lidar-staleness
stop, pose-staleness stop, a rear-clearance gate on reverse (`allow_blind_reverse` defaults
False, so an occluded rear reads as blocked), bounded forced-reverse pulses, and an absolute
velocity clamp as the last statement before every publish.

### nexva_coverage
Cleaning path planning, drives Nav2 rather than replacing it (adapted from Apache-2.0
`oomwoo_coverage`; Gazebo bumper dep made optional since Nexva has no bumpers).
- `coverage_planner_node.py` — whole-map boustrophedon (back-and-forth) planner, dozens of tunable
  params in `config/coverage.yaml`.
- `coverage_estimator_node.py` — tracks actual covered area (`/coverage_ratio`) so the planner
  knows when to stop; without it running, stop_at_target/gap-fill logic is inert.
- `zone_coverage_node.py` + `zone_planner.py` — user-drawn-rectangle zone cleaning (drawn on the
  web UI). `zone_planner.py` is pure numpy geometry (distance-transform clearance, region
  splitting, sweep-axis planning), no ROS dependency — has actual unit tests
  (`test/test_planner_geometry.py`).
- `config/zones.yaml` — saved zone polygons, bound to map `sep23map2`; currently ships with one
  placeholder rectangle that needs replacing with real corners.
- Depends on `nexva_web` (its `vacuum.launch.py` starts `web_bridge`).

### nexva_sensor
- `cam.py` — `/dev/video0` → `/vivo_camera/image_raw`, low-latency (BEST_EFFORT QoS, background
  capture thread). Not wired into any launch file — run manually.
- `bno055_imu.py` — BNO055 9-DOF IMU over I2C → `sensor_msgs/Imu` on `/imu`, plus `imu/status`
  (calibration counters), `imu/mag`, `imu/temperature`. Ported from the vac_main1 sim workspace.
  **Disabled**: `launch/imu.launch.py` defaults `enable:=false` and nothing includes it. Holds
  `orientation_covariance[0]` at -1 until all three sensors reach `min_calibration`, so an
  uncalibrated heading is marked untrusted rather than silently believed. Frame defaults to
  `base_link` because the URDF has no `imu_link`.
- `imu_monitor.py` — terminal readout of the IMU topics, for the bench axis/calibration check.
  Runs from the laptop; reads topics, not the I2C bus.
- `stall_guard.py` — **"the wheels spin but the robot is stuck" detector.** Sits near the
  wheels, not in the planner, so it protects teleop, Nav2, explore and clean identically.
  Compares wheel Δv from `/odom` against accelerometer-integrated Δv (plus a drift-bounded
  divergence channel for a steady-state pin). On a confirmed stall it raises the ESP32's min
  PWM through `/pid_limits` in bounded steps to break static friction, then zeroes `cmd_vel` if
  that fails, and restores the baseline on recovery. Publishes latched `stall_guard/stalled`
  and a numeric `stall_guard/status`. **With no IMU it refuses to fire at all** — verified.
  Vibration is measured and published but is NOT a trigger: on this robot the rough free floor
  out-vibrates the pin.
- `launch/imu.launch.py` — the driver plus the full bring-up sequence (wiring, I2C, pip
  packages, axis check, calibration) in its docstring. See [LAUNCHES.md](LAUNCHES.md).

Note: the Adafruit libraries (`adafruit-circuitpython-bno055`, `adafruit-blinka`) are pip-only
and Pi-only — they are deliberately **not** in `package.xml`, since there is no rosdep key and
they cannot import on x86. The node reports their absence and carries on.

### nexva_description
URDF/xacro model + meshes + Gazebo sim assets.
- `urdf/nexva.urdf.xacro` — links: base_footprint, base_link, laser, left/right wheel.
- `worlds/vacuum_world.sdf`, `config/gz_bridge.yaml`, `launch/gazebo.launch.py` — Gazebo simulation,
  not used on real hardware, not in the daily launch sequence.
- `esp32_rviz_control/esp32_bridge.py` — **older** direct-serial (`/dev/ttyUSB0`) ESP32 bridge,
  alternate to the current micro-ROS pipeline (nexva_frimware). Not wired into any launch file.

## Third-party / vendored (src/, not ours — don't "fix" these, just update)

- `rplidar_ros-ros2/` — Slamtec's official RPLIDAR ROS2 driver (BSD). `rplidar_c1_launch.py` used
  by bringup.
- `uros/micro-ROS-Agent/` — eProsima micro-ROS Agent, provides the `micro_ros_agent` executable.
- `uros/micro_ros_msgs/` — supporting msgs for the agent's graph manager.
- `uros/drive_base/` — Bosch drive_base_msgs, present but **unused** by anything in this workspace.
- `src/ros2.repos` — empty (`repositories: {}`), currently unused placeholder.

## Root-level scripts

- `build.sh` — builds the workspace. Incremental, `--symlink-install`. Cleans stale CMake
  caches left by moving this tree between machines, skips packages prebuilt for the wrong
  architecture, and keeps `simluationsequnce/` out of the build.
- `robotbring.sh` — bringup only (robot model, ESP32, odometry, lidar). Stays running; owns the
  hardware link.
- `robotnav.sh` — Nav2 + AMCL via `mode_manager`, in a second terminal. Checks bringup is up
  first (`FORCE=true` overrides); Ctrl-C leaves bringup alone so Nav2 can be bounced on its own.
- `robot.sh` — the old combined script. Runs nothing now; prints the split and exits.
- `tools/wait_for_bringup.py` — the readiness check both halves use: one rclpy process watching
  `/scan`, `/odom` and `odom -> base_footprint` with sensor QoS. Replaced per-check `ros2 topic
  echo` / `tf2_echo` calls that cost 1–2 s each to start and timed out on a healthy robot.
- `web.sh` — the web UI, after checking `aiohttp` and waiting for `map_server`.
- `push.sh` — laptop only. rsyncs **source** to the Pi and builds it there. Never sends
  `build/`/`install/`/`log/`, because the two machines are different architectures.

See [LAUNCHES.md](LAUNCHES.md) for options and the reasoning.

## Root-level (not under src/)

- `robot_zigzag_ui/` — **standalone React+Vite+Konva prototype UI, NOT integrated.** Expects
  `rosbridge_server` on `ws://localhost:9090` and a different topic set than what any package here
  actually publishes (`/selected_area`, `/custom_path`, `/cleaning_status`, ...). Contains a nested
  near-duplicate copy (`vaccum_spline_path/`) plus a zip archive and committed `node_modules`/`dist`
  build output. Treat as archived/experimental, not the active UI (that's `nexva_web`).
- `Docs/WIRING.md` — **the wiring reference to use.** Mermaid diagrams of every connection
  (IMU→Pi, ESP32→motors/encoders, USB, power) plus data flow and bring-up order. Pin numbers are
  taken from the running firmware.
- `Docs/circuit.txt` — older hardware notes. **Conflicts with the firmware** on left-motor
  direction pins (IN1/IN2 swapped) and on all four encoder pins; its ROS sections describe a
  different robot's namespace and node names. Kept for history — wire from `WIRING.md`.
- `Docs/notes.txt` — scratch shell-command notes (arduino-cli, micro_ros_arduino install).
- `Testing/` — ESP32 firmware bring-up history, oldest to newest:
  `ed_check.py` (misnamed, actually C++) → `v2_check.ino` (encoders only) →
  `ESP/drivercheck.ino` (motor only) → `ESP/encocheck.ino` (encoders only) →
  `ESP/motorpi.ino` → `ESP/teleop_with_encoder.ino` → **`ESP/microros_code/microros_code.ino`
  is the current production firmware** (matches `wheel_odometry.py`/`wheel_joint_publisher.py`
  constants: ENCODER_CPR=662.0, WHEEL_BASE=0.245, WHEEL_DIAMETER=0.067). Some test sketches have
  inconsistent pin numbering vs. each other and vs. `Docs/circuit.txt` — verify against
  `microros_code.ino` if unsure of current wiring, not the older test files.

  Firmware ROS interface (node `nexva_esp32`, two FreeRTOS tasks — ROS on core 0, a hard 50 Hz
  wheel PI loop on core 1):
  | Topic | Type | Dir | Purpose |
  |---|---|---|---|
  | `/cmd_vel` | Twist | in | velocity target; 500 ms watchdog stops the base if it goes quiet |
  | `/enco/counts` | Vector3 | out | raw encoder counts (x=L, y=R), BEST_EFFORT — the Pi derives `/odom` and `/joint_states` |
  | `/motor_pwm`, `/wheel_vel` | Vector3 | out | debug: applied PWM and measured m/s per wheel |
  | `/pid_gains` | Vector3 | in | live Kp (x), Ki (y), feedforward breakaway PWM (z); `<=0` leaves a field alone |
  | `/pid_limits` | Vector3 | in | live min PWM (x), max PWM (y), max linear m/s (z); `<=0` leaves a field alone. Clamped: PWM 0–255, max≥40, speed≤0.5, min forced below max |
  Neither tuning topic is echoed back; the web bridge holds the last-applied limits and
  republishes them every 5 s because the ESP32 forgets them on every reboot.
- `nav(1).rviz` — real-robot RViz config (see LAUNCHES.md).
- `launch.md` — the 3 canonical daily-use commands (see LAUNCHES.md for the full picture,
  including the stale hardcoded map path).

## Known parallel/duplicate implementations (pick the current one)

| Concern | Current (used) | Old/alternate (not wired in) |
|---|---|---|
| ESP32 bridge | `nexva_frimware` (micro-ROS agent + wheel_joint_publisher/wheel_odometry) | `nexva_description/esp32_rviz_control/esp32_bridge.py` (direct serial) |
| Web UI | `nexva_web` (aiohttp, port 8080) | `robot_zigzag_ui` (React, rosbridge port 9090) |
| Goal sender | `nexva_web` (`cli.py`, `web_bridge.py`) | `nexva_navigation/waypoint_follower.py` (orphaned) |
| SLAM | `nexva_navigation/navigation.launch.py` (`slam:=true`) | `nexva_slam` package (separate launch) |
