# Nexva Launch Reference

Source of truth for "how do I start this thing." Derived from [launch.md](../launch.md) plus every `*.launch.py` found in `src/`.

## Scripts (use these)

Four scripts at the workspace root. They source ROS and `install/setup.bash`
themselves, so a bare terminal is fine.

```bash
./build.sh          # build the workspace
./robotbring.sh     # terminal 1: bringup (ESP32, odometry, lidar)
./robotnav.sh       # terminal 2: Nav2 + AMCL, mode switching
./web.sh            # terminal 3: the web UI
./push.sh           # laptop only: send source to the Pi and build it there
```

`robot.sh` used to do the first two in one terminal. It no longer runs anything — it prints
the split and exits. See [`./robotbring.sh` and `./robotnav.sh`](#robotbringsh-and-robotnavsh).

### `./push.sh` — laptop → Pi

```bash
./push.sh                    # sync, then build on the Pi
./push.sh --dry              # list what would be sent, send nothing
./push.sh --no-build         # sync only
./push.sh --delete           # also remove files on the Pi that are gone here
PI_HOST=10.42.0.53 ./push.sh
```

Defaults: `Scrapify@10.42.0.241:~/varun/nexva_ws`. Override with `PI_HOST`, `PI_USER`, `PI_DIR`.

**`build/`, `install/` and `log/` are never sent.** The Pi is aarch64 and the laptop is
x86_64, so copying compiled artifacts either direction produces exactly the
`Relocations in generic ELF (EM: 183)` and stale-`CMakeCache` failures that `build.sh` exists
to clean up. Source crosses; each machine compiles its own. Also excluded: `.git/`,
`__pycache__/`, `simluationsequnce/` (a separate workspace), `robot_zigzag_ui/` (`node_modules`).

`--delete` previews every removal and asks before doing it.

Note: **do not `sudo ./build.sh`.** colcon must run as your own user — building as root leaves
root-owned files in `build/` and `install/` that your normal user then cannot overwrite.

### `./build.sh`

Incremental by default; `--symlink-install` is always on, so Python edits take effect without
a rebuild (you only need one after adding a file, changing an entry point, or touching a
launch/config file).

```bash
./build.sh                      # everything
./build.sh nexva_web            # one package
./build.sh nexva_web nexva_sensor
CLEAN=true ./build.sh           # wipe build/ install/ log/ first
TEST=true ./build.sh            # run test suites after
./build.sh -- --parallel-workers 2   # anything after -- goes to colcon
```

It handles two problems caused by this tree being copied between the Pi and a laptop:

- **Stale CMake caches.** `CMakeCache.txt` has absolute paths baked in, so a `build/` tree from
  `/home/Scrapify/nexva_ws` fails every C++ package. The script detects those and deletes just
  those build dirs (they are gitignored and regenerate).
- **Wrong-architecture prebuilds.** `micro_ros_agent`'s libraries are `aarch64` (built on the
  Pi) and will not link on x86_64 — the error is the opaque `Relocations in generic ELF
  (EM: 183)`. It is **skipped**, not deleted: only the Pi ever runs the agent, and rebuilding
  it needs the network (it fetches upstream via `ExternalProject`). Override with
  `FORCE_ALL=true CLEAN=true ./build.sh`.

It also drops a `COLCON_IGNORE` into `simluationsequnce/`, which is a second, separate ROS
workspace living inside this tree and would otherwise be built as part of it.

### `./robotbring.sh` and `./robotnav.sh`

Two terminals, two halves.

```bash
# terminal 1 — bringup: robot model, ESP32, odometry, lidar
./robotbring.sh
WAIT=120 ./robotbring.sh        # allow longer for the lidar to spin up

# terminal 2 — Nav2 + AMCL, and the web UI's mode switching
./robotnav.sh                   # map defaults to sep23map2
MAP=sep23map1 ./robotnav.sh
MAP=/abs/path/to/other.yaml ./robotnav.sh
SLAM=true ./robotnav.sh         # build a map instead of localising
FORCE=true ./robotnav.sh        # start even if the bringup check fails
```

`robotnav.sh` starts `mode_manager`, which immediately runs the **navigate** mission — Nav2 on
your saved map. The web UI can then switch to explore / clean / manual / stop without touching
a terminal. Ctrl-C in terminal 2 stops the mission and the manager and **leaves bringup alone**,
so Nav2 can be bounced without power-cycling the ESP32 link.

**Why they were split.** They used to be one script that waited for bringup before starting
Nav2, and it would sit there and never start Nav2 on a perfectly healthy robot. The sequencing
was fine; the readiness check was not. It shelled out to `ros2 topic echo --once` and
`tf2_echo` on a 3-second timeout, and those need 1–2 s just to start a Python process and
finish DDS discovery — longer on a loaded Pi. Measured here with healthy publishers running,
the `tf2_echo` check **timed out every time** and one loop iteration cost 6 s against a
3 s-per-check budget. `/scan` and `/odom` passed, TF never did, and the script burned the whole
`WAIT` and gave up.

The check now lives in `tools/wait_for_bringup.py`: one rclpy process watching all three at
once with sensor QoS, talking DDS directly so a stale `ros2 daemon` cannot hide the graph from
it. Same three conditions, **0 s instead of 6 s per iteration**. And because the halves are
separate, a check that is wrong can no longer stop you starting Nav2 — `FORCE=true` overrides it.

Run the check on its own any time:

```bash
python3 tools/wait_for_bringup.py --timeout 60
```

`robot.sh` still exists but runs nothing: it prints the split and exits 1.

### `./web.sh`

```bash
./web.sh
PORT=8081 ./web.sh
WAYPOINTS=/abs/path/to/other.yaml ./web.sh
NOWAIT=true ./web.sh            # do not wait for Nav2
```

Checks `aiohttp` is installed before launching (its absence otherwise surfaces as a bare
`ModuleNotFoundError` from inside a launch file), waits for `map_server` so the map and
waypoint check work on first load, and prints the actual LAN URL rather than `0.0.0.0`.

## The underlying commands

What the scripts run, if you'd rather do it by hand — in order, each in its own terminal:

```bash
# 1. Bringup: robot_state_publisher + ESP32 micro-ROS bridge + odometry + RPLIDAR C1
ros2 launch nexva_bringup bringup.launch.py

# 2. Navigation: AMCL localization + Nav2 stack (planner/controller/BT navigator)
ros2 launch nexva_navigation navigation.launch.py map:=<path-to-map.yaml>

# 3. Web UI: aiohttp server + WebSocket bridge (browser control panel)
ros2 launch nexva_web web.launch.py
```

**Map path warning:** [launch.md](../launch.md) hardcodes
`/home/Scrapify/nexva_ws/src/nexva_navigation/maps/sep23map2.yaml`. This workspace actually
lives at `/home/varun/nexvabuilds/rasp/nexva_ws`. Use:
```
map:=/home/varun/nexvabuilds/rasp/nexva_ws/src/nexva_navigation/maps/sep23map2.yaml
```
(same stale path also sits in `.vscode/settings.json` → `cmake.sourceDirectory`).

Two maps exist in `src/nexva_navigation/maps/`: `sep23map1` and `sep23map2` (different
origins, ~1.8 m apart). Waypoints/zones are bound to a specific map name — mixing them is a
documented failure mode (see `nexva_web/waypoints.py`, `nexva_coverage/config/zones.yaml`).

## Add cleaning / zone coverage on top

The 3 commands above do **not** start coverage. To get zone-drawing + cleaning from the web UI,
run this 4th command after the above three are up:

```bash
ros2 launch nexva_coverage vacuum.launch.py
```

This starts `coverage_estimator`, `zone_coverage`, and `nexva_web`'s `web_bridge` together, and
kills any stray `teleop_twist_keyboard` first (it fights Nav2 for `/cmd_vel`). It assumes
bringup + navigation are already running — it does not start them.

Narrower alternatives (also assume Nav2 is already running):
```bash
ros2 launch nexva_coverage coverage.launch.py       # whole-map boustrophedon only
ros2 launch nexva_coverage zone_coverage.launch.py  # zone cleaning only
```

## What each launch file actually brings up

| Launch file | Package | Starts |
|---|---|---|
| `bringup.launch.py` | nexva_bringup | robot_state_publisher (from xacro) → includes `nexva_frimware/robot.launch.py` (ESP32 reset + micro_ros_agent + wheel_joint_publisher + wheel_odometry) → includes `rplidar_ros/rplidar_c1_launch.py` |
| `robot.launch.py` | nexva_frimware | `cleanup_processes` (pkill stale micro_ros_agent, pulse ESP32 RTS to hard-reset) → then `micro_ros_agent` (serial, `/dev/esp`, 460800 baud) + `wheel_joint_publisher` + `wheel_odometry` |
| `navigation.launch.py` | nexva_navigation | AMCL (localization_launch.py) or SLAM (slam_launch.py, if `slam:=true`) + `navigation_launch.py`, inside `nav2_container`. Params default to `config/nav2_params.yaml` |
| `slam.launch.py` | nexva_slam | `async_slam_toolbox_node` (mapping mode by default), params from `config/slam_params.yaml` — separate/older path from navigation.launch.py's own slam option |
| `web.launch.py` | nexva_web | `web_bridge` only (waypoints file, host `0.0.0.0`, port `8080`) |
| `vacuum.launch.py` | nexva_coverage | kills stray teleop, then `coverage_estimator` + `zone_coverage` + `web_bridge` |
| `coverage.launch.py` | nexva_coverage | `coverage_estimator` + `coverage_planner` (params from `config/coverage.yaml`) |
| `zone_coverage.launch.py` | nexva_coverage | `zone_coverage` node only (params from `config/zone_coverage.yaml` + `config/zones.yaml`) |
| `gazebo.launch.py` | nexva_description | Gazebo sim (`vacuum_world.sdf`) + robot_state_publisher + spawn + optional RViz + `ros_gz_bridge` (`config/gz_bridge.yaml`). **Not used on real hardware, not in launch.md.** References `rviz/rviz.rviz`, which does not exist in this workspace (only root `nav(1).rviz` does) — will likely error if `rviz:=true`. |
| `imu.launch.py` | nexva_sensor | BNO055 IMU driver. **Disabled by default** (`enable:=false` starts nothing). Nothing else includes it. See below. |

## Diagnostics tab

The web UI has two tabs. **Diagnostics** shows:

- **Pi health** — CPU (total and per-core), clock, load, SoC temperature, memory, disk, uptime.
- **Throttling** — the one to watch on a battery-powered Pi. Under-voltage, ARM frequency
  capped, currently throttled, soft temperature limit; each shown both **now** and **since
  boot**, because a brownout under motor load is almost never caught live. If the robot
  misbehaves under load and this row is lit, it is a power problem, not a software one.
- **Live log** — `/rosout` from every node, filterable by level and node, with a 400-entry
  backlog so opening the tab shows what already happened. Pause/resume autoscroll.
- **Console** — a dropdown of permitted operations plus an argument box, showing the exact
  command before it runs.

The console is **allowlisted, not a shell**: `ros2 topic list/info/hz/echo --once`, `node
list/info`, `param list/get`, `service list`, `df -h`, `free -m`, `uptime`, `vcgencmd
measure_temp/get_throttled`, `i2cdetect -y 1`, and `ls -l /dev/esp*` — read-only introspection
only. Anything that publishes, launches, kills, writes or needs root is refused, as is any
argument containing a shell metacharacter, space, `..` or a leading `-`.

That boundary is deliberate: this UI listens on `0.0.0.0` with no authentication and it drives
real motors, so an arbitrary-command endpoint would be a physical-safety hole, not just a data
one. If you need a specific extra operation (e.g. `ros2 topic pub` for bench work), add it to
the table in `console_ops.py` deliberately rather than relaxing the pattern.

## Stall guard — "the wheels spin but the robot is stuck"

Starts with bringup alongside the IMU. Watches wheel Δv against accelerometer Δv; on a
confirmed stall it raises the ESP32's min PWM in bounded steps to break static friction, then
zeroes `cmd_vel` if that does not free it, and restores the baseline on recovery.

```bash
ros2 topic echo /stall_guard/stalled    # latched Bool
ros2 topic echo /stall_guard/status     # the numbers behind the decision
```

**With no IMU it never fires** — it says `NO IMU … cannot and will not fire` and leaves
`/pid_limits` alone, so an unwired board cannot cause it to raise motor current. Turn it off
with `stall_guard:=false` on `imu.launch.py`, or the whole IMU with `imu:=false` on bringup.

Vibration is measured and published but deliberately does **not** trigger it: measured here, a
rough free floor out-vibrates a genuine pin. `use_vibration:=true` with a measured
`vibration_threshold` enables it if the real robot says otherwise.

Every threshold was tuned against synthetic data. Drive a real floor before trusting them.

## BNO055 IMU — now on by default (accelerometer only)

**`./robotbring.sh` now starts the IMU and the stall guard**, in accelerometer-only mode.
`imu:=false` on bringup skips both; `stall_guard:=false` keeps the IMU but drops the guard.

**Accelerometer only** means orientation is published as unavailable (REP-145
`orientation_covariance[0] = -1`) no matter what the chip's calibration says, and the
magnetometer is not published at all — that is the part that makes an uncalibrated BNO055
heading confidently wrong. Nothing downstream can accidentally trust a heading nobody has
verified. `accel_only:=false` restores the full fused output once you have calibrated it.

```bash
ros2 launch nexva_sensor imu.launch.py                    # driver + guard (defaults)
ros2 launch nexva_sensor imu.launch.py stall_guard:=false # driver only
ros2 launch nexva_bringup bringup.launch.py imu:=false    # neither
ros2 run nexva_sensor imu_monitor                         # live readout, for the bench check
```

Until the board is wired the driver logs what is missing and publishes nothing — it does not
crash, and bringup still comes up. Do this before trusting any of it, in order: wire it per
[circuit.txt](circuit.txt) §3 (4 wires, Pi pins 1/3/5/6) → enable I2C via `raspi-config` and
reboot → confirm `i2cdetect -y 1` shows `28` → `pip3 install --break-system-packages
adafruit-circuitpython-bno055 adafruit-blinka` on the Pi → check the axes with `imu_monitor`
(turning left must raise yaw; fix with `axis_remap:=`) → calibrate all three sensors to 2/3.
The full sequence with the reasoning is in the `imu.launch.py` docstring.

### "imu is not being published"

That message is the driver's **last** step, not its first — you get the identical line whether
the board is unwired, the bus is off, or the libraries are missing. It tells you nothing on its
own. Run the chain instead:

```bash
./tools/check_imu.sh                                              # on the Pi
ssh Scrapify@10.42.0.241 'cd ~/varun/nexva_ws && ./tools/check_imu.sh'   # from the laptop
```

It walks Pi → I2C enabled → i2c-tools → **does anything answer on the bus** → libraries → an
actual read, and stops at the first real failure with the fix for that step. On a laptop it
stops at step 1 and says so: `board` is a Pi-only library, so "No BNO055" there is expected and
not a fault.

**Known cause on this robot:** a completely empty `i2cdetect` scan — every address `--` —
usually means the BNO055 is wired to the **ESP32's** I2C rather than the Pi's. The ROS driver
runs on the Pi, so the board has to be on the Pi's header (pins 1/3/5/6, see
[circuit.txt](circuit.txt) §3). Nothing in software can detect this; from the Pi's side the bus
is simply idle.

Args: `enable` (true), `accel_only` (true), `stall_guard` (true), `frame_id` (default
`base_link`, because the URDF has no `imu_link`), `rate` (50.0), `address` (40 = 0x28),
`axis_remap` (`[1, 2, 3]`), `min_calibration` (2).

The axis check is not optional for the stall guard: it integrates **forward** acceleration, so
if the board's x does not point forward the guard is measuring the wrong direction and its
thresholds are meaningless. `imu_monitor` is how you check it.

## Missions: navigate / explore / clean / manual

`mode_manager` owns what the robot is doing. One mission at a time; switching stops the previous
one and waits for it to be gone before starting the next (a half-dead SLAM still owns
`map -> odom`). Missions are the scripts in `launch/realbot/`, each assuming bringup is already
up. Normally you drive this from the web UI's **Autonomous mission** card, but it is just a topic:

```bash
ros2 topic pub --once /set_robot_mode std_msgs/msg/String \
  '{data: "{\"mode\":\"clean\",\"map\":\"sep23map2\"}"}'
ros2 topic pub --once /set_robot_mode std_msgs/msg/String '{data: "stop"}'
ros2 topic echo /robot_mode      # latched: what is running right now
```

| Mode | Stack it brings up | Map |
|---|---|---|
| `navigate` | Nav2 + AMCL (what `robot.sh` starts by default) | existing |
| `explore` | slam_toolbox + autosaver + `frontier_explorer` | new name |
| `clean` | Nav2 + AMCL + autosaver + `auto_clean` | existing |
| `manual` | slam_toolbox + autosaver (you drive from the joystick) | optional |

A map name is looked up in `~/nexva_maps/<source>/` first, then in
`src/nexva_navigation/maps/` — so `sep23map1`/`sep23map2` work for `clean` and `navigate`
without being copied anywhere. A bad name is refused with the list of what does exist.

While a mission runs, the explorer takes runtime commands of its own:

```bash
ros2 topic pub --once /frontier_explorer/command std_msgs/msg/String '{data: pause}'
ros2 topic echo /frontier_explorer/status_json   # coverage, blocks, sensor ages
```

**Telling the robot where it is.** Two ways to seed AMCL:

- **Set 2D pose** on the map card — drag from where the robot is, in the direction it faces,
  like RViz's 2D Pose Estimate. The spot is checked against the map first and refused if the
  robot could not physically stand there.
- **Seed AMCL here** from a named waypoint, when the robot is parked on a surveyed point.

Do this before sending any goal. An unseeded or wrongly-seeded AMCL does not announce itself —
it just converges somewhere wrong and every goal afterwards misbehaves confusingly.

**Saving a map.** Exploring no longer writes the map to disk continuously — you decide when.

```bash
# the web UI's SAVE MAP button, or by hand:
ros2 topic pub --once /save_map std_msgs/msg/String '{data: "kitchen"}'   # empty = default name
ros2 topic echo /save_map_result                                          # ok / warnings / error
```

One command writes a consistent snapshot: the `.pgm` + `.yaml`, the `map.md` registry entry,
the pose file, and the slam_toolbox pose graph when that service is up. A missing pose graph is
a warning, not a failure — the map is still usable, it just cannot be *resumed* by slam_toolbox
(clean falls back to AMCL + map_updater, which reads a plain `.pgm`).

**When exploring finishes** the robot stops, holds still for 1.5 s, records its 2D pose and
heading, saves the map, and only then starts cleaning. If the save fails it says so in a `!!!`
banner and cleans on the live SLAM map anyway rather than losing the run.

**The pose file** — `~/nexva_maps/<map_name>.pose.yaml`:

```yaml
map_name: kitchen
x: 1.5
y: 0.25
yaw_rad: 0.700
yaw_deg: 40.1
saved_at: 2026-10-05 11:40:02
```

Written atomically and throttled to 1 s, and only while exploring — a clean run never
overwrites your resume point. A `clean` mission seeds AMCL from it instead of guessing, so the
robot starts localised where it stopped. It refuses to seed (and warns) if the file is missing,
corrupt, names a different map, or is more than 10 minutes older than the map itself.
`seed_pose:=false` turns the seeding off.

**Stopping.** The web STOP button and `{"mode":"stop"}` both end the mission AND publish a burst
of zero velocities — `mode_manager` does not trust the thing it just killed to have stopped the
wheels on the way out. Restarting `./web.sh` does **not** stop a mission: mission lifetime
belongs to `mode_manager` precisely so a clean survives the UI going away.

## Tuning the ESP32 motor controller live (no reflash)

Both topics are `geometry_msgs/Vector3`; a value `<= 0` leaves that field unchanged, so any
one number can be changed on its own. Bringup must be up (the micro-ROS agent carries these
to the ESP32).

```bash
# PWM band and speed cap: x = min PWM, y = max PWM, z = max linear m/s
ros2 topic pub --once /pid_limits geometry_msgs/msg/Vector3 "{x: 120, y: 200, z: 0.22}"
ros2 topic pub --once /pid_limits geometry_msgs/msg/Vector3 "{x: -1, y: -1, z: 0.15}"   # speed only

# PI gains: x = Kp, y = Ki, z = feedforward breakaway PWM
ros2 topic pub --once /pid_gains  geometry_msgs/msg/Vector3 "{x: -1, y: -1, z: 160}"

# watch the effect
ros2 topic echo /motor_pwm      # applied PWM per wheel
ros2 topic echo /wheel_vel      # measured m/s per wheel
```

The web UI's "Motor limits" card does the `/pid_limits` half with validation, and keeps
resending the values every 5 s so an ESP32 reboot does not silently revert them. If you raise
`min PWM`, also raise the breakaway (`z` on `/pid_gains`) to match, or the feedforward starts
below the new floor and the clamp does the work the integrator should.

## Manual / not-launch-wired

Run directly with `ros2 run`, not part of any launch file:

- `ros2 run nexva_sensor cam` — low-latency `/dev/video0` camera publisher (`/vivo_camera/image_raw`). Not in bringup.
- `ros2 run nexva_sensor imu_monitor` — live BNO055 readout from `/imu`, safe to run from the laptop. Prints a waiting screen if the driver is off.
- `ros2 run nexva_web waypoint_cli <cmd>` — terminal equivalent of the web UI (`list`, `check`, `validate`, `init`, `goto`, `tour`). Useful to validate waypoints/map/initial-pose without the browser.
- `ros2 run nexva_frimware esp32_interface` — old/legacy encoder logger node, superseded by wheel_joint_publisher + wheel_odometry. Not wired anywhere.
- `ros2 run nexva_description esp32_rviz_control esp32_bridge` — older direct-serial (`/dev/ttyUSB0`) ESP32 bridge, alternate to the current micro-ROS pipeline. Not wired anywhere.
- `nexva_navigation/waypoint_follower.py` — standalone interactive goal-sender CLI. **Orphaned**: no console_script entry point registered in `setup.py`, so `ros2 run` won't find it as-is.

## RViz

`nav(1).rviz` (workspace root) — general real-robot viz: Grid, RobotModel, LaserScan, PointCloud2,
Map, TF, TopDownOrtho view. Open with:
```bash
rviz2 -d "/home/varun/nexvabuilds/rasp/nexva_ws/nav(1).rviz"
```

## Not currently integrated

- `robot_zigzag_ui/` (React+Konva app, root level) expects a `rosbridge_server` on
  `ws://localhost:9090` and a different topic set (`/selected_area`, `/custom_path`,
  `/cleaning_status`, …) than what any package here publishes. It is a disconnected prototype UI,
  not part of any launch sequence. The active web UI is `nexva_web` (port 8080).
