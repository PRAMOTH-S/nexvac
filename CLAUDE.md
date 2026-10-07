# Nexva robot vacuum — working agreement

Read this before changing anything. It is short on purpose.

## Who works here

**Varun** and one colleague share this workspace and one physical robot
(`Scrapify@10.42.0.241:~/varun/nexva_ws`). Assume the other person may be using
the robot right now.

## After every change: update Docs/UPDATE.md

**This is not optional.** Add an entry at the TOP of
[Docs/UPDATE.md](Docs/UPDATE.md) for anything that changes behaviour, and
**always name who made it** (ask the user if you do not know — do not guess, and
do not write "Claude" alone; it is the person's change, made with your help):

```
## YYYY-MM-DD — <one line summary>
**By:** <person's name> (with <assistant>)
**Changed:** <packages / files>
**You need to:** <rebuild? restart? nothing?>
<2-4 lines on what is different now>
```

Keep it short. Deep reasoning, measurements and rejected approaches go in
[Docs/BUILD_LOG.md](Docs/BUILD_LOG.md) instead. UPDATE.md answers "what do I
need to know to keep working", for a colleague who was not here.

## The docs, and what each is for

| File | Use it for |
|---|---|
| `Docs/UPDATE.md` | what changed, by whom, what to do about it |
| `Docs/BUILD_LOG.md` | why it changed: evidence, measurements, dead ends |
| `Docs/LAUNCHES.md` | how to run everything |
| `Docs/ARCHITECTURE.md` | what each package and node does |
| `Docs/WIRING.md` | physical connections — pins come from the firmware |
| `Docs/circuit.txt` | **historical, partly wrong.** Wire from WIRING.md |

## Running it

```
terminal 1:  ./robotbring.sh     bringup: ESP32, odometry, lidar
terminal 2:  ./robotnav.sh       Nav2 + mode manager
terminal 3:  ./web.sh            browser UI, port 8080
./build.sh [pkg...]              build (incremental, --symlink-install)
./push.sh                        laptop -> Pi, changed packages only
```

`./robot.sh` starts nothing — it was split in two and now only prints guidance.

## Rules that exist because something went wrong

- **Never `kill -9` the lidar.** It leaves `/dev/ttyUSB0` locked and the next
  `rplidar_node` starts but publishes nothing — a silent `/scan` with a healthy
  log. Use SIGINT, which is what Ctrl-C in `robotbring.sh` does.
- **Nothing may write `/pid_limits` automatically.** A node that raises the
  ESP32's PWM floor and is killed before restoring it leaves the firmware
  driving the wheels with nothing publishing `/cmd_vel`. This already happened.
- **Pin numbers come from `Testing/ESP/microros_code/microros_code.ino`**, not
  from prose. `circuit.txt` disagrees with it on the left-motor direction pins
  and on all four encoder pins.
- **Never send `build/`, `install/` or `log/` to the Pi.** It is aarch64, the
  laptop is x86_64; copied artifacts fail to link in confusing ways.
- **A TF lookup at `Time()` returns the last transform of a dead publisher
  forever.** If you check a transform for liveness, check its stamp age.
- **`/scan` is BEST_EFFORT.** A RELIABLE subscriber never connects to it, with
  no error anywhere.

## Before touching the real robot

- It drives real motors. Prefer the simulator in `simluationsequnce/` for
  behaviour changes, and say clearly what you have NOT verified on hardware.
- Ask before commanding motion you cannot see the result of.
- Leave no stray processes. Check with `pgrep` when you finish.
- The IMU is accelerometer-only and currently unreliable (I2C read failures,
  uncalibrated). Do not let anything act on it without saying so.
