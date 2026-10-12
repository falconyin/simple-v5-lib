# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

`simple-v5-lib` is a VEX V5 C++ library (PID autonomous driving, odometry, driver control) that
users install by **copying `src/` and `include/` into their own VEXcode V5 project**. There is no
package manager, no build system for consumers, and no `main()` in the library itself — `examples/*/main.cpp`
provide one for the CI link step and as user-facing templates.

An explicit goal (see README "Why simple-v5-lib?") is that a beginner can *read* the code: plain
language, simple logic, and comments that explain the reasoning. `docs/how-it-works.md` is the
narrative counterpart and is kept in sync with the code section by section. When changing behaviour,
update the matching section of that doc, the README quick reference, and `include/simpleV5lib.h`'s
doc comments — these are treated as part of the deliverable, not optional extras.

More user-facing docs depend on behaviour: `docs/first-robot-checklist.md` (the real-robot
steps; keep it in sync when a sim-only constant changes), `docs/odometry-and-paths.md` (how to
use odometry, poses, chains and wall resets, with measured simulator numbers such as the
`exit_range` table and the short-pose miss), `docs/tuning.md` (Brain-screen graphs of each tuning
mistake, and tables of time/overshoot/end error per gain), `docs/writing-autonomous.md` (putting a
routine together, walking through `examples/autonomous` and its mirroring helpers) and
`docs/troubleshooting.md` (symptom tables that quote the library's printed messages word for word:
change a message, change the quote). Their pictures in `docs/images` are generated:
`docs/diagrams/diagrams.cpp` drives the library on the simulator, draws the paths as SVG (and the tuning
graphs from every line `graphOnScreen` drew on the simulator's Brain screen), and prints the guides' numbers
(`CXX=... docs/diagrams/make_diagrams.sh`). It `#include`s `examples/autonomous/main.cpp` (with its `main`
renamed) and drives that example's own `leftSpot()`/`rightSpot()`, so the autonomous guide's paths and
times follow the example. The picture's step labels and the points its numbers measure against are
copies of the example's coordinates (constants at the top of `autonomousExample`), and the guide's
walk-through copies the routine: update both with the example.
Rerun it after changing how point/pose driving, chaining,
the PID or the turn/forward loops behave, a gain in the config, or `examples/autonomous`, look at the pictures, and compare the
printed numbers with the guides. Each number is a range over 10 runs started 0–9 ms apart: where a run starts between the
library's 10 ms ticks can change a chained corner a lot, so a single run can't be trusted. The carrot
in `pose-carrot.svg` comes from `poseCarrot()`, the part of `aimPoint` that is not `static` for this
reason. `tests/run_tests.sh` builds `diagrams.cpp` (without running it), so a change that breaks it fails there.

## Commits

Do **not** add AI attribution to commits or pull requests in this repo: no `Co-Authored-By: Claude`,
no `Claude-Session:` trailer, no "Generated with Claude Code" line in PR descriptions. This overrides
Claude Code's default attribution behaviour. Commits are authored as
`falconyin <1156029+falconyin@users.noreply.github.com>`.

## Commands

```sh
tests/run_tests.sh            # build against the simulator and run all tests (needs C++17 g++/clang++)
CXX=clang++ tests/run_tests.sh
ci/build_with_vex_sdk.sh      # download the real VEXcode V5 SDK, compile for ARM, link every example
VEX_SDK_VERSION=V5_20240802_15_00_00 ci/build_with_vex_sdk.sh
ci/build_local_windows.sh     # the same SDK build on Windows (Git Bash), with the VEX VS Code extension's SDK and tools
```

`.github/workflows/build.yml` runs `tests/run_tests.sh` (with g++) and `ci/build_with_vex_sdk.sh` on every PR, with the SDK
version pinned in its `VEX_SDK_VERSION` (a manual run can try another one). `ci/build_with_vex_sdk.sh`
additionally needs `clang` (it defaults to `CXX=clang++` and cross-compiles with `-target`, so g++ won't
do), `curl`, `unzip`, `python3`, and `binutils-arm-none-eabi`. `ci/build_local_windows.sh` needs none of
those: it points `build_with_vex_sdk.sh` at the extension's installed SDK (`VEX_SDK_HOME`) and its clang
8 / `arm-none-eabi-ld` (`CXX`, `LD`, `OBJCOPY`), the same compiler VEXcode itself uses.

How tests are structured and written (test order, running one group, `-DSIM_REALISTIC` consequences,
mutation checks, simulator timing traps) is in `tests/CLAUDE.md`.

`tests/run_tests.sh` runs the suite **four times**: with `include/simpleV5LibConfig.h` as-is, and
against a `sed`-patched copy in `build/tracking_include/` that enables tracking wheels and all four distance sensors,
each on the perfect simulator and again with `-DSIM_REALISTIC` (see `tests/CLAUDE.md`). The script
greps the patched file and fails loudly if the `sed` didn't match — so renaming or reformatting the
`TRACKING_*` or `DISTANCE_*` constants in the config breaks the tracking runs and the script must be updated too.

If a branch has no PR yet, CI can be run on it by triggering `build.yml` manually (`workflow_dispatch`).

## The two `vex.h` shims

The library only ever includes `"vex.h"`, which it never ships. Both builds supply their own:

- `tests/sim/vex.h` — a ~300-line drivetrain simulator standing in for the whole VEX SDK: motors with
  a first-order speed response, an inertial sensor derived from wheel speeds, rotation sensors as
  tracking wheels, distance sensors that measure to a 144-inch square of walls around (0, 0),
  independent mechanism motors on ports 7–21 (a gravity `mech_load`, hold mode that pushes back,
  `mech_jammed`, current draw from the power-vs-speed gap; ports 1–6 are always the drivetrain),
  simulated time (tests run far faster than real time), and cooperative "tasks" that are real threads
  but only switch inside `vexDelay`/`wait`; each remembers the function it was started with, so
  `task::_index` and stopping a task by its function work. Its `competition` copies what the real SDK
  (V5_20240802, decoded from `libv5rt.a`, not documented by VEX) does on a mode change: stop the task running
  the other mode's function (`bStopTasksBetweenModes`, on by default), and with `bStopAllTasksBetweenModes`
  being disabled (only that, not starting a mode) stops every task except main; `test_auton`/`test_driver`/`test_disable`
  switch modes. A stopped task stays stuck in the `vexDelay` it was in. While disabled (`sim::disabled`) the motors
  get no power and coast; enabled again, they go back to the program's last commands (a guess, not checked on a
  real Brain). It can inject setup faults (`sim::unplugged`, `wrong_direction`, `sides_swapped`, `gyro_rate_flipped`, `wheel_diam`) which is
  how `src/robotSetup.cpp` is tested, and disturbances (`gain` for a weak side, `bump`, `push`, `obstacle`,
  `heat`). Its real-world layer (`sim::real`: delays, noise, drift, battery, grip) is off unless
  `-DSIM_REALISTIC`; `sim::real::friction` (a power deadband on the drive) is not part of it and only
  `docs/diagrams` sets it, to show a robot stopping short; `sim::v` is the robot's true speed and `sim::wheel_v` the wheels', they differ while skidding, and
  `sim::slipped` adds up the forward/backward part of that difference (what drive-motor odometry gets wrong; a
  symmetric turn-in-place skid cancels out). A drive motor's `velocity(pct)` is the wheel speed as a percent of
  `MAXV`, like a real motor's percent of its cartridge speed. A `screen` keeps what was printed since its last
  `clearScreen()` in `text`, so tests can check which message the library showed, and every `drawLine`/`printAt`
  with its pen color in `drawn`, which is what the tuning pictures are drawn from.
- `ci/vex.h` — a copy of the `vex.h` a fresh VEXcode project generates, used only so the SDK build
  compiles. Consumers already have this file.

Consequences for any new code: if you use a VEX API the simulator lacks, add it to `tests/sim/vex.h`.
The SDK build compiles with `-std=gnu++11 -fno-exceptions -fno-rtti` and treats warnings seriously,
so C++14/17 features, exceptions, and RTTI are off limits in `src/` even though the test build uses
C++17. VEXcode's `vex.h` `#define`s `repeat` and `waitUntil`, so neither may be used as a name (examples
use a plain `while` loop instead of `waitUntil`). The real SDK has a `Brain.Screen.printAt` overload with
an extra `bool` (opaque) argument, which makes `printAt(x, y, "fmt", args...)` ambiguous: write
`printAt(x, y, true, "fmt", ...)`. The simulator declares both overloads too, so `CXX=clang++
tests/run_tests.sh` and the SDK build reject the ambiguous call, but CI's default g++ simulator build
only warns about it.

## Architecture

**Configuration is compile-time.** `include/simpleV5LibConfig.h` is all `const` globals (ports, gear
ratios, directions, PID gains, timeouts, tolerances, robot geometry). Users edit this file. The
runtime-mutable parts are the `PIDGains` globals (`turnGains`, `swingGains`, `forwardGains`, `arcGains`)
which are *initialised* from the config and can be changed in user code or by `tuneWithController()`.

**Devices are global objects** constructed in `src/simpleV5lib.cpp` (`Brain`, `Controller`, the six
drive motors, `leftDrive`/`rightDrive` motor groups, `Inertial`). The library owns `Brain` and
`Controller`, which is why users must delete their own `brain Brain;`.

**One motion task owns the drivetrain.** `src/simpleV5lib.cpp` is the core:

- Every movement is a `MotionRequest` struct (type, target, tolerances, timeout, max speed, radius,
  `exit_range`, target point, flags, plus a *snapshot of all four gain sets*).
- `startMotion()` claims the drivetrain with a `compare_exchange` spin on `motion_running` and
  publishes the request via `motion_requested`; the single `motionLoop` task is started by
  `keepMotionTaskRunning()` (see "Background tasks" below).
- `motionLoop()` picks it up, dispatches through `runMotion()`, and clears `motion_running`.
- The public API is three thin layers over this: `PID_x_async` = `startMotion`; `PID_x` = async +
  `waitUntilDone`; `PID_x_chain` = the same with a non-zero `exit_range`. The two `*_to_point*` turns
  are the exception: they resolve the point to a heading in the *caller's* task (so they must
  `waitUntilDone` first) and then delegate to the ordinary turn.
- Cross-task state is all `std::atomic` (`motion_running`, `motion_requested`, `cancel_requested`,
  `motion_progress`, and the `chain_*` values). `last_result` is plain, safe only because it is written
  before `motion_running` goes false and read after waiting on it.

**Background tasks** (motion, odometry, mechanisms, auton selector) are all started through
`startTaskOnce(fn)` in `src/simpleV5lib.cpp`, which asks the SDK whether a task runs `fn` (`task::_index`,
protected, reached through a derived `TaskFinder`) and starts one if not. They are started lazily, and every
function that relies on one calls it again first, because `Competition.bStopAllTasksBetweenModes = true`
makes the SDK stop every task except main when the robot is disabled: the motion task's callers (`startMotion`,
`waitUntilDone`, `waitUntilTraveled`, `isMoving`, `cancelMovement`) go through `keepMotionTaskRunning()`,
which also stops the robot and frees `motion_running` when the task had to be restarted (its movement can
never finish); odometry through `keepOdometryRunning()` (getters and `cancelMovement`, which driver control
calls every loop; the loop's last reading is file-static, so a restarted loop counts what the robot rolled
meanwhile); Arm/Intake commands *before* their repeated-command check (a driver loop repeats
autonomous's last command; a `moveTo` that gave up is not a repeat, so it tries again). The selector is the
exception: only `startAutonSelector()` starts it, so a disable during pre-auton stops it for good (documented).
A new background task, or a new function that waits on one, must do the same; `testCompetitionModes` plays a
match with and without that switch, disabled for longer than `ARM_TIMEOUT_MS`. The default
(`bStopTasksBetweenModes`) only stops the task running the user's `autonomous`/`usercontrol` function.

Only three inner loops exist. `driveForward()` handles `PID_forward` (and heading-hold straightness via
`FORWARD_HEADING_KP`), `driveToPoint()` handles `PID_drive_to_point` *and* `PID_drive_to_pose` (the
same loop; `aimPoint()` returns the point, or for a pose the boomerang "carrot" `lead × distance` behind
the target along the target heading, pulled forward near that line to stay 2× `POINT_AIM_DISTANCE` ahead of the
robot but never past the target (without it a big lead weaves and ends inches off, and lead 1 stalls with the carrot
on the robot; the cost: a pose about a foot away and well beside its line has no room left to curve and ends
inches off, and shorter look-aheads traded that for worse results on ordinary poses), and `aimHeading()` slides a pose's aim over to the target heading itself
between 2× and 1× `POINT_AIM_DISTANCE`; a sudden switch there needed a brake harder than traction control allows), and `turnToHeading()` handles
*everything that ends at a heading* — point turns, swings and arcs — differing only in `turnStyle` and
how power reaches the wheels. Adding a new heading-based movement means a `turnStyle` case, not a new loop.

Each loop has a `chaining` branch (`exit_range > 0`) alongside its `SettleCheck` branch. `driveToPoint`'s
also requires the robot to roughly face its aim before handing over, or a point beside the robot would
read as "0 inches ahead" and the movement would end before it started. `runMotion` additionally lets a
chained drive-to-point *curve* into its new direction (while the turn needed is within
`POINT_CURVE_ANGLE`, 120°) instead of inserting the turn-first step, so a chain of points never stops mid-path.
Inside `driveToPoint` the steering has three rules that came out of the realistic simulator, keep them
together: a D term on the heading (`POINT_HEADING_KD`, or the robot swings past a moving aim and the hard
correction skids), "steering first" (the correction is capped so the inside wheel never reverses, and the
drive power gives way to it, so hard steering means slower driving), and `CORNER_POWER` through a chained
corner of more than 90° (there `cos(aim error)` would cut the drive power to zero and the PID sees the point
as "behind"; the robot rolls through at `CORNER_POWER` in its travel direction instead). The 90° square
chained-points test is sensitive to all three: its second corner's turn angle lands right around 90–110°
depending on how the first corner was exited.

**Traction control** sits under all three loops: they drive through `driveWithGrip()` (or `gripPower()`
for a swing's one side), which keeps each side's power within `DRIVE_SLIP_LIMIT` of that side's wheel speed
(`velocity(pct)`), widening by `SLIP_LIMIT_WIDENS_PER_S` from the last power so a stalled robot (pushing a
wall) still reaches full power. It is what removed the realistic simulator's slip-driven odometry errors.
The public `move()` and driver control bypass it on purpose; `spinSide()`/`stopSide()` record each side's
last power (`side_power`), which `lastDrivePower()` averages for `rampFrom`/`slowDown`. Because braking is
limited too, a movement can no longer rely on an instant hard correction: the pose aim blend above exists
for that reason, and a new steering rule should be checked on the perfect *and* realistic runs.

**The speed profile** (optional, `DRIVE_ACCELERATION` is 0 by default) lives in `driveForward` and
`driveToPoint` only: a `Ghost` (made by `makeGhost`, which reads the robot's speed *only* when the profile is
on, so with it off every run is byte-identical to the PID alone, realistic noise sequence included) speeds
up at the acceleration, cruises at `max_speed` × top speed and slows down by `sqrt(2·a·distance left)`. The
loop gives `Ghost::power()` (feedforward for the ghost's speed `response_ms` ahead, extrapolated below 0 when
the ghost stops within that time, which is what keeps long drives from overshooting) plus the PID on the
distance to the ghost, its D on the speed difference, instead of `startRamp`. When the ghost stops the PID
alone finishes. A chained ghost stops at the PID's target (`exit_range` past), and a new ghost starts at the
robot's speed. `speedProfile` (`SpeedProfileSettings`: acceleration, top speed, response time) is snapshotted
into the request like the gains. `measureTopSpeed()` measures the other two (63% rise time, so it includes
the realistic simulator's 20 ms of delays: 119 ms there vs 99 perfect). Measured on the simulator it helps
mostly on the realistic runs (less overshoot, shorter long drives); the robot brakes up to ~1.4× (realistic
1.8×) the acceleration at the very end, since the look-ahead starts braking early and gently; tests allow it.
The gain-semantic tests (all gains 0 doesn't move, kD 0 overshoots more) would fail with the profile on.

**Chaining** is why so much state is global: a chained movement returns with the robot still moving,
so the next one must know the heading that was being aimed for (`chain_heading`), whether the previous
was a forward move and where its target was (`chain_was_forward`, `chain_forward_end`, so the handed-over
inches aren't lost), and when it ended (`chain_end_time`, so `motionLoop` can stop the robot after
`CHAIN_STOP_AFTER_MS` if nothing follows).

**Odometry** (`src/odometry.cpp`) is a separate 10 ms background task doing arc-based dead reckoning:
per-tick forward/sideways deltas, tracking-wheel offset compensation, chord correction, rotated into
field coordinates by the mid-tick heading. `setPose`/`setHeading` bump a `manual_changes` counter
(not a flag, so a change can't be lost) and the loop restarts its baseline when it sees it change.
`setX`/`setY` only overwrite one coordinate and need no baseline restart. `resetXFromWall`/`resetYFromWall`
compute that value from a configured distance sensor, and refuse while moving, too close, with the wall
behind the sensor, too angled, or with too big a change. The simulator's `distance::installed()` is
true only on ports with a `sim::distance_mount`, so checkDevices can't probe other devices as distance sensors.
Heading always comes from the inertial sensor, never from integration.

**Sign conventions, easy to get wrong:**
- Headings are degrees, **clockwise positive**, and `getInertial()` keeps counting past 360 (`PID_turn(450)`
  really turns 450°). `PID_turn_shortest` is the wrapping variant.
- `getGyroRate()` negates the IMU's z-axis rate, which is counter-clockwise positive. If turns start
  oscillating after an SDK update, check this sign first.
- The PID D term is `-kd * rate` (a brake), not `kd * (error - past_error)`.
- Field coordinates: x right, y forward along heading 0.

The remaining files are leaf features with no dependents: `driverControl.cpp` (tank/arcade with deadband
and curve; both call `cancelMovement()`), `telemetry.cpp` (terminal CSV + 480×240 Brain-screen graph,
driven by `telemetryStart`/`telemetryUpdate` calls inside the motion loops), `controllerTuner.cpp`
(`tuneWithController()`), `autonSelector.cpp` (background selector task, max 10 routines),
`robotSetup.cpp` (`checkDevices`, `testDrivetrain`, `measureTrackWidth`, `measureWheelSize` — all report
to Brain screen, controller and terminal; `measureTrackWidth` consumes `measureWheelSize`'s in-run result).

**Mechanisms** (`src/mechanisms.cpp`): `Arm` and `Intake` are classes the user constructs globally around
their own `motor` or `motor_group` (a `motor` is wrapped in an owned `motor_group`). Constructors only
register the object (max 8 of each); the first command lazily starts one shared 10 ms task that calls
every object's `update()`. User-facing methods never touch the motors: they write atomics and bump a
`request` counter, and `update()` starts the new command when the counter changes, so the task is the
only motor writer. A command identical to the last one is ignored (driver loops repeat them every 20 ms).
`Arm` moves with its own `PIDController`, then hands off to `brakeType::hold`; `manual(0)` only holds
after a manual command, so it doesn't cancel a running `moveTo`. `Intake` jam = per-motor current over
`INTAKE_JAM_CURRENT` *and* speed under `INTAKE_JAM_SPEED`, for `INTAKE_JAM_MS`; each condition is
covered by its own test.

**SD card** (`src/sdCard.cpp`, plus the `logToSDCard` part of `telemetry.cpp`):
- `saveGainsToSDCard`/`loadGainsFromSDCard` write one line per gain with the config value it was saved
  against (`TURN_KP = 3.5 config 3.2`). On load, a gain whose config value has changed since keeps the
  config value, so an old file on the card can't silently override an edit to the config. A file over
  2 KB is ignored entirely, since a truncated line could lose its `config` part and bypass that rule.
  `tuneWithController()` saves when B is pressed.
- `logToSDCard` lines are formatted into a 256-byte temp (a line that doesn't fit is dropped) and kept in
  a 32 KB RAM buffer. Card writes are slow and would stall a PID loop, so `motionLoop` calls
  `telemetryWriteSDCard()` while idle *and* no chained movement is pending. The one exception: if a long
  movement fills the buffer, `telemetryUpdate()` writes it out immediately, stalling that movement's
  PID loop for the length of the write. One failed write stops
  logging, including for the current movement. Log files take the first free number, not the highest
  (finding the highest would mean an `exists()` per possible number).
