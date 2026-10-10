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

`.github/workflows/build.yml` runs `tests/run_tests.sh` (with g++) and `ci/build_with_vex_sdk.sh` on every PR. `ci/build_with_vex_sdk.sh`
additionally needs `clang` (it defaults to `CXX=clang++` and cross-compiles with `-target`, so g++ won't
do), `curl`, `unzip`, `python3`, and `binutils-arm-none-eabi`. `ci/build_local_windows.sh` needs none of
those: it points `build_with_vex_sdk.sh` at the extension's installed SDK (`VEX_SDK_HOME`) and its clang
8 / `arm-none-eabi-ld` (`CXX`, `LD`, `OBJCOPY`), the same compiler VEXcode itself uses.

There is no test framework and no per-test selection: `tests/simulation_test.cpp` is a single `main()`
of `check(name, ok, a, b)` calls that print PASS/FAIL and exit with the failure count. To run one
case, comment out the others or build that file directly:

```sh
g++ -std=c++17 -O1 -pthread -I include -I tests/sim src/*.cpp tests/simulation_test.cpp -o build/t && ./build/t
```

`tests/run_tests.sh` runs the suite **twice**: once with `include/simpleV5LibConfig.h` as-is, and once
against a `sed`-patched copy in `build/tracking_include/` that enables tracking wheels and all four distance sensors. The script
greps the patched file and fails loudly if the `sed` didn't match — so renaming or reformatting the
`TRACKING_*` or `DISTANCE_*` constants in the config breaks the second run and the script must be updated too.

New tests are mutation-checked: break the code on purpose and confirm the test fails (the first tests for
chained drive-to-point missed 5 of 6 such mutations). Timing traps in the cooperative simulator:
- To test the per-movement gain snapshot, change the gains right after the `_async` call; any later and
  the movement has already copied them.
- Back-to-back chained calls never reach `motionLoop`'s idle branch. To test something that runs only
  while idle (SD-card writes), put user code such as `vexDelay(40)` between them.
- A chained movement hands over while the robot is still rolling, so compute expected values (e.g. a
  `turn_to_point_chain` heading) from the pose at hand-over, not from before the previous movement.

If a branch has no PR yet, CI can be run on it by triggering `build.yml` manually (`workflow_dispatch`).

## The two `vex.h` shims

The library only ever includes `"vex.h"`, which it never ships. Both builds supply their own:

- `tests/sim/vex.h` — a ~200-line drivetrain simulator standing in for the whole VEX SDK: motors with
  a first-order speed response, an inertial sensor derived from wheel speeds, rotation sensors as
  tracking wheels, distance sensors that measure to a 144-inch square of walls around (0, 0),
  simulated time (tests run far faster than real time), and cooperative "tasks" that are real threads
  but only switch inside `vexDelay`/`wait`. It can inject setup faults (`sim::unplugged`, `wrong_direction`, `sides_swapped`, `gyro_rate_flipped`, `wheel_diam`) which is
  how `src/robotSetup.cpp` is tested.
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
- `startMotion()` claims the drivetrain with a `compare_exchange` spin on `motion_running`, lazily
  creates the single `motionLoop` task, and publishes the request via `motion_requested`.
- `motionLoop()` picks it up, dispatches through `runMotion()`, and clears `motion_running`.
- The public API is three thin layers over this: `PID_x_async` = `startMotion`; `PID_x` = async +
  `waitUntilDone`; `PID_x_chain` = the same with a non-zero `exit_range`. The two `*_to_point*` turns
  are the exception: they resolve the point to a heading in the *caller's* task (so they must
  `waitUntilDone` first) and then delegate to the ordinary turn.
- Cross-task state is all `std::atomic` (`motion_running`, `motion_requested`, `cancel_requested`,
  `motion_progress`, and the `chain_*` values). `last_result` is plain, safe only because it is written
  before `motion_running` goes false and read after waiting on it.

Only three inner loops exist. `driveForward()` handles `PID_forward` (and heading-hold straightness via
`FORWARD_HEADING_KP`), `driveToPoint()` handles `PID_drive_to_point`, and `turnToHeading()` handles
*everything that ends at a heading* — point turns, swings and arcs — differing only in `turnStyle` and
how power reaches the wheels. Adding a new heading-based movement means a `turnStyle` case, not a new loop.

Each loop has a `chaining` branch (`exit_range > 0`) alongside its `SettleCheck` branch. `driveToPoint`'s
also requires the robot to roughly face its aim before handing over, or a point beside the robot would
read as "0 inches ahead" and the movement would end before it started. `runMotion` additionally lets a
chained drive-to-point *curve* into its new direction (while the turn needed is within
`POINT_CURVE_ANGLE`) instead of inserting the turn-first step, so a chain of points never stops mid-path.

**Chaining** is why so much state is global: a chained movement returns with the robot still moving,
so the next one must know the heading that was being aimed for (`chain_heading`), whether the previous
was a forward move and where its target was (`chain_was_forward`, `chain_forward_end`, so the handed-over
inches aren't lost), and when it ended (`chain_end_time`, so `motionLoop` can stop the robot after
`CHAIN_STOP_AFTER_MS` if nothing follows).

**Odometry** (`src/odometry.cpp`) is a separate 10 ms background task doing arc-based dead reckoning:
per-tick forward/sideways deltas, tracking-wheel offset compensation, chord correction, rotated into
field coordinates by the mid-tick heading. `setPose`/`setHeading` bump a `manual_changes` counter
(not a flag, so a change can't be lost) and the loop restarts its baseline when it sees it change.
`setX`/`setY` (and `resetXFromWall`/`resetYFromWall`, which compute the value from a configured distance
sensor, with an angle and a max-change guard) only overwrite one coordinate and need no baseline restart.
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
