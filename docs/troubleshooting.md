# Troubleshooting

Find what you see, then the most likely cause and what to do. The sections follow a program's life:
building it, starting it, autonomous, the movements themselves, driver control, the mechanisms and
the SD card. Two other guides have longer tables for their own topics:
[Tuning the PID, section 11](tuning.md#11-what-you-see-what-to-change) (overshoot, swinging, stopping
short) and [Odometry and paths, section 7](odometry-and-paths.md#7-when-something-goes-wrong)
(position, points, wall resets).

Many problems print a message: on the Brain screen, on the controller, or in the **terminal** (connect
the Brain or the controller to the computer with a USB cable and open the terminal in VEXcode). The
messages are quoted here the way the library prints them.

- [1. It doesn't build](#1-it-doesnt-build)
- [2. When the program starts](#2-when-the-program-starts)
- [3. In autonomous](#3-in-autonomous)
- [4. Movements](#4-movements)
- [5. Driver control](#5-driver-control)
- [6. Arm and intake](#6-arm-and-intake)
- [7. SD card](#7-sd-card)
- [8. Still stuck?](#8-still-stuck)

---

## 1. It doesn't build

| The build says | Most likely | What to do |
| --- | --- | --- |
| ``multiple definition of `Brain'`` (or `Controller`, `leftFront`, `leftDrive`, `Inertial`, ...) | Your project makes a device the library already makes | Delete your line (`brain Brain;` in main.cpp, or in robot-config.cpp if your project has one), and use the library's: `Brain`, `Controller`, the six drive motors, `leftDrive` / `rightDrive` and `Inertial` |
| `use of undeclared identifier 'PID_forward'` (or any other library name) | main.cpp doesn't include the library | `#include "simpleV5lib.h"` near the top of main.cpp, after `#include "vex.h"` |
| `'simpleV5lib.h' file not found` | The headers are in the wrong folder | Both files from `include` go in your project's `include` folder |
| `undefined reference to 'PID_forward(...)'` (or any other library name) | The library's `.cpp` files aren't in the project | Every file from `src` goes in your project's `src` folder |
| An error pointing into `simpleV5LibConfig.h` | A typo in a line you changed | Every line ends with `;`, ports are written `PORT1` to `PORT21`, cartridges `ratio36_1`, `ratio18_1` or `ratio6_1` |
| `call to member function 'printAt' is ambiguous` | `Brain.Screen.printAt(10, 40, "%s", name)`: the first value printed is text, and the SDK has a second `printAt` that takes a `true` or `false` there | Add `true` after the position: `Brain.Screen.printAt(10, 40, true, "%s", name)` |

## 2. When the program starts

| What you see | Most likely | What to do |
| --- | --- | --- |
| The controller rumbles and shows `2 PROBLEMS` (or `1 PROBLEM`) | `checkDevices()` found something | The Brain screen lists it: `No motor on port 3 (left back)`, `Port 7 is used twice: ...`, `left front motor is hot (55 C): it loses power`. Fix the port in the config or the cable, or let a hot motor cool down |
| The autonomous selector doesn't show | `startAutonSelector()` before any `addAuton`, or not called | `addAuton` once per routine, then `startAutonSelector()` at the end of `pre_auton` |
| A routine is missing from the selector | More than 10 routines | Only the first 10 `addAuton`s count |
| The selector stops answering taps and buttons before the match | `Competition.bStopAllTasksBetweenModes = true`, and the robot was disabled while you were choosing (a field cable or competition switch plugged in): that stops the selector for good | Leave `bStopAllTasksBetweenModes` off (the default), or start the program again |
| The heading slowly changes while the robot stands still, or is off from the start | The robot moved while `calibrateInertial()` ran | Keep the robot completely still for those 2 seconds; restart the program to calibrate again |
| The gains aren't the ones the tuner saved | `loadGainsFromSDCard()` didn't load them: the terminal says why | `no SD card`, `no pid_gains.txt on the SD card yet`, `is too big`, or for one gain `from simpleV5LibConfig.h (it changed since ... was saved)`: you changed that gain in the config, and the config wins ([Tuning the PID, section 8](tuning.md#8-keeping-the-gains)) |

## 3. In autonomous

| What you see | Most likely | What to do |
| --- | --- | --- |
| Nothing happens | The wrong routine is selected ("Do nothing"?), or `autonomous()` doesn't call `runSelectedAuton()` | Check the selector before the match. `autonomous()` only runs when a competition switch or the field starts it, and needs `Competition.autonomous(autonomous)` in `main` |
| The first turn goes to the wrong heading, or the first drive to a point goes the wrong way | The routine doesn't start with `setPose` (or `setHeading`), and the robot was moved or turned after calibrating | `setPose(0, 0, 0)` as the first line of every routine ([Writing autonomous, section 2](writing-autonomous.md#2-plan-it-on-paper-first)) |
| The routine ends in a different place every run | The starting spot isn't exactly the same; or the battery; or wheels slipping | Line the robot up the same way every time (against the wall, on a tile line). Test with a charged battery. Wheel slip: tracking wheels, wall resets ([Odometry and paths, section 6](odometry-and-paths.md#6-keeping-the-position-right-walls)) |
| It stops in the middle and stands still for the rest | A loop of yours waits for something that never happens | Give every wait a time limit ([Writing autonomous, section 6](writing-autonomous.md#6-doing-things-on-the-way)). The library's own waits all have one |
| It loses seconds standing against something | A movement is blocked, and waits out its timeout (5 seconds for a drive) | Pass a shorter timeout to movements that can get blocked ([Writing autonomous, section 5](writing-autonomous.md#5-the-numbers-tolerances-timeouts-max-speed)); `lastMovementResult().timed_out` tells you which one |
| The last movement stops short of its target | The routine ends with a `_chain` movement: nothing takes over, so the robot stops after `CHAIN_STOP_AFTER_MS` | End with a normal movement |
| The left spot works, the mirrored right spot doesn't | A number without `flipX` / `flipHeading`, or a swing without `flipSide` | Check every x and heading ([Writing autonomous, section 8](writing-autonomous.md#8-one-routine-for-both-starting-spots)) |
| It doesn't fit in the autonomous period | Too many stops, or tight tolerances | Chains where there is room, looser tolerances where a little off doesn't matter ([Writing autonomous, section 9](writing-autonomous.md#9-faster-chains-and-the-clock)) |
| Works at home, not on the competition field | Different tiles (grip), a different battery, a different starting spot | Practice on field tiles; check `DRIVE_SLIP_LIMIT` there ([checklist, step 8](first-robot-checklist.md#8-traction-control)) |
| The robot keeps moving for a moment when driver control starts | A movement from autonomous is still running | `cancelMovement()` at the start of `usercontrol()` (`tankDrive` / `arcadeDrive` stop it too, as soon as the loop calls them) |
| The intake keeps spinning when driver control starts | Mechanisms keep doing their last command until they get a new one (an arm holding its position is what you want) | Give them one at the start of `usercontrol()` (`intake.stop()`), or in the driving loop |
| The arm stops partway when driver control starts | Autonomous ended before it got there, and it gave up while the robot was disabled (longer than `ARM_TIMEOUT_MS`, with no power): it holds where it got to | Give it a command in `usercontrol()`. The same `moveTo` again starts it over |
| A task of your own, started in autonomous, still runs in driver control (or one stops when the mode changes) | A change of mode only stops the task running `autonomous()` itself; with `Competition.bStopAllTasksBetweenModes = true`, being disabled stops every task except `main()` | Stop your task yourself at the start of `usercontrol()` (`myTask.stop()`). The library's own tasks start again by themselves either way ([How it works, section 5](how-it-works.md#when-the-competition-mode-changes)) |
| The Left / Right buttons change the routine during driver control | The selector is still running | `stopAutonSelector()` at the start of `usercontrol()` |

## 4. Movements

| What you see | Most likely | What to do |
| --- | --- | --- |
| The robot drives or turns the wrong way, one side fights the other, or it drives in a curve | A motor's direction or port | Run `testDrivetrain()` ([checklist, step 2](first-robot-checklist.md#2-wiring-and-motor-directions)): it says which `_DIRECTION` to change |
| Turns shake wildly, or spin faster and faster | `getGyroRate()` has the wrong sign: the brake pushes | `testDrivetrain()` says `getGyroRate() has the wrong sign`. Please report it too ([checklist, step 2](first-robot-checklist.md#2-wiring-and-motor-directions)) |
| Goes past the target and swings back, stops short, or is slow | The gains | [Tuning the PID, section 11](tuning.md#11-what-you-see-what-to-change) |
| Every movement waits at the end, then moves on | The tolerances are tighter than the robot can stop | Looser tolerances, or tune; a movement that always takes its full time is the same problem, worse |
| `PID_turn(0)` turns all the way around | The heading counts past 360: after a full turn it is at 360, and 0 is one turn back | `PID_turn_shortest(0, ...)` |
| Every distance is a little short or long | `WHEEL_DIAMETER_INCH` or `MOTOR_TO_WHEEL_GEAR_RATIO` | `measureWheelSize()` ([checklist, step 3](first-robot-checklist.md#3-measure-the-robot)) |
| Arcs end too wide or too tight, or at the wrong heading | `TRACK_WIDTH_INCH` (the `12.0` in the config is a placeholder) | `measureTrackWidth()` ([checklist, step 3](first-robot-checklist.md#3-measure-the-robot)) |
| `PID_forward` ends turned a degree or more | `FORWARD_HEADING_KP` too small | Raise it; weaving means too big ([checklist, step 6](first-robot-checklist.md#6-tune-driving-forward)) |
| The wheels chirp or slide when a movement starts or stops | `DRIVE_SLIP_LIMIT` too big for your wheels and tiles | Lower it ([checklist, step 8](first-robot-checklist.md#8-traction-control)) |
| A drive to a point weaves, or a drive to a pose ends inches off | The steering settings, or a pose that has no room to curve | [Odometry and paths, section 7](odometry-and-paths.md#7-when-something-goes-wrong) |
| A movement started with a button in driver control stops at once | It's an `_async` movement, and `arcadeDrive()` / `tankDrive()` in the same loop cancels it (the driver always wins) | Use the normal version, which waits until it's done, or skip the driving while it runs: `if (!isMoving()) { arcadeDrive(); }` |

## 5. Driver control

| What you see | Most likely | What to do |
| --- | --- | --- |
| The robot creeps with the sticks let go | The sticks don't sit exactly at 0 | Raise `DRIVE_DEADBAND` (5 is the default) |
| Hard to drive slowly and exactly | The sticks are too direct | Raise `DRIVE_CURVE` (2 is the default: half stick = a quarter power) |
| Turning is too twitchy (arcade) | Full turning power on the right stick | `DRIVE_TURN_SCALE = 0.7` |

## 6. Arm and intake

| What you see | Most likely | What to do |
| --- | --- | --- |
| The terminal says `Only 8 Arms can be used: this one does nothing` (or Intakes) | More than 8 of one kind | Combine motors that always move together into one `motor_group` |
| The arm goes to the wrong place | Positions are degrees of the motor, counted from where it was when the program started | `arm.resetPosition(0)` in `pre_auton` with the arm all the way down, then read your positions off `arm.position()` |
| The arm stops at a limit instead of where `moveTo` sends it | `setLimits`: a `moveTo` past a limit goes to the limit | Check the limits, set after `resetPosition` |
| The arm swings past its position, or creeps there | `ARM_KP` / `ARM_KD` | Tune them like a turn ([checklist, step 11](first-robot-checklist.md#11-arm-and-intake)) |
| A heavy arm stops a little short | P alone can't hold the weight up there | A little `ARM_KI`, and `ARM_INTEGRAL_RANGE` bigger than how far short it stops |
| The arm never finishes, `arm.waitUntilDone()` returns false | It's stuck, or can't get within `ARM_TOLERANCE` in `ARM_TIMEOUT_MS` | Look for what blocks it; then tune |
| The intake runs backwards now and then with nothing stuck | A normal game piece draws more than `INTAKE_JAM_CURRENT` | Measure the current (print `intakeMotor.current(amp)`) and set it between normal and stuck ([checklist, step 11](first-robot-checklist.md#11-arm-and-intake)) |
| A stuck intake isn't noticed | `INTAKE_JAM_CURRENT` too high, or the intake runs at low power (it can't draw that much) | Lower it, between normal and stuck |
| `isJammed()` is hardly ever true | With unjam on, it is only true while the intake backs off (`INTAKE_UNJAM_MS`) | To know a piece is all the way in, spin with unjam off: `intake.spin(100, false)`, then it stays jammed until `stop()` |

## 7. SD card

| What you see | Most likely | What to do |
| --- | --- | --- |
| `logToSDCard: no SD card in the Brain` (returns false) | No card, or not pushed in all the way | Put it in before the program starts |
| `logToSDCard: the SD card already has pidlog1.csv to pidlog9999.csv, delete some` | A very full card | Delete all the old logs (then the highest number is always the newest) |
| `logToSDCard: could not write to ..., was the SD card taken out? Logging stopped.` | The card was taken out, or is full or broken | Take the card out only after the robot is done |
| The log misses the last movements | They are written once the robot stands still between movements | Leave the program running a moment after the last movement before taking the card out |
| A short pause between movements with logging on | The log being written to the card | It happens when your code pauses anyway; if it hurts, log only while testing ([checklist, step 12](first-robot-checklist.md#12-sd-card)) |
| `saveGainsToSDCard: no SD card in the Brain` when pressing B in the tuner | No card | The gains are still shown on the Brain screen and in the terminal: copy them into the config |

## 8. Still stuck?

Open an issue on the [GitHub Issues tab](https://github.com/ianyin-vex/simple-v5-lib/issues) with
what you did, what you expected, what happened, your `simpleV5LibConfig.h`, and what the terminal
and the Brain screen said. If it is one of the things the simulator can't know about a real robot
([the checklist's last section](first-robot-checklist.md#things-the-simulator-cant-tell-please-report)),
that report is especially welcome.
