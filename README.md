# simple-v5-lib

A beginner-friendly VEX V5 C++ library with simple, easy-to-understand PID controllers for accurate autonomous driving and turning.

## What is simple-v5-lib?

This library provides basic autonomous movement for the VEX V5 system using a PID Controller for two main movements: **Forward** and **Turn**.

Standard "move-until-target" code can cause robots to overshoot and lose accuracy. A PID controller solves this by moving fast at the beginning, slowing down at the end, and coming to a stop just as the robot reaches the target. It achieves this by using three different parts:
* **Proportional (P):** The main driving force, proportional to your distance from the target.
* **Integral (I):** The final push, overcoming friction when the P and D values become too small to move the robot.
* **Derivative (D):** The braking system, slowing the robot down as it approaches the target to prevent overshooting.

## Why simple-v5-lib?

Advanced public libraries (like PROS) often use complex methods that are difficult for beginners to understand. Using these tools too early can force you in a **"using" mindset** rather than a **"learning" mindset**. 

This library uses simple language and easy logic. It helps you get the hang of it so you can eventually build your own custom library from scratch.

## What's Included

* **Autonomous movements:** `PID_forward`, `PID_turn`, `PID_turn_relative`, `PID_turn_shortest`, `PID_swing` and `PID_arc` (drive along a curve), with an optional timeout and max speed. `PID_forward` also keeps the robot driving straight.
* **Chained movements:** `PID_forward_chain` and friends move on to the next movement without stopping, for faster routes.
* **Do things while driving:** every movement has an `_async` version, so you can run an intake or lift while the robot drives.
* **Live data for tuning:** watch the error and power as a graph on the Brain screen, or as numbers in the terminal.
* **Driver control:** `tankDrive()` and `arcadeDrive()`, with a joystick deadband and curve.
* **Autonomous selector:** pick your routine on the Brain screen or the controller before the match.
* **A reusable `PIDController`:** the same one the drivetrain uses. Use it for your own lift or arm too.

## How to Use This Library

1. **Download:** Download the code.
2. **Start Project:** Start a project. If you already have one skip this step.
3. **Copy Files:** Copy every file in `src` to your src folder, and every file in `include` to your include folder. In main.cpp, add #include "simpleV5lib.h" at the very beginning. The library already creates `Brain` and `Controller`, so if your main.cpp has a `brain Brain;` line, delete it. In simpleV5LibConfig.h, change the ports and motor ratios to the ones of your robot. Here is an image regarding which motor name refers to which motor

This is an example of a drivetrain and what the motor names (left front, left middle, etc.) stand for:
![Drivetrain](images/drivetrainExampleImage.png)
4. **Calibrate:** Call `calibrateInertial();` once before any movement (for example in `pre_auton`). It takes about 2 seconds, and the robot must stay still.
5. **Tune:** Go to your simpleV5LibConfig.h file and tune your kp, ki, and kd constants by following this [YouTube Video Tutorial](https://www.youtube.com/watch?v=WN3_vxA_D04).
6. **Code:** Start writing your autonomous routes! `examples/competitionTemplate` is a full competition program that uses everything, `examples/tuning` helps you tune with live graphs, and the other examples show a single movement. Every movement stops on its own after a timeout (see `TURN_TIMEOUT_MS` and `FORWARD_TIMEOUT_MS`), so a stuck robot won't freeze your whole autonomous.

## Quick Reference

```cpp
// Autonomous (tolerances first, then the optional timeout and max speed)
PID_forward(24, 0.3, 0.2);                          // drive 24 inches forward (negative = backwards)
PID_forward(24, 0.3, 0.2, FORWARD_TIMEOUT_MS, 50);  // same, but at most 50% power
PID_turn(90, 0.5, 0.2);                             // turn to face exactly 90 degrees
PID_turn_relative(-45, 0.5, 0.2);                   // turn 45 degrees counter-clockwise from where you are
PID_turn_shortest(270, 0.5, 0.2);                   // face 270 degrees, whichever way is shorter
PID_swing(90, LEFT_SIDE, 0.5, 0.2);                 // swing to 90 degrees: only the left side moves
PID_arc(90, 24, 0.5, 0.2);                          // curve along a 24 inch circle until facing 90
PID_arc(0, -24, 0.5, 0.2);                          // same, but backing up (negative radius)
setHeading(0);                                      // "the robot is facing 0 degrees right now"

// Chained: don't stop between movements (much faster). End with a normal movement.
PID_forward_chain(24, 3);   // move on to the next movement 3 inches before the target
PID_turn_chain(90, 10);     // move on 10 degrees before the target
PID_forward(24, 0.3, 0.2);  // stops exactly at the end

// Do something while driving
PID_forward_async(48, 0.3, 0.2);   // start driving, and keep going in your code right away
waitUntilTraveled(24);             // wait until 24 inches are done (degrees for turns)
intake.spin(forward);              // your own motor
waitUntilDone();                   // wait until the drive is finished
cancelMovement();                  // stop the current movement now

// Live data while tuning
graphOnScreen(true);   // error and power as a graph on the Brain screen
logToTerminal(true);   // numbers in the terminal, to paste into a spreadsheet

// Driver control (inside the while loop in usercontrol)
arcadeDrive();   // or tankDrive();

// Autonomous selector
addAuton("Left side", leftSideAuton);   // in pre_auton, once per routine
startAutonSelector();                   // at the end of pre_auton
runSelectedAuton();                     // in autonomous()
stopAutonSelector();                    // at the start of usercontrol()
cancelMovement();                       // at the start of usercontrol(), in case autonomous was still moving
```

### Units
* Forward targets are in inches. It is worth noting that a single tile is around 23.622 inches wide.
* Turn targets are in degrees, clockwise is positive. You should know this, but 360 degrees make a full circle.
* Speeds and powers are percentages, from -100 to 100.
* Arcs need `TRACK_WIDTH_INCH` in simpleV5LibConfig.h: the distance between the middle of your left and right wheels.

## Testing

Every pull request is checked automatically (`.github/workflows/build.yml`):
* **Simulator tests:** `tests/run_tests.sh` builds the library against a small drivetrain simulator (`tests/sim/vex.h`) and checks that every movement ends where it should. You can run it on your own computer with any C++17 compiler.
* **VEX SDK build:** `ci/build_with_vex_sdk.sh` downloads the official VEXcode V5 SDK and compiles the library and all examples for the V5 brain.

## Feedback & Support

Notice a bug or have a feature request? Please open an issue on the [GitHub Issues tab](https://github.com/ianyin-vex/simple-v5-lib/issues).