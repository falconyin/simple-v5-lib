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

* **Autonomous movements:** `PID_forward`, `PID_turn`, `PID_turn_relative`, `PID_turn_shortest` and `PID_swing`, with an optional timeout and max speed. `PID_forward` also keeps the robot driving straight.
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
6. **Code:** Start writing your autonomous routes! `examples/competitionTemplate` is a full competition program that uses everything; the other examples show a single movement. Every movement stops on its own after a timeout (see `TURN_TIMEOUT_MS` and `FORWARD_TIMEOUT_MS`), so a stuck robot won't freeze your whole autonomous.

## Quick Reference

```cpp
// Autonomous (tolerances first, then the optional timeout and max speed)
PID_forward(24, 0.3, 0.2);                          // drive 24 inches forward (negative = backwards)
PID_forward(24, 0.3, 0.2, FORWARD_TIMEOUT_MS, 50);  // same, but at most 50% power
PID_turn(90, 0.5, 0.2);                             // turn to face exactly 90 degrees
PID_turn_relative(-45, 0.5, 0.2);                   // turn 45 degrees counter-clockwise from where you are
PID_turn_shortest(270, 0.5, 0.2);                   // face 270 degrees, whichever way is shorter
PID_swing(90, LEFT_SIDE, 0.5, 0.2);                 // swing to 90 degrees: only the left side moves
setHeading(0);                                      // "the robot is facing 0 degrees right now"

// Driver control (inside the while loop in usercontrol)
arcadeDrive();   // or tankDrive();

// Autonomous selector
addAuton("Left side", leftSideAuton);   // in pre_auton, once per routine
startAutonSelector();                   // at the end of pre_auton
runSelectedAuton();                     // in autonomous()
stopAutonSelector();                    // at the start of usercontrol()
```

### Units
* Forward targets are in inches. It is worth noting that a single tile is around 23.622 inches wide.
* Turn targets are in degrees, clockwise is positive. You should know this, but 360 degrees make a full circle.
* Speeds and powers are percentages, from -100 to 100.

## Feedback & Support

Notice a bug or have a feature request? Please open an issue on the [GitHub Issues tab](https://github.com/ianyin-vex/simple-v5-lib/issues).