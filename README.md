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

This library uses simple language and easy logic. It helps you get the hang of it so you can eventually build your own custom library from scratch. When you want to know how something works inside, [docs/how-it-works.md](docs/how-it-works.md) explains every part.

## What's Included

* **Autonomous movements:** `PID_forward`, `PID_turn`, `PID_turn_relative`, `PID_turn_shortest`, `PID_swing` and `PID_arc` (drive along a curve), with an optional timeout and max speed. `PID_forward` also keeps the robot driving straight.
* **Odometry:** the robot keeps track of where it is on the field (x, y in inches), and `PID_drive_to_point` / `PID_turn_to_point` drive to field points. Works with just the drive motors and inertial sensor; tracking wheels are optional. Correct the position against a wall, by touching it (`setX` / `setY`) or with a distance sensor (`resetXFromWall` / `resetYFromWall`).
* **Chained movements:** `PID_forward_chain` and friends move on to the next movement without stopping, for faster routes. `PID_drive_to_point_chain` turns a list of field points into one smooth path.
* **Do things while driving:** every movement has an `_async` version, so you can run an intake or lift while the robot drives.
* **Mechanisms:** an `Arm` (or lift) that moves to a position and holds it there, with a joystick in driver control too, and an `Intake` that notices when it jams (the motor draws a lot of current but can't turn) and runs backwards a moment to free it.
* **Live data for tuning:** watch the error and power as a graph on the Brain screen, or as numbers in the terminal.
* **Tuning from the controller:** `tuneWithController()` changes kP, kI and kD with the controller buttons and tries them right away, no re-downloading. The gains are saved on the SD card, and `loadGainsFromSDCard()` brings them back in the next run.
* **SD card logging:** `logToSDCard(true)` saves every movement's data in a CSV file, to look at after a match.
* **Robot setup checks:** `checkDevices()` finds unplugged or overheating devices, `testDrivetrain()` finds motors set to the wrong direction, and `measureTrackWidth()` / `measureWheelSize()` measure your robot for `simpleV5LibConfig.h`.
* **Driver control:** `tankDrive()` and `arcadeDrive()`, with a joystick deadband and curve.
* **Autonomous selector:** pick your routine on the Brain screen or the controller before the match.
* **A reusable `PIDController`:** the same one the drivetrain (and `Arm`) uses. Use it for your own mechanisms too.

## How to Use This Library

1. **Download:** Download the code.
2. **Start Project:** Start a project. If you already have one skip this step.
3. **Copy Files:** Copy every file in `src` to your src folder, and every file in `include` to your include folder. In main.cpp, add #include "simpleV5lib.h" at the very beginning. The library already creates `Brain` and `Controller`, so if your main.cpp has a `brain Brain;` line, delete it. In simpleV5LibConfig.h, change the ports and motor ratios to the ones of your robot. Here is an image regarding which motor name refers to which motor

This is an example of a drivetrain and what the motor names (left front, left middle, etc.) stand for:
![Drivetrain](images/drivetrainExampleImage.png)
4. **Check and measure:** Run `examples/robotSetup` once. It checks that everything is plugged in and that the motor directions are right, and measures your track width and wheel size. Put the numbers it shows into simpleV5LibConfig.h.
5. **Calibrate:** Call `calibrateInertial();` once before any movement (for example in `pre_auton`). It takes about 2 seconds, and the robot must stay still.
6. **Tune:** Run `examples/tuning`: it lets you change kp, ki and kd from the controller and try them right away, with a live graph on the Brain screen. Copy the gains it shows into simpleV5LibConfig.h when you're done. The [tuning guide](docs/how-it-works.md#9-tuning-step-by-step) and this [YouTube Video Tutorial](https://www.youtube.com/watch?v=WN3_vxA_D04) explain how.
7. **Code:** Start writing your autonomous routes! `examples/competitionTemplate` is a full competition program that uses everything, `examples/odometry` drives to field points, `examples/mechanisms` runs an arm and an intake, and the other examples show a single movement. Every movement stops on its own after a timeout (see `TURN_TIMEOUT_MS` and `FORWARD_TIMEOUT_MS`), so a stuck robot won't freeze your whole autonomous.

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
setHeading(0);                                      // "the robot is facing 0 degrees right now" (fine mid-chain too)

// Odometry: field points in inches, x to the right, y forward
setPose(0, 0, 0);                                   // "I'm at (0, 0) facing 0" (calibrateInertial does this too)
PID_drive_to_point(24, 24, 0.5, 0.2);               // turn towards (24, 24) and drive there
PID_drive_to_point(0, 0, 0.5, 0.2, FORWARD_TIMEOUT_MS, 100, true);  // back up to (0, 0)
PID_turn_to_point(0, 48, 0.5, 0.2);                 // face (0, 48)
getX(); getY();                                     // where the robot is now
setY(70 - 7);                                       // after bumping into the wall at y = 70: fix only y
resetXFromWall(RIGHT_SENSOR, 70);                   // stopped, a distance sensor sees the wall at x = 70: fix x

// Chained: don't stop between movements (much faster). End with a normal movement.
PID_forward_chain(24, 3);   // move on to the next movement 3 inches before the target
PID_turn_chain(90, 10);     // move on 10 degrees before the target
PID_drive_to_point_chain(24, 24, 4);  // field points too: move on 4 inches before (24, 24), curving to the next
PID_forward(24, 0.3, 0.2);  // stops exactly at the end

// Do something while driving
PID_forward_async(48, 0.3, 0.2);   // start driving, and keep going in your code right away
waitUntilTraveled(24);             // wait until 24 inches are done (degrees for turns, swings, arcs)
intakeMotor.spin(forward);         // your own motor
waitUntilDone();                   // wait until the drive is finished
cancelMovement();                  // stop the current movement now

// Mechanisms (see examples/mechanisms). Make them outside any function:
//   motor armMotor(PORT9, ratio36_1, false);     Arm arm(armMotor);
//   motor intakeMotor(PORT11, ratio6_1, false);  Intake intake(intakeMotor);
arm.resetPosition(0);              // "the arm is at 0 now" (resting all the way down)
arm.setLimits(0, 600);             // never lower than 0 or higher than 600 degrees
arm.moveTo(450);                   // go to 450 degrees (of the motor) and hold there; returns right away
arm.waitUntilDone();               // wait until it's there (false if it didn't make it in ARM_TIMEOUT_MS)
arm.manual(Controller.Axis2.position(percentUnits::pct));  // driver control: stick moves it, let go = hold
intake.spin(100);                  // runs until stop(); when it jams, backs off for a moment and goes on
intake.spin(100, false);           // when it jams, it stops instead: isJammed() = a piece is all the way in
intake.stop();
intake.isJammed(); intake.jamCount();

// How did the last movement go?
MovementResult r = lastMovementResult();   // r.time_ms, r.error, r.overshoot, r.timed_out

// Tuning
graphOnScreen(true);   // error and power as a graph on the Brain screen
logToTerminal(true);   // numbers in the terminal, to paste into a spreadsheet
tuneWithController();  // change kP/kI/kD with the controller buttons and try them (B ends it and saves them on the SD card)
loadGainsFromSDCard(); // at the start of your program: use the gains saved on the SD card
logToSDCard(true);     // every movement's data into pidlog1.csv, pidlog2.csv, ... on the SD card
turnGains.kp = 3.5;    // or change the gains in code (also forwardGains, swingGains, arcGains)

// Robot setup (see examples/robotSetup)
checkDevices();        // everything plugged in? motors not too hot? (doesn't move)
testDrivetrain();      // motor directions, left/right, gyro sign (the robot turns a little)
measureWheelSize(48);  // push the robot 48 inches by hand, shows WHEEL_DIAMETER_INCH
measureTrackWidth();   // spins 3 times, shows TRACK_WIDTH_INCH and tracking wheel offsets (after measureWheelSize)

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
* Arm positions are degrees of the motor (what `arm.position()` says), not of the arm itself.
* Arcs need `TRACK_WIDTH_INCH` in simpleV5LibConfig.h: the distance between the middle of your left and right wheels.
* Field positions (odometry) are in inches: x to the right, y forward, measured from where you called `setPose`. If you have tracking wheels, set their ports and positions in simpleV5LibConfig.h, and the same for distance sensors.

## Testing

Every pull request is checked automatically (`.github/workflows/build.yml`):
* **Simulator tests:** `tests/run_tests.sh` builds the library against a small drivetrain simulator (`tests/sim/vex.h`) and checks that every movement ends where it should. You can run it on your own computer with any C++17 compiler.
* **VEX SDK build:** `ci/build_with_vex_sdk.sh` downloads the official VEXcode V5 SDK, compiles the library for the V5 brain, and links every example into a full program, the same `.bin` file VEXcode downloads to the brain.

## Feedback & Support

Notice a bug or have a feature request? Please open an issue on the [GitHub Issues tab](https://github.com/ianyin-vex/simple-v5-lib/issues).