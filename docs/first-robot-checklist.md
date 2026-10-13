# First day on a real robot: a checklist

The numbers in `simpleV5LibConfig.h` were picked and tuned on the simulator in `tests/sim`, a
pretend robot that runs on a computer. A real robot has its own weight, wheels, grip and motors,
so go through this list once on every new robot (and again after rebuilding the drivetrain). Do
the steps in order: each one needs the ones before it to be right.

Every step says what to run, what you should see, and what to change. The details of *why* are
in [how-it-works.md](how-it-works.md); the steps link to the right section.

- [Before you start](#before-you-start)
- [1. The config file](#1-the-config-file)
- [2. Wiring and motor directions](#2-wiring-and-motor-directions)
- [3. Measure the robot](#3-measure-the-robot)
- [4. Odometry, pushing by hand](#4-odometry-pushing-by-hand)
- [5. Tune turns](#5-tune-turns)
- [6. Tune driving forward](#6-tune-driving-forward)
- [7. Tune swings and arcs](#7-tune-swings-and-arcs)
- [8. Traction control](#8-traction-control)
- [9. Driving to points and poses](#9-driving-to-points-and-poses)
- [10. Distance sensors](#10-distance-sensors)
- [11. Arm and intake](#11-arm-and-intake)
- [12. SD card](#12-sd-card)
- [13. A match, with a competition switch](#13-a-match-with-a-competition-switch)
- [14. Optional: a speed profile](#14-optional-a-speed-profile)
- [Things the simulator can't tell: please report](#things-the-simulator-cant-tell-please-report)

---

## Before you start

- A charged battery. A low battery makes everything slower, and you'd tune for that.
- Field tiles and room: at least 2 tiles in front of the robot, and room to spin.
- A tape measure, and a USB cable to see the terminal (most steps also show on the Brain screen).
- An SD card in the Brain, so the tuner can save your gains.
- Write down every number you change, and what you saw. If something looks wrong, that note is
  what lets someone find out why.

## 1. The config file

Open `include/simpleV5LibConfig.h` and set what you can read off the robot:

- [ ] The six drive motor ports (`PORT_LEFTFRONT`, ...) and the inertial sensor port.
- [ ] Each motor's cartridge (`LF_GEAR_RATIO`, ...): `ratio36_1` red, `ratio18_1` green,
  `ratio6_1` blue.
- [ ] `WHEEL_DIAMETER_INCH`, what is printed on the wheel. Step 3 measures it more exactly.
- [ ] `MOTOR_TO_WHEEL_GEAR_RATIO`: wheel turns per motor turn. A 36-tooth gear on the motor
  driving a 48-tooth gear on the wheel is `36.0 / 48.0`. No gears between them: `1.0`.
- [ ] Tracking wheel and distance sensor ports, if you have them (`-1` if not).

Don't guess the motor directions (`LF_DIRECTION`, ...): step 2 tells you which ones are wrong.

## 2. Wiring and motor directions

Run `examples/robotSetup`. Press A on the controller to go from one step to the next.

- [ ] **`checkDevices()`** (the robot doesn't move) must end with "Everything is plugged in."
  Fix every problem it lists first: an unplugged motor makes every later step wrong.
- [ ] **`calibrateInertial()`**: keep the robot completely still for about 2 seconds.
- [ ] **`testDrivetrain()`** turns the robot a little each way. It must end with "Drivetrain OK:
  motors, sides and inertial sensor agree." If not, change what it says (a `_DIRECTION`, or a
  port), download again, and run it again until it does.

If it says **"getGyroRate() has the wrong sign"**, please also report it (see the
[end of this page](#things-the-simulator-cant-tell-please-report)): the library *assumes* which
way the inertial sensor counts its turning speed, and this is the first real robot that can say
whether that is right. The fix is to remove the minus sign in `getGyroRate()` in
`src/simpleV5lib.cpp`. With the wrong sign, the D part of every turn pushes instead of braking,
and turns shake wildly. ([How it works, section 10](how-it-works.md#10-setting-up-a-new-robot))

## 3. Measure the robot

Still in `examples/robotSetup`:

- [ ] **`measureWheelSize(48)`**: push the robot by hand, straight forward, exactly 48 inches
  along a tape measure. Copy `WHEEL_DIAMETER_INCH` into the config, and with a forward tracking
  wheel also `TRACKING_WHEEL_DIAMETER_INCH` (both tracking wheels use it). A wheel size far from
  what is printed on the wheel means `MOTOR_TO_WHEEL_GEAR_RATIO` is wrong. With only a sideways
  tracking wheel, nothing measures it: use what is printed on it, and step 4 checks it.
- [ ] **`measureTrackWidth()`** spins the robot 3 times. Copy `TRACK_WIDTH_INCH` (and the tracking
  wheel offsets). The `12.0` in the config is only a placeholder: arcs are wrong until you
  replace it. The result is often a little more than the tape measure says; use the measured one.
- [ ] Download again and run `examples/robotSetup` a second time. The numbers should now hardly
  change (within about 1%). If they jump around from run to run, push straighter, and check that
  the wheels and the inertial sensor are mounted firmly.

## 4. Odometry, pushing by hand

Before anything drives by itself, check that the robot knows where it is. A tiny program:

```cpp
int main() {
    calibrateInertial();
    setPose(0, 0, 0);
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.printAt(10, 40, true, "x %.1f  y %.1f  heading %.1f", getX(), getY(), getInertial());
        wait(100, msec);
    }
}
```

- [ ] Push the robot 48 inches straight forward: y goes to about 48, x stays near 0.
- [ ] Turn it by hand to face right: heading about 90 (clockwise is positive).
- [ ] Push it 24 inches forward (to the right on the field): x goes up by about 24, y stays.
- [ ] With a sideways tracking wheel: push the robot 24 inches sideways (to its left) without
  turning it: y goes up by about 24, x stays. Wrong direction: change
  `TRACKING_SIDEWAYS_REVERSED`. Too far or not far enough: `TRACKING_WHEEL_DIAMETER_INCH` is off.
- [ ] Leave it running a few minutes, then push it again: the numbers still change. (They
  come from a background task; this checks it keeps running.)

Off by a few percent in one direction only? Redo step 3. ([How it works, section 7](how-it-works.md#7-odometry-where-is-the-robot))

## 5. Tune turns

Run `examples/tuning` (the controller buttons are explained at the top of its `main.cpp`). Steps
5 to 7 are one tuning session: it starts on TURN, and X moves on to FORWARD, SWING, then ARC.
A tries a 90° turn, the next A turns back. Follow the
[tuning guide](tuning.md) (with pictures of the graph at each step): kI and kD to 0, raise kP until it swings
around the target, raise kD until the overshoot (`ov`) is near 0, kI only if it stops short.

**Don't press B until step 7 is done**: B saves all four sets of gains on the SD card and ends
the tuner. If you have to stop in between, press B anyway: the next run of `examples/tuning`
loads what was saved, and you carry on from there (press X until you're back at the step).

- [ ] `TURN_KP`, `TURN_KI`, `TURN_KD` tuned on the robot.
- [ ] Later, with the gains in the config, also try a 180° and a 45° turn in your own code
  (`PID_turn_relative(180, 0.5, 0.2)`): good at 90° doesn't always mean good at 180°.
- [ ] Turns stop just short and stay there? Raise `TURN_MIN_SPEED` a little before reaching for kI.

## 6. Tune driving forward

Press X once: FORWARD. A drives 24 inches, the next A drives back.

- [ ] **`FORWARD_KD`: start from 0 and tune it again.** The `0.78` in the config was tuned while
  the D part of the PID had the wrong sign (it pushed instead of braking), so it means nothing now.
- [ ] `FORWARD_KP`, `FORWARD_KI` tuned. Later, in your own code, try a short (6 inch) and a long
  (48 inch) drive too.
- [ ] **`FORWARD_HEADING_KP`** (keeps the robot straight, `1.0` is only a starting value): drive
  48 inches and look at the heading before and after. It drifted off by more than a degree or
  two: raise it. The robot weaves left and right while driving: lower it.

## 7. Tune swings and arcs

The swing and arc gains in the config are copies of the turn and forward gains: nobody has
tuned them yet.

- [ ] Press X: SWING (a 90° swing on the left side). Swings often need a bigger kP than turns,
  because only one side pushes. Tune `SWING_KP`, `SWING_KI`, `SWING_KD`.
- [ ] Press X: ARC (a 90° arc along a 24 inch circle). Arcs need `TRACK_WIDTH_INCH` from step 3
  first. Tune `ARC_KP`, `ARC_KI`, `ARC_KD`.
- [ ] Now press B. Copy the gains it shows (all four sets) into the config, so they are right
  even without the SD card.

## 8. Traction control

`DRIVE_SLIP_LIMIT = 40` keeps each side's power within 40% of how fast its wheels already turn,
so a movement can't spin the wheels at the start or skid them when braking. 40 is what the
*simulator's* wheels could take; your wheels and tiles grip differently.
([How it works, section 1](how-it-works.md#not-letting-the-wheels-slip-traction-control))

- [ ] Drive a fast route a few times, for example 48 inches forward, a 90° turn and 24 inches
  forward, all at full speed, and listen and look. Wheels chirping, or the robot lurching or
  sliding at a start or stop: lower `DRIVE_SLIP_LIMIT` (try 30).
- [ ] No slipping at all, and starts and stops feel slow and soft: try a higher value (60). Keep
  the highest value that doesn't slip.
- [ ] After the route, measure where the robot really is with a tape measure and compare it with
  `getX()`/`getY()`. Without tracking wheels, odometry counts the drive wheels' turns, so
  slipping wheels count inches the robot didn't drive. (Tracking wheels don't slip, so with them
  this check says little about traction control.)
- [ ] With the robot on a stand, wheels in the air, run `move(50, 50)` and print
  `leftDrive.velocity(velocityUnits::pct)`. It should say about 50. Traction control compares
  power with that number: if it is far off (say 30), it holds the power back too much, and starts
  are slow. Please report what you see either way.

## 9. Driving to points and poses

Run `examples/odometry`. It drives a square, backs up, then drives the square again as one
smooth path. `POINT_HEADING_KP`, `POINT_HEADING_KD` and `POSE_LEAD` were tuned on the simulator only.

- [ ] At the end the robot should be back at (0, 0). Measure how far off it is with a tape measure.
  (With tracking wheels it should be within an inch or so; with only the drive motors, a few
  inches after this many corners is normal.)
- [ ] The robot weaves around the line to the point: lower `POINT_HEADING_KP`, or raise
  `POINT_HEADING_KD`.
- [ ] It turns too slowly towards the point and drives a wide curve: raise `POINT_HEADING_KP`.
`examples/odometry` only drives to points. For poses, a short program of your own, with 3 tiles
of room ahead and 2 to the right:

```cpp
int main() {
    calibrateInertial();
    setPose(0, 0, 0);
    PID_drive_to_pose(24, 48, 90, 0.5, 0.2); // one curve to (24, 48), arriving facing right
}
```

- [ ] It ends near (24, 48), facing about 90. Measure it with a tape measure.
- [ ] On the curve it swings past the direction it should be going, then corrects hard (the
  wheels may skid): raise `POINT_HEADING_KD`.
- [ ] It arrives facing the wrong way and turns hard at the end: raise `POSE_LEAD` (a wider
  curve). Its curve is too wide for the field: lower it.

([How it works, section 8](how-it-works.md#8-driving-to-a-point))

## 10. Distance sensors

Only if you set `DISTANCE_*_PORT`s. The positions (`DISTANCE_*_AHEAD`, `DISTANCE_*_RIGHT`) are
measured from the robot's center, to the front of the sensor.

- [ ] Put the robot still, square to a wall, at a distance you measure with a tape measure.
  `setPose` it to the right place, then call `resetXFromWall` or `resetYFromWall`: it should
  return true and change the position by less than an inch. The terminal says why when it
  refuses.
- [ ] Do the same with the robot at about 15° to the wall. The result should still be within an
  inch or so.
- [ ] Do it right after a movement, without the `wait(100, msec)` before it. If the result is
  worse than when standing still, the sensor reads late: keep the wait in your autonomous.

([How it works, section 7](how-it-works.md#correcting-the-position-from-a-wall))

## 11. Arm and intake

`ARM_KP = 1.5`, `ARM_KD = 10` and `INTAKE_JAM_CURRENT = 2.0` were tuned for a simulated arm and
intake. Run `examples/mechanisms` with your motor ports.

- [ ] `arm.moveTo(...)` gets there without swinging past it. Tune `ARM_KP` and `ARM_KD` like a turn.
- [ ] Once there, the arm holds its position. Push it down gently by hand and let go: it should
  push back to where it was. Then move it back and forth between two positions several times
  (and with `manual`, letting go of the stick each time), and check it still holds after each.
- [ ] Find what your intake draws: print `intakeMotor.current(amp)` on the Brain screen while it
  runs empty, while it pulls a game piece in, and while a game piece is stuck (stuck against
  something, never your fingers). `INTAKE_JAM_CURRENT` goes between the normal and the stuck
  number. A V5 motor draws at most 2.5 A.
- [ ] Stuck: `isJammed()` turns true and `jamCount()` goes up. A normal game piece: it doesn't.

([Arms and intakes](mechanisms.md) shows what each tuning mistake looks like, and how to measure
the intake's current; [How it works, section 11](how-it-works.md#11-mechanisms-an-arm-that-holds-an-intake-that-unjams)
explains the code.)

## 12. SD card

Writing to an SD card is slow, so the log waits in memory and is written once the robot stands
still with no movement running. Movements that follow each other right away don't leave that
moment, so the writing usually happens when your code pauses, for example waiting for an intake.
If the next movement starts during the writing, it starts late. Nobody has measured how long the
writing takes on a real Brain. This program measures it:

```cpp
int main() {
    calibrateInertial();
    logToSDCard(true);         // run it a second time with false, to compare
    PID_forward(24, 0.3, 0.2); // a movement to log
    wait(20, msec);            // a pause: the writing starts now
    double start = Brain.timer(timeUnits::msec);
    PID_forward(-24, 0.3, 0.2); // can't start until the writing is done
    printf("took %.0f ms\n", Brain.timer(timeUnits::msec) - start);
}
```

- [ ] Run it with logging on and with logging off. The difference is how much longer than the
  20 ms pause the writing took. No difference: it fits in the pause, nobody will notice it. More
  than about a tenth of a second: please report the numbers.

## 13. A match, with a competition switch

When the competition mode changes, VEXcode stops the task that ran `autonomous()`, usually in the
middle of a movement. The library's own tasks must carry on, or start again (how this works, and
how the simulator copies it, is in [How it works, section 5](how-it-works.md#when-the-competition-mode-changes)).
Plug a competition switch into the controller and run `examples/mechanisms` with your ports.

- [ ] Start autonomous, and switch to disabled while the robot is driving: it stops.
- [ ] Switch to driver control: the robot doesn't move until you touch the sticks, then drives
  normally, and the arm and intake answer their buttons.
- [ ] Again, but stay disabled for 5 seconds, with the arm still on its way when autonomous ends: in
  driver control its button moves it again. Odometry (on the Brain screen, as in step 4) still matches
  where the robot is, including how far it rolled after autonomous ended.
- [ ] Again, with `Competition.bStopAllTasksBetweenModes = true;` as the first line of `main()`:
  the same.
- [ ] With that line still in, run the program before plugging in the competition switch, and plug
  it in (disabled) while the selector shows: taps and Left / Right should now do nothing (the
  selector stops, see [How it works, section 5](how-it-works.md#when-the-competition-mode-changes)).
  If they still work, please report it.

## 14. Optional: a speed profile

Only if you want drives to plan their speed (`DRIVE_ACCELERATION`, off unless you set it;
[Tuning the PID, section 12](tuning.md#12-optional-a-speed-profile) says what it does). Do it
after steps 3 and 6: it uses the measured wheel size, and the PID still finishes every drive.

- [ ] Run step 5 of `examples/robotSetup` (`measureTopSpeed()`): THE ROBOT DRIVES FORWARD at full
  power for about a second, so give it 6 feet. Put the `DRIVE_TOP_SPEED` and `DRIVE_RESPONSE_MS`
  it shows into the config. Run it twice: the numbers should hardly change. (On the simulator it
  says 69.8 in/s and 99 ms; a drained battery gives less top speed.)
- [ ] Set `DRIVE_ACCELERATION = 150`, and run `PID_forward(24)` and `PID_forward(48)` a few times
  with `graphOnScreen(true)`. Goes past the target: more `DRIVE_RESPONSE_MS`. Stops short and
  creeps in: less. Wheels chirp or slide at the start or end: less `DRIVE_ACCELERATION`.
- [ ] Compare the times and overshoot (the controller line, as in step 6) with
  `DRIVE_ACCELERATION = 0`. Keep the profile only if it is better on your robot, and please report
  what you saw either way.

---

## Things the simulator can't tell: please report

Some things depend on how the real V5 hardware behaves, which the simulator can only guess. The
steps above check most of them; here they are in one list. Whatever you find, right or wrong,
please open an issue on the [GitHub Issues tab](https://github.com/ianyin-vex/simple-v5-lib/issues)
with what you saw.

| What | Checked in | What would be wrong |
| --- | --- | --- |
| Which way the inertial sensor counts turning speed | step 2 | "getGyroRate() has the wrong sign" |
| Background tasks keep running the whole program | step 4 | odometry stops changing after a while |
| `velocity(pct)` of a free-spinning motor matches its power | step 8 | about 30 at 50% power |
| How hard the wheels can push before slipping | step 8 | best `DRIVE_SLIP_LIMIT` far from 40 |
| How late a distance sensor reads | step 10 | worse results right after moving |
| A motor's hold mode after several stops in a row | step 11 | the arm sags after a few moves |
| How long an SD card write takes | step 12 | a movement after a pause starts late |
| Which tasks a change of competition mode stops | step 13 | the robot drives on by itself in driver control, or the arm or intake doesn't answer |
| A motor picks up its last command again after being disabled | step 13 | the intake autonomous left spinning stands still in driver control |
| How a drivetrain follows a new power (the simulator's is one smooth lag) | step 14 | `measureTopSpeed()` gives a different `DRIVE_RESPONSE_MS` every run, or the profile still goes past the target with the measured one |
| Whether the speed profile helps at all on real wheels | step 14 | slower or less exact than `DRIVE_ACCELERATION = 0` |
| `setPose` while the robot moves | below | the position jumps back to the old value |

The last one: the simulator switches between tasks only when one of them waits, a real Brain
might switch at any moment. Add this to the program from step 4, push the robot around by hand,
and press A a few times while it moves: every time, the screen should show x and y close to 0
right away and count on from there.

```cpp
if (Controller.ButtonA.pressing()) {
    setPose(0, 0, 0);
}
```
