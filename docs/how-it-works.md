# How simple-v5-lib works

This page explains the ideas behind every part of the library, in the order you meet them.
You don't need any of it to *use* the library. But when you want to know why the robot
does something, or want to write your own library later, start here. Every section points
to the code that does it.

- [1. PID: the heart of every movement](#1-pid-the-heart-of-every-movement)
- [2. When is a movement finished?](#2-when-is-a-movement-finished)
- [3. Driving straight and turning](#3-driving-straight-and-turning)
- [4. Swings and arcs](#4-swings-and-arcs)
- [5. Doing things while driving: the motion task](#5-doing-things-while-driving-the-motion-task)
- [6. Chained movements](#6-chained-movements)
- [7. Odometry: where is the robot?](#7-odometry-where-is-the-robot)
- [8. Driving to a point](#8-driving-to-a-point)
- [9. Tuning, step by step](#9-tuning-step-by-step)
- [10. Setting up a new robot](#10-setting-up-a-new-robot)
- [11. Mechanisms: an arm that holds, an intake that unjams](#11-mechanisms-an-arm-that-holds-an-intake-that-unjams)

---

## 1. PID: the heart of every movement

Every movement works the same way. 100 times a second (every 10 ms) it:

1. measures how far the robot still has to go: the **error** (`target - where we are`),
2. works out a motor power from that error,
3. sends the power to the motors.

The "works out a power" part is the PID controller (`PIDController::compute` in
`src/simpleV5lib.cpp`). The power is the sum of three parts:

```
power = kP * error  +  kI * (sum of recent errors)  -  kD * speed
          P                  I                           D
```

### P: push harder the further away you are

`kP * error` is the main push. Far from the target the error is big, so the robot drives
fast; close to it the error is small, so it slows down. With only P, the robot usually
arrives too fast and swings past the target: when it *gets* to the target the error is 0,
but the robot is still moving.

### D: brake

`-kD * speed` pushes **against** the way the robot is moving. The faster the robot moves,
the harder it brakes. This is what stops the swinging back and forth.

Many PID examples use "change in error" instead of speed. While the target stays the same,
the change in error is exactly the negative speed, so it's the same thing. Using the measured
speed (from the motors or the gyro) has one advantage: it is smoother, and it doesn't jump
when the target changes.

### I: the final push

When the robot is almost there, P is tiny (the error is tiny) and friction can stop the robot
just short of the target. The I part adds up the error over time, so a small error that
doesn't go away slowly builds up a push until the robot gets there.

The I part is dangerous: if it adds up during the whole drive, it builds a huge push that
makes the robot fly past the target. Two rules keep it under control:

- **Integral range** (`TURN_INTEGRAL_RANGE`, `FORWARD_INTEGRAL_RANGE`): only add up the error
  when the robot is closer to the target than this. Further away, the sum is reset to 0.
- **Reset when crossing the target**: when the error changes sign, the robot just went past
  the target, and the old push would only make it go further. The sum is reset to 0.

### Speeding up gently

At the start of a movement the error is at its biggest, so P asks for full power right away.
Full power from standing still makes the wheels slip, and slipping wheels make the distance
measurement wrong. So for the first 0.3 s, the power is limited to a ramp that starts at 30%
and rises to 100% (`startRamp`). The ramp only limits how *hard* the robot pushes; the
direction always comes from the PID.

### The gains

kP, kI and kD are called the **gains**. They start with the values in `simpleV5LibConfig.h`
and live in `turnGains`, `forwardGains`, `swingGains` and `arcGains` while the program runs,
so `tuneWithController()` (or your own code) can change them. Each movement copies the gains
when it starts, so a change only counts for the movements after it.

---

## 2. When is a movement finished?

A movement is finished when the robot is **close enough and has stopped**:

- `error_tolerance`: closer to the target than this, and
- `speed_tolerance`: moving slower than this,
- both for at least `TURN_SETTLE_MS` / `FORWARD_SETTLE_MS` (50 ms by default).

Why the speed, too? Without it, a robot flying past the target would count as "finished"
for the moment it is close, and would then roll on. Why the settle time? A single reading can
be lucky; staying inside the tolerances for a little while proves the robot really stopped.

Two more ways a movement ends:

- **Timeout**: if it takes longer than `timeout_ms` (a robot stuck against a wall never
  reaches its target), it gives up so the rest of the autonomous can still run.
  `lastMovementResult().timed_out` tells you this happened.
- **Minimum turning power** (`TURN_MIN_SPEED`): a turn that has almost stopped, but isn't at the
  target yet, gets at least this much power, so friction can't hold it just short.

`lastMovementResult()` tells you how the last movement went: how long it took, how far it
went past the target (the overshoot), and how far from the target it ended (the error).

---

## 3. Driving straight and turning

### Turning in place (`PID_turn`)

The error is `target heading - current heading` in degrees. The inertial sensor gives the
heading (`getInertial`, clockwise is positive) and the turning speed for the D part
(`getGyroRate`). The power goes to the two sides in opposite directions:
`move(power, -power)`.

Headings keep counting past 360: `PID_turn(450)` after `PID_turn(0)` turns 450 degrees, all
the way. `PID_turn_shortest` and `PID_turn_relative` work out the target heading for you.

### Driving straight (`PID_forward`)

The error is `target - distance driven`, in inches. The distance comes from the drive motors:

```
inches = motor turns * MOTOR_TO_WHEEL_GEAR_RATIO * wheel circumference
```

averaged over the left and right side (`getPosition`). Motor speed gives the D part
(`getMotorRate`).

A drivetrain never drives perfectly straight: one side is always a bit stronger. So
`PID_forward` remembers the heading it started with, and while driving it adds a small
correction to one side and takes it from the other:

```
correction = (start heading - current heading) * FORWARD_HEADING_KP
left  = power + correction
right = power - correction
```

If both sides together would need more than `max_speed`, both are scaled down by the same
amount (`moveLimited`), so the robot still steers the same way.

---

## 4. Swings and arcs

### Swing turns (`PID_swing`)

Only one side drives; the other side holds still (`brakeType::hold`). The robot turns around
the wheels that hold still. It is the same loop as a turn in place, with the power going to
one side only.

### Arcs (`PID_arc`)

An arc drives along a circle. Picture the middle of the robot following a circle of radius
`r`. The wheels on the outside of the circle are `TRACK_WIDTH_INCH / 2` further from its
center, the inside ones are that much closer:

```
outside wheels: (r + track/2) / r  times as fast as the middle of the robot
inside wheels:  (r - track/2) / r  times as fast
```

To end up facing the target, the middle of the robot must drive along the circle for

```
arc length = angle to turn (in radians) * r
```

So the arc's PID works in **inches along the circle**, just like `PID_forward` (that's why the
arc gains start from the forward gains). Each loop, the remaining angle becomes remaining
inches, the PID gives the power for the middle of the robot, and the two sides get that power
times their factor from above.

This is why `TRACK_WIDTH_INCH` matters: with the wrong track width, the two sides go at the
wrong ratio, and the robot curves on a different circle than you asked for.

---

## 5. Doing things while driving: the motion task

Every movement runs in **one background task** (`motionLoop`). `PID_forward(...)` just hands the
movement to that task and waits until it says "done". `PID_forward_async(...)` hands it over
and returns right away, so your code can run an intake while the robot drives.

The V5 brain runs tasks by taking turns: a task runs until it waits (`wait`, `vexDelay`), then
the next task gets a turn. That's why every loop in the library has a `vexDelay`: without it,
no other task would ever run.

Your code and the background task share a few values (is a movement running, how far has it
gone, ...). They are `std::atomic`, which guarantees that when one task changes a value, the
other one sees the new value. Starting a movement "claims" the drivetrain in one step
(`compare_exchange_weak` in `startMotion`), so two tasks can never start a movement at the same
time; the second one waits until the first movement is done.

---

## 6. Chained movements

A normal movement brakes to a full stop at its target. That's exact, but it wastes time:
slow down, stop, speed up again for the next movement.

A chained movement (`PID_forward_chain`, `PID_turn_chain`, ...) doesn't stop:

1. Its PID aims a bit **past** the real target (by `exit_range`), so it doesn't slow down
   much near the real target.
2. As soon as the robot is within `exit_range` of the real target, it hands over to the next
   movement **with the motors still running**.
3. If the robot is still *driving* the way the next movement goes (faster than
   `ALREADY_DRIVING_SPEED`), that movement skips the gentle start (`startRamp`). After a chained
   turn in place the robot is turning but not driving, and a backward drive after a forward one
   has to reverse first: both still start gently, or the wheels would slip.

Some details that keep chains exact:

- **The heading to keep**: after a chained turn, the robot is still turning when the next
  `PID_forward` starts. "Keep the heading you start with" would lock in a heading a few degrees
  short of the turn's target, so the forward keeps the heading the *turn was aiming for*.
- **No lost distance**: a chained forward hands over a few inches early. A forward chained
  after it measures from where the previous one's *target* was, so 24 + 24 + 24 ends at 72.
- **Safety stop**: if nothing follows a chained movement within `CHAIN_STOP_AFTER_MS`, the
  background task stops the drivetrain, so the robot can't drive off on its own.

---

## 7. Odometry: where is the robot?

Odometry (`src/odometry.cpp`) keeps track of the robot's position on the field (x, y in inches,
heading from the inertial sensor). Every 10 ms it:

1. reads how far the robot moved forward (`Δforward`) and sideways (`Δsideways`) since last
   time, and how much it turned (`Δθ`),
2. turns that small movement "relative to the robot" into a movement "on the field",
3. adds it to x and y.

### Forward and sideways movement

Without tracking wheels, the forward movement comes from the drive motors and the sideways
movement is 0 (a tank drivetrain can't drive sideways... unless another robot pushes it).
Tracking wheels are small unpowered wheels with a rotation sensor; they don't slip when the
drivetrain pushes hard, and a sideways one also notices pushes.

A tracking wheel that isn't in the middle of the robot also rolls when the robot turns in
place: on a circle around the robot's center. That part isn't a movement of the robot, so
it is taken out:

```
Δforward  += TRACKING_FORWARD_OFFSET  * Δθ   (Δθ in radians, clockwise is positive)
Δsideways -= TRACKING_SIDEWAYS_OFFSET * Δθ
```

### Curves

While turning, the robot moved along a little curve, but x and y need the straight line
from the start to the end of it (the "chord"). For a circle piece, the chord is a little
shorter than the curve:

```
chord = curve * 2 * sin(Δθ / 2) / Δθ
```

### Onto the field

On the way, the robot faced, on average, halfway between its old and new heading:
`θ = old heading + Δθ / 2`. With heading 0 pointing along +y and clockwise positive:

```
x += Δforward * sin(θ) + Δsideways * cos(θ)
y += Δforward * cos(θ) - Δsideways * sin(θ)
```

### Resetting the position

`setPose` and `setHeading` change the position or heading by hand. The heading would jump,
and the next odometry step would see that jump as the robot turning. So these functions count
up a counter (`manual_changes`), and when the odometry loop sees it change, its next reading
starts fresh instead of counting as a movement.

`setHeading` between chained movements also moves the heading the chained movement was aiming
for (`shiftChainHeading`) by the same jump. Without that, the next movement would keep the old
number and turn the robot back to it.

### Correcting the position from a wall

Odometry adds up thousands of small movements, so small mistakes (a wheel slipping, a push
the wheels didn't notice) add up too. The field walls don't move, so they can fix that.

The simplest way needs no sensor: drive gently into a wall, and tell the robot where it is
now with `setX` or `setY`. Only that one number changes; the other one and the heading stay.
Odometry keeps reading the wheels and the inertial sensor as before, so (unlike `setPose`)
no fresh start is needed.

`resetXFromWall` and `resetYFromWall` do it from a distance, with a distance sensor. The
sensor measures how far the wall is along its beam. Going back from the wall to the robot:

```
wall  →  along the beam, back to the sensor  →  from the sensor to the robot's center
x = wall_x - reading * beam_x - sensor_x
```

- `beam_x` is how much of each inch along the beam goes in the x direction: `sin` of the
  direction the sensor looks (the robot's heading, plus 0 for the front sensor, 90 for the
  right one, ...). Looking straight at the wall it is 1 (or -1 for the wall on the other
  side, which the minus sign handles), and at an angle it is less, because the beam travels
  further to reach the wall.
- `sensor_x` is where the sensor sits, measured from the robot's center
  (`DISTANCE_..._AHEAD` and `_RIGHT`), turned onto the field like a movement in odometry.

For a wall at `y = ...` it's the same with `cos` and `sensor_y`.

Some checks keep a bad reading out. Each one leaves the position alone and says why in the
terminal:
- **Still moving.** The sensor's reading is a little behind, so while the robot moves it belongs
  to where the robot was a moment ago. If either side's wheels go faster than
  `DISTANCE_RESET_MAX_SPEED`, nothing is changed. (Each side on its own: turning in place moves
  the sensor too, even though the average speed is 0.)
- **Too close.** The sensor can't measure under 20 mm, so a robot pressed against the wall
  should use `setX` / `setY` instead.
- **The wall behind the sensor.** If odometry says the wall is on the other side of the robot
  than the sensor looks, it's the wrong sensor or the wrong wall.
- **The angle.** When the sensor looks at the wall too much from the side (more than
  `DISTANCE_RESET_MAX_ANGLE`), the beam can hit something else, and the sensor measures less
  exactly. Then nothing is changed.
- **The size of the change.** If the wall says the position is more than
  `DISTANCE_RESET_MAX_CHANGE` inches off, the sensor most likely saw another robot or a game
  object, not the wall. Odometry is rarely that far off, so the reading is ignored. After a big
  crash, when it really can be, pass a bigger limit for that one reset:
  `resetXFromWall(RIGHT_SENSOR, 70, 12)`.

The distance sensor is most exact up close (about ±15 mm under 20 cm, about 5% further away),
so reset near the wall.

---

## 8. Driving to a point

`PID_drive_to_point(x, y)` uses odometry to drive to a field point:

1. **Turn first**: if the robot faces more than `POINT_TURN_FIRST_ANGLE` away from the point,
   it first turns towards it (a chained turn, so it doesn't stop in between).
2. **Drive**: the PID error is how far the point is **in front of** the robot (measured along
   the way it faces). If the robot drives past it, that becomes negative and it backs up.
3. **Keep aiming**: every loop the robot works out the heading to the point again and steers
   towards it, like the heading correction of `PID_forward`. So it still arrives if it gets
   bumped or one side is weaker.
4. **Slow down while not facing it**: the drive power is multiplied by `cos(aim error)`: full
   power when facing the point, none when facing 90° away.
5. **Close to the point**, it stops re-aiming (`POINT_AIM_DISTANCE`): there, a tiny position
   change would swing the aim around wildly.

It is finished when the point is (almost) 0 inches ahead, the robot has stopped, and it faces
the way it was aiming. The robot can't drive sideways, so a tiny sideways miss is accepted.

### Chaining points into a path

`PID_drive_to_point_chain(x, y, exit_range)` works like the other chained movements (section 6):
its PID aims `exit_range` inches past the point, and it hands over when the point is less than
`exit_range` ahead. Two extra rules make paths of points work:

- **It must roughly face the point** (within `POINT_TURN_FIRST_ANGLE`) before it hands over.
  Otherwise a point right beside the robot is "0 inches ahead" from the start, and the
  movement would end before the robot even moved.
- **It curves instead of turning first**: when a chained point follows another chained movement,
  the robot is still driving. Stopping to turn would waste that speed, so it skips the turn and
  lets the steering (and the `cos(aim error)` slow-down) curve it into the new direction. Only a
  point more than 90° around still gets a turn first. The last, normal movement of a path does
  turn first: driving straight into the point is what makes it stop exactly there.

---

## 9. Tuning, step by step

Tune turns first (they are easiest to see), then forward, then swings and arcs.
`examples/tuning` runs `tuneWithController()` with the live graph on the Brain screen.

1. **Start simple**: set kI and kD to 0.
2. **kP**: raise it until the robot gets to the target quickly but swings back and forth
   around it a few times. Too low: slow, stops short. Too high: wild swinging.
3. **kD**: raise it until the swinging stops: the overshoot (`ov` on the controller) gets close
   to 0. Too much kD makes the robot slow, or makes it shake with a buzzing sound.
4. **kI**, only if needed: does the robot stop a little short and stay there (`e` stays bigger
   than your tolerance)? Raise kI in small steps. Keep the integral range small (a few
   degrees or an inch or two).
5. **Test with other distances**: a gain that's great for 90° might overshoot at 180°. Try a
   small and a big movement.
6. **Keep the gains**: with an SD card in the Brain, B saves them in `pid_gains.txt`, and
   `loadGainsFromSDCard()` at the start of your program loads them in the next run. Without a
   card, copy them into `simpleV5LibConfig.h`: the tuner's changes are lost when the program stops.

### What the gains file remembers

Each line of `pid_gains.txt` holds the tuned gain and what `simpleV5LibConfig.h` said when it
was saved: `TURN_KP = 3.5 config 3.2`. That solves a confusing problem: you tune with the
controller, later change `TURN_KP` in the config by hand, and nothing happens, because the old
file on the card overrides it. When loading, a gain whose config value has changed since the
file was saved comes from the config: it is the newer decision.

### Logging to the SD card

`logToSDCard(true)` writes the same numbers as `logToTerminal`, plus where the robot was (x, y,
heading), into `pidlog1.csv`, `pidlog2.csv`, ... (a new file every program run). Then you can
see what went wrong in a match, where no USB cable is connected.

Writing to an SD card is slow: one write can take several milliseconds. If the motion task
wrote every line right away, each write would hold up the PID loop and the robot would steer
worse. So the lines wait in memory (`sd_buffer` in `telemetry.cpp`, room for about 10 seconds
of movements), and the motion task writes them out when no movement is running and the robot has
stopped (`telemetryWriteSDCard`). Only a single movement longer than the buffer is written in the
middle.

| What you see | What to change |
| --- | --- |
| Goes past the target and swings back | more kD, or less kP |
| Slow to get there | more kP |
| Stops just short of the target | more kI (or more kP; for turns, more `TURN_MIN_SPEED`) |
| Shakes or buzzes, power line jumps up and down | less kD |
| Movement times out | look at the graph: stopped short (kI) or still swinging (kD) |
| Turns go crazy, spinning faster and faster | `getGyroRate` has the wrong sign: run `testDrivetrain()` |

---

## 10. Setting up a new robot

Most "the PID doesn't work" problems are really setup problems. `examples/robotSetup` runs these
checks in order:

- **`checkDevices()`** asks every device from `simpleV5LibConfig.h` if it is plugged in
  (`installed()`), reads the motor temperatures (V5 motors lose power above 55 °C), and checks
  that no port is used twice.
- **`testDrivetrain()`** drives one side forward at a time and checks three things:
  - Every motor on that side must report turning **forward**. A motor that reports backwards
    has the wrong direction setting: it is fighting the other motors.
  - The motors on the other side must not move. If they do, ports of the two sides are mixed up.
  - The left side driving forward must turn the robot **clockwise** (the heading goes up), the
    right side counter-clockwise. If not, that side physically drives backwards, or left and
    right are swapped. While it turns, `getGyroRate()` must have the same sign as the heading
    change; if not, the D part of every turn would push instead of brake.
- **`measureTrackWidth()`** spins the robot in place 3 times. Each wheel rolls along a circle
  around the robot's center, so `inches rolled = distance from the center * angle turned`.
  The inertial sensor gives the angle, the motors give the inches, so the track width is
  `(left inches - right inches) / angle`. Wheels slide sideways a little while turning, so the
  result is often a bit more than the tape measure says, and that's the number arcs need.
  The tracking wheel offsets come out the same way. The inches are counted with the wheel
  size, so a wrong wheel size makes the track width wrong too: run `measureWheelSize()` first,
  and `measureTrackWidth()` uses what it measured.
- **`measureWheelSize(48)`**: you push the robot exactly 48 inches by hand. If the library
  counted 47 inches, the wheels are really `48 / 47` times as big as the config says. A
  wrong `MOTOR_TO_WHEEL_GEAR_RATIO` shows up here too, as a wheel size that is way off.

---

## 11. Mechanisms: an arm that holds, an intake that unjams

Code: `src/mechanisms.cpp`. Example: `examples/mechanisms`.

### One background task for all of them

An `Arm` or `Intake` is made outside any function, next to its motors. That registers it with
the library, but doesn't start anything yet: it is too early to start a task before `main()`
runs. The first command (`moveTo`, `spin`, ...) starts one background task, which looks after
every Arm and Intake every 10 ms.

Your code never moves the motors itself. `moveTo` only writes down the new target and adds one
to a counter (`request`). The task sees that the counter changed and starts the new command.
So there is only ever one piece of code telling the motors what to do, and your code never
has to wait. It is the same idea as the motion task in section 5.

Driver control calls the same command every 20 ms (`arm.moveTo(ARM_SCORE)` for as long as
the button is held, `intake.spin(100)` for as long as R1 is). Starting over every time would
reset the PID and the jam timer 50 times a second. So a command that is the same as the last
one is ignored.

### Arm: move with PID, hold with the motor

`moveTo` is a PID loop like `PID_turn`, but in degrees of the motor: the error is
`target - position()`, the D part brakes with the motor's speed. It counts as there when it
stays within `ARM_TOLERANCE` for `ARM_SETTLE_MS`, like section 2.

Then it stops the motor with `brakeType::hold`. In hold mode the motor runs its own position
control, inside the motor, and pushes back whenever something moves it away. That is what
keeps a heavy arm up, and it costs nothing to tune. If the arm doesn't get there within
`ARM_TIMEOUT_MS` (stuck on something), it holds wherever it got to, and `waitUntilDone()`
returns false.

Gravity is the hard part of moving an arm. While going down, gravity helps, and P alone stops
where its push up is just as strong as gravity: below the target, by `gravity / kP` degrees.
Then I slowly pushes it the rest of the way. So for a heavy arm:

- `ARM_INTEGRAL_RANGE` must be bigger than that sag, or I never starts.
- A bigger kP sags less, but too big makes it swing on the way up.

`manual(power)` is for driver control. A stick that isn't at 0 drives the arm; letting go
(power 0, or inside `DRIVE_DEADBAND`) holds it right where it is. A 0 while a `moveTo` is
running is ignored, so a stick at rest doesn't stop a button's `moveTo`.

`setLimits` stops the arm at its lowest and highest position. `moveTo` targets are clamped
into the range. With the stick, the arm holds when it reaches a limit. It keeps holding for as
long as the stick pushes that way, even if the held arm sags back a little below the limit.
Otherwise it would drive up, hold, sag, drive up, ... many times a second.

### Intake: when is it jammed?

A motor that is told to spin but can't turn draws a lot of current. Here is why:
A spinning motor makes a voltage of its own that works against the battery
(a motor is also a generator), and the faster it spins, the less current flows. Stalled, that
counter-voltage is gone. So a jam is:

- more than `INTAKE_JAM_CURRENT` amps per motor (`current()` of a `motor_group` is all its
  motors together, so it is divided by `count()`), **and**
- slower than `INTAKE_JAM_SPEED` percent, **and**
- both for at least `INTAKE_JAM_MS`.

Each condition alone gets it wrong:

- Starting up also draws a lot of current at low speed, for a moment: that's why it must last
  `INTAKE_JAM_MS`.
- A heavy but still turning intake draws a lot too: that's why it must also be slow.
- An intake running slowly on purpose is slow: that's why the current must be high.

The downside of the current rule: the less power the motor is given, the less current it draws
when stalled, so a jam at low power can go unnoticed.

When it jams, `spin(power)` runs the intake the other way at the same power for
`INTAKE_UNJAM_MS`, to let the stuck piece go, then forward again. If it is still stuck, it
jams again, and backs off again. With `spin(power, false)` it stops instead and `isJammed()`
stays true until you give it a new command. "Can't turn any more" is also what happens when
a game piece is all the way in, so that's a way to know it is.
