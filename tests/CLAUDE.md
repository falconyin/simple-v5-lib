# Writing and running tests

There is no test framework and no per-test selection: `tests/simulation_test.cpp` is one `testX()`
function per feature, full of `check(name, ok, a, b)` calls that print PASS/FAIL, and a `main()` that
calls them in order and exits with the failure count. The order matters: every test starts where the
previous one left the simulated robot (and the noise sequence), and the wheel-size tests must stay
last. Add a new test to the function for its feature, or a new function called from `main()`. To run
one group, comment out the other calls in `main()` (later tests may then fail, since they start
from a different state) and build that file directly:

```sh
g++ -std=c++17 -O1 -pthread -I include -I tests/sim src/*.cpp tests/simulation_test.cpp -o build/t && ./build/t
```

`-DSIM_REALISTIC` turns on the simulator's real-world layer (`sim::real`, also `sim::realistic(true)` or one
knob at a time): sensor readings and motor commands 10 ms late (`History` ring buffers, one entry per
simulated ms), ±10 dps noise on `velocity()` and ±2 deg/s on `gyroRate()` (deterministic LCG), an IMU that
drifts 1°/min, an 11 V battery, a traction limit (`grip`, 250 in/s²: the wheel spins faster than the robot
accelerates, and the encoder counts the wheel) and a 2 s IMU calibration. The realistic runs are what catch
skid-related odometry errors and stale-reading overshoots; the perfect runs stay strict. Consequences:
- A test that reads what the program *commanded* must read `sim::cmd_in` (`.cmd`, `.mode`, `.mech_mode`), not
  `sim::cmd`/`sim::mode`/`sim::mech_mode`, which are what reaches the motors after the command delay.
- A test that teleports the robot (`sim::x = ...`) must call `sim::forgetReadings()` or the sensors report the
  old place for 10 ms. `push()`/`bump()` do it themselves.
- Tolerances that a real robot can't meet (dead-reckoning exactness after slip, true position after a stale
  settle) are widened with `sim::isRealistic()` in the test, with a comment saying why, never silently.
- Noise is deterministic but its sequence shifts when tests before it change; a realistic check that passes by
  a hair will flake later, so leave real margin.

New tests are mutation-checked: break the code on purpose and confirm the test fails (the first tests for
chained drive-to-point missed 5 of 6 such mutations). Timing traps in the cooperative simulator:
- To test the per-movement gain snapshot, change the gains right after the `_async` call; any later and
  the movement has already copied them.
- Back-to-back chained calls never reach `motionLoop`'s idle branch. To test something that runs only
  while idle (SD-card writes), put user code such as `vexDelay(40)` between them.
- A chained movement hands over while the robot is still rolling, so compute expected values (e.g. a
  `turn_to_point_chain` heading) from the pose at hand-over, not from before the previous movement.
- A test that stops tasks (`competition::test_disable`, `sim::stopTasks`) should wait with a bounded loop
  (`for (i < N && !arm.isDone()) vexDelay(10)`) rather than `waitUntilDone`, and let the waiting happen in
  a task of its own (the simulated `usercontrol`), so a library that doesn't restart its task fails a
  check instead of hanging. Where the blocking call is the point (main waiting in `PID_forward`), a
  regression hangs; CI's 10-minute job timeout turns that into a failure.
